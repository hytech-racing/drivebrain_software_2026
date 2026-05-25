#include "htx_ekf/ekf_manager.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "htx_ekf/ekf.hpp"
#include "htx_ekf/math_utils.hpp"
#include "htx_ekf/state.hpp"

namespace htx_ekf
{

EkfManager::EkfManager(const EkfManagerConfig& config)
{
    load_config(config);
    initialize_ekf();
}

void EkfManager::load_config(const EkfManagerConfig& config)
{
    init_std_pn_ = config.init_std_pn;
    init_std_pe_ = config.init_std_pe;
    init_std_yaw_ = config.init_std_yaw;
    init_std_vx_ = config.init_std_vx;
    init_std_vy_ = config.init_std_vy;
    init_std_alpha_ = config.init_std_alpha;
    init_std_bg_ = config.init_std_bg;
    init_std_bax_ = config.init_std_bax;
    init_std_bay_ = config.init_std_bay;

    alpha_alignment_min_speed_ = config.alpha_alignment_min_speed;

    gnss1_offset_x_vehicle_frd_ = config.gnss1_offset_x_vehicle_frd;
    gnss1_offset_y_vehicle_frd_ = config.gnss1_offset_y_vehicle_frd;
    gnss2_offset_x_vehicle_frd_ = config.gnss2_offset_x_vehicle_frd;
    gnss2_offset_y_vehicle_frd_ = config.gnss2_offset_y_vehicle_frd;

    zero_vel_update_sigma_ = config.zero_vel_update_sigma;

    gnss1_speed_update_sigma_floor_ = config.gnss1_speed_update_sigma_floor;
    gnss1_vel_update_sigma_floor_ = config.gnss1_vel_update_sigma_floor;
    gnss1_pos_update_sigma_floor_ = config.gnss1_pos_update_sigma_floor;
    gnss2_speed_update_sigma_floor_ = config.gnss2_speed_update_sigma_floor;
    gnss2_vel_update_sigma_floor_ = config.gnss2_vel_update_sigma_floor;
    gnss2_pos_update_sigma_floor_ = config.gnss2_pos_update_sigma_floor;

    use_gnss_reported_uncertainty_ = config.use_gnss_reported_uncertainty;

    zero_lat_base_sigma_ = config.zero_lat_base_sigma;
    zero_lat_disabled_sigma_ = config.zero_lat_disabled_sigma;
    zero_lat_yaw_rate_gain_ = config.zero_lat_yaw_rate_gain;
    zero_lat_lateral_accel_gain_ = config.zero_lat_lateral_accel_gain;
    zero_lat_current_sigma_ = zero_lat_base_sigma_;

    zero_vel_confidence_level_ = config.zero_vel_confidence_level;
    zero_lat_velocity_confidence_level_ =
        config.zero_lat_velocity_confidence_level;
    gnss_speed_magnitude_confidence_level_ =
        config.gnss_speed_magnitude_confidence_level;
    gnss_velocity_ned_confidence_level_ =
        config.gnss_velocity_ned_confidence_level;
    gnss_pos_ned_confidence_level_ = config.gnss_pos_ned_confidence_level;

    imu_stationary_required_samples_ = config.imu_stationary_required_samples;
    gnss1_stationary_required_samples_ =
        config.gnss1_stationary_required_samples;
    gnss2_stationary_required_samples_ =
        config.gnss2_stationary_required_samples;

    stationary_gyro_z_threshold_ = config.stationary_gyro_z_threshold;
    stationary_accel_xy_threshold_ = config.stationary_accel_xy_threshold;

    zero_lat_min_speed_ = config.zero_lat_min_speed;
    gnss_velocity_ned_min_speed_ = config.gnss_velocity_ned_min_speed;

    gnss_stale_timeout_s_ = config.gnss_stale_timeout_s;

    load_ekf_proc_config(config.proc);

    spdlog::info(
        "Loaded EKF config: q_vx={}, "
        "q_vy={}, "
        "use_gnss_uncertainty={}",
        config.proc.q_std_vx, config.proc.q_std_vy,
        use_gnss_reported_uncertainty_);
}

void EkfManager::load_ekf_proc_config(const EkfProcConfig& config)
{
    ekf_.set_proc_config(config);
    spdlog::info(
        "EKF proc config: q_vx={}, "
        "q_vy={}, q_bg={}",
        config.q_std_vx, config.q_std_vy, config.q_std_bg);
}

StateVec EkfManager::make_initial_state()
{
    StateVec x0;
    x0.setZero();

    return x0;
}

StateMat EkfManager::make_initial_covariance()
{
    StateMat P0;
    P0.setZero();

    P0(StateIndex::PN, StateIndex::PN) =
        init_std_pn_ * init_std_pn_;  // PN initial
                                      // variance
    P0(StateIndex::PE, StateIndex::PE) =
        init_std_pe_ * init_std_pe_;  // PE initial
                                      // variance
    P0(StateIndex::YAW,
       StateIndex::YAW) = init_std_yaw_ * init_std_yaw_;  // YAW initial
                                                          // variance
    P0(StateIndex::VX, StateIndex::VX) =
        init_std_vx_ * init_std_vx_;  // VX initial
                                      // variance
    P0(StateIndex::VY, StateIndex::VY) =
        init_std_vy_ * init_std_vy_;  // VY initial
                                      // variance
    P0(StateIndex::ALPHA,
       StateIndex::ALPHA) = init_std_alpha_ * init_std_alpha_;  // ALPHA
                                                                // initial
                                                                // variance
    P0(StateIndex::BG, StateIndex::BG) =
        init_std_bg_ * init_std_bg_;  // BG initial
                                      // variance
    P0(StateIndex::BAX,
       StateIndex::BAX) = init_std_bax_ * init_std_bax_;  // BAX initial
                                                          // variance
    P0(StateIndex::BAY,
       StateIndex::BAY) = init_std_bay_ * init_std_bay_;  // BAY initial
                                                          // variance

    return P0;
}

void EkfManager::initialize_ekf()
{
    StateVec x0 = make_initial_state();
    StateMat P0 = make_initial_covariance();

    ekf_.reset(x0, P0);
}

EkfStepResult EkfManager::handle_imu(const ImuSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EkfStepResult result;

    if (!last_imu_time_startup_ns_)
    {
        last_imu_time_startup_ns_ = sample.time_startup_ns;

        if (should_publish_debug_snapshot(sample.time_startup_ns))
        {
            update_debug_snapshot();
            result.has_debug_snapshot = true;
            result.debug_snapshot = latest_debug_snapshot_;
        }

        return result;
    }

    last_imu_time_startup_ns_ = sample.time_startup_ns;

    if (!imu_stationary_init_done_)
    {
        handle_imu_stationary_init(sample);

        if (should_publish_debug_snapshot(sample.time_startup_ns))
        {
            update_debug_snapshot();
            result.has_debug_snapshot = true;
            result.debug_snapshot = latest_debug_snapshot_;
        }

        return result;
    }

    const double dt_s = sample.dt_s;
    last_imu_dt_s_ = dt_s;

    if (dt_s <= 0.0 || dt_s >= 0.02)
    {
        if (should_publish_debug_snapshot(sample.time_startup_ns))
        {
            update_debug_snapshot();
            result.has_debug_snapshot = true;
            result.debug_snapshot = latest_debug_snapshot_;
        }
        return result;
    }

    const StateVec& x = ekf_.state();

    corrected_yaw_rate_vehicle_frd_ = sample.gz_frd_rad_s - x(StateIndex::BG);
    corrected_accel_x_vehicle_frd_ = sample.ax_frd_m_s2 - x(StateIndex::BAX);
    corrected_accel_y_vehicle_frd_ = sample.ay_frd_m_s2 - x(StateIndex::BAY);

    // use raw sample here
    ekf_.predict_imu(sample);

    update_ekf_output();
    result.has_state = true;
    result.state = latest_output_;

    if (should_publish_debug_snapshot(sample.time_startup_ns))
    {
        update_debug_snapshot();
        result.has_debug_snapshot = true;
        result.debug_snapshot = latest_debug_snapshot_;
    }

    return result;
}

EkfStepResult EkfManager::handle_gnss(const DualGnssSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EkfStepResult result;

    // TODO: port GNSS stationary init,
    // alpha course-over-ground
    // alignment, GNSS speed, GNSS
    // velocity NED, and GNSS position
    // NED updates.
    (void)sample;
    if (should_publish_debug_snapshot(sample.vn_time_startup_ns))
    {
        update_debug_snapshot();
        result.has_debug_snapshot = true;
        result.debug_snapshot = latest_debug_snapshot_;
    }
    return result;
}

EkfStepResult EkfManager::handle_ins(const InsSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EkfStepResult result;
    // TODO: port INS
    // correction/supervisor logic
    // later.

    (void)sample;

    if (should_publish_debug_snapshot(sample.vn_time_startup_ns))
    {
        update_debug_snapshot();
        result.has_debug_snapshot = true;
        result.debug_snapshot = latest_debug_snapshot_;
    }

    return result;
}

EkfOutput EkfManager::get_latest_output() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_output_;
}

void EkfManager::handle_imu_stationary_init(const ImuSample& sample)
{
    if (imu_stationary_init_done_)
    {
        return;
    }

    if (imu_stationary_sample_count_ < imu_stationary_required_samples_)
    {
        imu_stationary_sample_count_++;

        gz_sum_ += sample.gz_frd_rad_s;
        ax_sum_ += sample.ax_frd_m_s2;
        ay_sum_ += sample.ay_frd_m_s2;

        if (imu_stationary_sample_count_ % 120 == 0)
        {
            spdlog::info(
                "IMU stationary init "
                "progress: {}%",
                static_cast<float>(imu_stationary_sample_count_) /
                    static_cast<float>(imu_stationary_required_samples_) *
                    100.0);
        }

        return;
    }

    const double count = static_cast<double>(imu_stationary_sample_count_);

    const double bg = gz_sum_ / count;
    const double bax = ax_sum_ / count;
    const double bay = ay_sum_ / count;

    StateVec x0 = make_initial_state();
    x0(StateIndex::BG) = bg;
    x0(StateIndex::BAX) = bax;
    x0(StateIndex::BAY) = bay;

    StateMat P0 = make_initial_covariance();

    ekf_.reset(x0, P0);

    imu_stationary_init_done_ = true;

    const UpdateResult zero_vel_update_result = ekf_.update_zero_vel(
        zero_vel_update_sigma_, zero_vel_confidence_level_);

    update_ekf_output();
    update_debug_statuses(MeasurementId::ZERO_VEL, zero_vel_update_result);

    spdlog::info(
        "IMU stationary init complete: "
        "bg={:.6f}, bax={:.6f}, "
        "bay={:.6f}, "
        "zero_vel accepted={}",
        bg, bax, bay, zero_vel_update_result.accepted);
}

void EkfManager::update_ekf_output()
{
    const StateVec& x = ekf_.state();

    latest_output_.time_startup_ns = last_imu_time_startup_ns_.value_or(0);

    latest_output_.ax_corrected_vehicle_frd_m_s2 =
        corrected_accel_x_vehicle_frd_;
    latest_output_.ay_corrected_vehicle_frd_m_s2 =
        corrected_accel_y_vehicle_frd_;
    latest_output_.yaw_rate_corrected_vehicle_frd_rad_s =
        corrected_yaw_rate_vehicle_frd_;

    latest_output_.ax_corrected_vehicle_flu_m_s2 =
        corrected_accel_x_vehicle_frd_;
    latest_output_.ay_corrected_vehicle_flu_m_s2 =
        corrected_accel_y_vehicle_frd_ * -1.0;
    latest_output_.yaw_rate_corrected_vehicle_flu_rad_s =
        corrected_yaw_rate_vehicle_frd_ * -1.0;

    latest_output_.position_n_m = x(StateIndex::PN);
    latest_output_.position_e_m = x(StateIndex::PE);

    // latest_output_.yaw_sensor_frd_rad = yaw_sensor;
    latest_output_.yaw_vehicle_frd_rad = wrap_angle(x(StateIndex::YAW));
    latest_output_.yaw_vehicle_flu_rad = wrap_angle(-x(StateIndex::YAW));

    latest_output_.alpha_startup_to_ned_rad = wrap_angle(x(StateIndex::ALPHA));

    latest_output_.vx_vehicle_frd_m_s = x(StateIndex::VX);
    latest_output_.vy_vehicle_frd_m_s = x(StateIndex::VY);

    latest_output_.vx_vehicle_flu_m_s = x(StateIndex::VX);
    latest_output_.vy_vehicle_flu_m_s = x(StateIndex::VY) * -1.0;

    latest_output_.bg_rad_s = x(StateIndex::BG);
    latest_output_.bax_m_s2 = x(StateIndex::BAX);
    latest_output_.bay_m_s2 = x(StateIndex::BAY);

    latest_output_.initialized = imu_stationary_init_done_;

    latest_output_.last_imu_dt_s = last_imu_dt_s_;
}

void EkfManager::update_debug_statuses(int meas_id,
                                       const UpdateResult& update_result)
{
    switch (meas_id)
    {
        case MeasurementId::ZERO_VEL:

            zero_vel_update_enabled_ = update_result.attempted;
            zero_vel_update_accepted_ = update_result.accepted;
            zero_vel_nis_ = update_result.nis;
            zero_vel_gate_ = update_result.threshold;

            break;

        case MeasurementId::ZERO_LAT_VEL:

            zero_lat_vel_update_enabled_ = update_result.attempted;
            zero_lat_vel_update_accepted_ = update_result.accepted;
            zero_lat_vel_nis_ = update_result.nis;
            zero_lat_vel_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS1_SPEED_MAG:

            gnss1_speed_update_enabled_ = update_result.attempted;
            gnss1_speed_update_accepted_ = update_result.accepted;
            gnss1_speed_nis_ = update_result.nis;
            gnss1_speed_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS1_VEL_NED:

            gnss1_vel_update_enabled_ = update_result.attempted;
            gnss1_vel_update_accepted_ = update_result.accepted;
            gnss1_vel_nis_ = update_result.nis;
            gnss1_vel_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS1_POS_NED:

            gnss1_pos_update_enabled_ = update_result.attempted;
            gnss1_pos_update_accepted_ = update_result.accepted;
            gnss1_pos_nis_ = update_result.nis;
            gnss1_pos_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS2_SPEED_MAG:

            gnss2_speed_update_enabled_ = update_result.attempted;
            gnss2_speed_update_accepted_ = update_result.accepted;
            gnss2_speed_nis_ = update_result.nis;
            gnss2_speed_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS2_VEL_NED:

            gnss2_vel_update_enabled_ = update_result.attempted;
            gnss2_vel_update_accepted_ = update_result.accepted;
            gnss2_vel_nis_ = update_result.nis;
            gnss2_vel_gate_ = update_result.threshold;

            break;

        case MeasurementId::GNSS2_POS_NED:

            gnss2_pos_update_enabled_ = update_result.attempted;
            gnss2_pos_update_accepted_ = update_result.accepted;
            gnss2_pos_nis_ = update_result.nis;
            gnss2_pos_gate_ = update_result.threshold;

            break;

        default:

            spdlog::info("Unknown measurement id");

            break;
    }
}

bool EkfManager::should_publish_debug_snapshot(uint64_t vn_time_startup_ns)
{
    if (!last_debug_snapshot_publish_time_ns_.has_value())
    {
        last_debug_snapshot_publish_time_ns_ = vn_time_startup_ns;
        return true;
    }

    const uint64_t elapsed_ns =
        vn_time_startup_ns - last_debug_snapshot_publish_time_ns_.value();

    if (elapsed_ns >= debug_snapshot_period_ns_)
    {
        last_debug_snapshot_publish_time_ns_ = vn_time_startup_ns;
        return true;
    }

    return false;
}

InitStatus EkfManager::compute_imu_status() const
{
    if (imu_stationary_init_done_)
    {
        return InitStatus::Done;
    }

    if (imu_stationary_sample_count_ > 0)
    {
        return InitStatus::InProgress;
    }

    return InitStatus::Waiting;

    // maybe add a failed logic later, for now this is fine
}

InitStatus EkfManager::compute_gnss1_status() const
{
    if (gnss1_stationary_init_done_)
    {
        return InitStatus::Done;
    }

    if (gnss1_stationary_sample_count_ > 0)
    {
        return InitStatus::InProgress;
    }

    return InitStatus::Waiting;
}

InitStatus EkfManager::compute_gnss2_status() const
{
    if (gnss2_stationary_init_done_)
    {
        return InitStatus::Done;
    }

    if (gnss2_stationary_sample_count_ > 0)
    {
        return InitStatus::InProgress;
    }

    return InitStatus::Waiting;
}

InitStatus EkfManager::compute_alpha_status() const
{
    if (alpha_course_aligned_)
    {
        return InitStatus::Done;
    }

    return InitStatus::Waiting;
}

double EkfManager::progress_ratio(std::size_t count, std::size_t required)
{
    if (required == 0)
    {
        return 0.0;
    }

    const double ratio =
        static_cast<double>(count) / static_cast<double>(required);

    return std::min(1.0, ratio);
}

void EkfManager::update_debug_snapshot()
{
    latest_debug_snapshot_.time_startup_ns =
        last_imu_time_startup_ns_.value_or(0);

    latest_debug_snapshot_.imu_status = compute_imu_status();
    latest_debug_snapshot_.gnss1_status = compute_gnss1_status();
    latest_debug_snapshot_.gnss2_status = compute_gnss2_status();
    latest_debug_snapshot_.alpha_status = compute_alpha_status();

    latest_debug_snapshot_.imu_sample_count = imu_stationary_sample_count_;
    latest_debug_snapshot_.imu_required_samples =
        imu_stationary_required_samples_;
    latest_debug_snapshot_.imu_init_progress = progress_ratio(
        imu_stationary_sample_count_, imu_stationary_required_samples_);

    latest_debug_snapshot_.gnss1_sample_count = gnss1_stationary_sample_count_;
    latest_debug_snapshot_.gnss1_required_samples =
        gnss1_stationary_required_samples_;
    latest_debug_snapshot_.gnss1_init_progress = progress_ratio(
        gnss1_stationary_sample_count_, gnss1_stationary_required_samples_);

    latest_debug_snapshot_.gnss2_sample_count = gnss2_stationary_sample_count_;
    latest_debug_snapshot_.gnss2_required_samples =
        gnss2_stationary_required_samples_;
    latest_debug_snapshot_.gnss2_init_progress = progress_ratio(
        gnss2_stationary_sample_count_, gnss2_stationary_required_samples_);

    latest_debug_snapshot_.stationary_detected = stationary_detected_;

    latest_debug_snapshot_.latest_zero_vel_update_enabled =
        zero_vel_update_enabled_;
    latest_debug_snapshot_.latest_zero_lat_vel_update_enabled =
        zero_lat_vel_update_enabled_;
    latest_debug_snapshot_.latest_gnss1_speed_update_enabled =
        gnss1_speed_update_enabled_;
    latest_debug_snapshot_.latest_gnss1_vel_update_enabled =
        gnss1_vel_update_enabled_;
    latest_debug_snapshot_.latest_gnss1_pos_update_enabled =
        gnss1_pos_update_enabled_;
    latest_debug_snapshot_.latest_gnss2_speed_update_enabled =
        gnss2_speed_update_enabled_;
    latest_debug_snapshot_.latest_gnss2_vel_update_enabled =
        gnss2_vel_update_enabled_;
    latest_debug_snapshot_.latest_gnss2_pos_update_enabled =
        gnss2_pos_update_enabled_;

    latest_debug_snapshot_.latest_zero_vel_update_accepted =
        zero_vel_update_accepted_;
    latest_debug_snapshot_.latest_zero_lat_vel_update_accepted =
        zero_lat_vel_update_accepted_;
    latest_debug_snapshot_.latest_gnss1_speed_update_accepted =
        gnss1_speed_update_accepted_;
    latest_debug_snapshot_.latest_gnss1_vel_update_accepted =
        gnss1_vel_update_accepted_;
    latest_debug_snapshot_.latest_gnss1_pos_update_accepted =
        gnss1_pos_update_accepted_;
    latest_debug_snapshot_.latest_gnss2_speed_update_accepted =
        gnss2_speed_update_accepted_;
    latest_debug_snapshot_.latest_gnss2_vel_update_accepted =
        gnss2_vel_update_accepted_;
    latest_debug_snapshot_.latest_gnss2_pos_update_accepted =
        gnss2_pos_update_accepted_;

    latest_debug_snapshot_.latest_zero_vel_nis = zero_vel_nis_;
    latest_debug_snapshot_.latest_zero_vel_gate = zero_vel_gate_;

    latest_debug_snapshot_.latest_zero_lat_vel_nis = zero_lat_vel_nis_;
    latest_debug_snapshot_.latest_zero_lat_vel_gate = zero_lat_vel_gate_;

    latest_debug_snapshot_.latest_gnss1_speed_nis = gnss1_speed_nis_;
    latest_debug_snapshot_.latest_gnss1_speed_gate = gnss1_speed_gate_;
    latest_debug_snapshot_.latest_gnss1_vel_nis = gnss1_vel_nis_;
    latest_debug_snapshot_.latest_gnss1_vel_gate = gnss1_vel_gate_;
    latest_debug_snapshot_.latest_gnss1_pos_nis = gnss1_pos_nis_;
    latest_debug_snapshot_.latest_gnss1_pos_gate = gnss1_pos_gate_;

    latest_debug_snapshot_.latest_gnss2_speed_nis = gnss2_speed_nis_;
    latest_debug_snapshot_.latest_gnss2_speed_gate = gnss2_speed_gate_;
    latest_debug_snapshot_.latest_gnss2_vel_nis = gnss2_vel_nis_;
    latest_debug_snapshot_.latest_gnss2_vel_gate = gnss2_vel_gate_;
    latest_debug_snapshot_.latest_gnss2_pos_nis = gnss2_pos_nis_;
    latest_debug_snapshot_.latest_gnss2_pos_gate = gnss2_pos_gate_;

    latest_debug_snapshot_.latest_zero_lat_vel_sigma_used =
        zero_lat_vel_sigma_used_;

    latest_debug_snapshot_.latest_gnss1_speed_sigma_used =
        gnss1_speed_sigma_used_;
    latest_debug_snapshot_.latest_gnss1_vel_sigma_used = gnss1_vel_sigma_used_;
    latest_debug_snapshot_.latest_gnss1_pos_sigma_used = gnss1_pos_sigma_used_;

    latest_debug_snapshot_.latest_gnss2_speed_sigma_used =
        gnss2_speed_sigma_used_;
    latest_debug_snapshot_.latest_gnss2_vel_sigma_used = gnss2_vel_sigma_used_;
    latest_debug_snapshot_.latest_gnss2_pos_sigma_used = gnss2_pos_sigma_used_;
}

}  // namespace htx_ekf