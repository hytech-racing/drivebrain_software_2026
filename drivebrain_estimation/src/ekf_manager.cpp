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

void EkfManager::hard_reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    initialize_ekf();

    imu_stationary_init_done_ = false;
    gnss1_stationary_init_done_ = false;
    gnss2_stationary_init_done_ = false;

    ax_sum_ = 0.0;
    ay_sum_ = 0.0;
    gz_sum_ = 0.0;

    imu_stationary_sample_count_ = 0;
    gnss1_stationary_sample_count_ = 0;
    gnss2_stationary_sample_count_ = 0;

    alpha_course_aligned_ = false;
    gnss_initial_heading_ned_ = 0.0;
    last_imu_dt_s_ = 0.0;

    stationary_detected_ = false;

    gnss1_origin_lla_.setZero();
    gnss2_origin_lla_.setZero();

    last_imu_time_startup_ns_.reset();
    last_dual_gnss_time_startup_ns_.reset();
    last_debug_snapshot_publish_time_ns_.reset();

    last_gnss1_pos_valid_ = false;
    last_gnss1_speed_valid_ = false;
    last_gnss1_vel_valid_ = false;

    last_gnss2_pos_valid_ = false;
    last_gnss2_speed_valid_ = false;
    last_gnss2_vel_valid_ = false;

    zero_vel_update_enabled_ = false;
    zero_vel_update_accepted_ = false;

    zero_lat_vel_update_enabled_ = false;
    zero_lat_vel_update_accepted_ = false;

    gnss1_speed_update_enabled_ = false;
    gnss1_speed_update_accepted_ = false;
    gnss1_vel_update_enabled_ = false;
    gnss1_vel_update_accepted_ = false;
    gnss1_pos_update_enabled_ = false;
    gnss1_pos_update_accepted_ = false;

    gnss2_speed_update_enabled_ = false;
    gnss2_speed_update_accepted_ = false;
    gnss2_vel_update_enabled_ = false;
    gnss2_vel_update_accepted_ = false;
    gnss2_pos_update_enabled_ = false;
    gnss2_pos_update_accepted_ = false;

    zero_lat_vel_residual_ = 0.0;

    gnss1_speed_residual_ = 0.0;
    gnss1_vel_ned_n_residual_ = 0.0;
    gnss1_vel_ned_e_residual_ = 0.0;
    gnss1_pos_ned_n_residual_ = 0.0;
    gnss1_pos_ned_e_residual_ = 0.0;

    gnss2_speed_residual_ = 0.0;
    gnss2_vel_ned_n_residual_ = 0.0;
    gnss2_vel_ned_e_residual_ = 0.0;
    gnss2_pos_ned_n_residual_ = 0.0;
    gnss2_pos_ned_e_residual_ = 0.0;

    last_gnss1_speed_ = 0.0;
    last_gnss1_vn_ = 0.0;
    last_gnss1_ve_ = 0.0;
    last_gnss1_vd_ = 0.0;
    last_gnss1_pn_ = 0.0;
    last_gnss1_pe_ = 0.0;

    last_gnss2_speed_ = 0.0;
    last_gnss2_vn_ = 0.0;
    last_gnss2_ve_ = 0.0;
    last_gnss2_vd_ = 0.0;
    last_gnss2_pn_ = 0.0;
    last_gnss2_pe_ = 0.0;

    zero_vel_nis_ = 0.0;
    zero_lat_vel_nis_ = 0.0;

    gnss1_speed_nis_ = 0.0;
    gnss1_vel_nis_ = 0.0;
    gnss1_pos_nis_ = 0.0;

    gnss2_speed_nis_ = 0.0;
    gnss2_vel_nis_ = 0.0;
    gnss2_pos_nis_ = 0.0;

    gnss1_speed_gate_ = 0.0;
    gnss1_vel_gate_ = 0.0;
    gnss1_pos_gate_ = 0.0;

    gnss2_speed_gate_ = 0.0;
    gnss2_vel_gate_ = 0.0;
    gnss2_pos_gate_ = 0.0;

    zero_lat_vel_sigma_used_ = 0.0;

    gnss1_speed_sigma_used_ = 0.0;
    gnss1_vel_sigma_used_ = 0.0;
    gnss1_pos_sigma_used_ = 0.0;

    gnss2_speed_sigma_used_ = 0.0;
    gnss2_vel_sigma_used_ = 0.0;
    gnss2_pos_sigma_used_ = 0.0;

    zero_lat_current_sigma_ = zero_lat_base_sigma_;

    corrected_accel_x_vehicle_frd_ = 0.0;
    corrected_accel_y_vehicle_frd_ = 0.0;
    corrected_yaw_rate_vehicle_frd_ = 0.0;

    update_ekf_output();
    update_debug_snapshot();

    spdlog::warn("EKF hard reset requested and performed");
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

    stationary_detected_ = is_stationary();

    UpdateResult stationary_update_result;
    if (stationary_detected_)
    {
        stationary_update_result = ekf_.update_zero_vel(
            zero_vel_update_sigma_, zero_vel_confidence_level_);
    }
    update_ekf_output();
    result.has_state = true;
    result.state = latest_output_;
    update_debug_statuses(MeasurementId::ZERO_VEL, stationary_update_result);

    zero_lat_current_sigma_ = compute_zero_lat_sigma(sample);
    zero_lat_vel_sigma_used_ = zero_lat_current_sigma_;

    zero_lat_update_enabled_ =
        zero_lat_current_sigma_ < zero_lat_disabled_sigma_;

    bool zero_lat_update_accepted = false;

    UpdateResult zero_lat_update_result;
    if (zero_lat_update_enabled_)
    {
        zero_lat_update_result = ekf_.update_zero_lat_velocity(
            zero_lat_current_sigma_, zero_lat_velocity_confidence_level_);
        zero_lat_update_accepted = zero_lat_update_result.accepted;
    }
    update_ekf_output();
    result.has_state = true;
    result.state = latest_output_;
    update_debug_statuses(MeasurementId::ZERO_LAT_VEL, zero_lat_update_result);

    // check whether gnss has gone stale
    check_gnss_stale_status();

    // Decide whether to send debug snapshot at specified rate
    if (should_publish_debug_snapshot(sample.time_startup_ns))
    {
        update_debug_snapshot();
        result.has_debug_snapshot = true;
        result.debug_snapshot = latest_debug_snapshot_;
    }

    return result;
}

bool EkfManager::handle_single_gnss(EkfStepResult& result, GnssSample& sample,
                                    double gnss_speed_sigma,
                                    double gnss_vel_sigma,
                                    double gnss_pos_sigma, int antenna_id)
{
    determine_gnss_validities(sample, antenna_id);

    int speed_meas_id = 0;
    int vel_meas_id = 0;
    int pos_meas_id = 0;
    bool gnss_init_done = false;
    double gnss_offset_x_vehicle_frd = 0.0;
    double gnss_offset_y_vehicle_frd = 0.0;
    Eigen::Vector3d gnss_pos_origin_lla = Eigen::Vector3d::Zero();

    UpdateResult speed_update_result;
    UpdateResult vel_update_result;
    UpdateResult pos_update_result;

    UpdateResult default_false_result;

    if (antenna_id == AntennaId::GNSS1)
    {
        speed_meas_id = MeasurementId::GNSS1_SPEED_MAG;
        vel_meas_id = MeasurementId::GNSS1_VEL_NED;
        pos_meas_id = MeasurementId::GNSS1_POS_NED;
        gnss_init_done = gnss1_stationary_init_done_;
        gnss_pos_origin_lla = gnss1_origin_lla_;
        gnss_offset_x_vehicle_frd = gnss1_offset_x_vehicle_frd_;
        gnss_offset_y_vehicle_frd = gnss1_offset_y_vehicle_frd_;
    }
    else if (antenna_id == AntennaId::GNSS2)
    {
        speed_meas_id = MeasurementId::GNSS2_SPEED_MAG;
        vel_meas_id = MeasurementId::GNSS2_VEL_NED;
        pos_meas_id = MeasurementId::GNSS2_POS_NED;
        gnss_init_done = gnss2_stationary_init_done_;
        gnss_pos_origin_lla = gnss2_origin_lla_;
        gnss_offset_x_vehicle_frd = gnss2_offset_x_vehicle_frd_;
        gnss_offset_y_vehicle_frd = gnss2_offset_y_vehicle_frd_;
    }
    else
    {
        spdlog::warn("UNKNOWN ANTENNA ID");
        return false;
    }

    update_debug_statuses(speed_meas_id, default_false_result);
    update_debug_statuses(vel_meas_id, default_false_result);
    update_debug_statuses(pos_meas_id, default_false_result);

    if (!gnss_init_done)
    {
        handle_gnss_stationary_init(sample, antenna_id);
        update_debug_statuses(speed_meas_id, speed_update_result);
        update_debug_statuses(vel_meas_id, vel_update_result);
        update_debug_statuses(pos_meas_id, pos_update_result);
        return false;
    }

    if (!imu_stationary_init_done_)
    {
        update_debug_statuses(speed_meas_id, speed_update_result);
        update_debug_statuses(vel_meas_id, vel_update_result);
        update_debug_statuses(pos_meas_id, pos_update_result);
        return false;
    }

    const double v_N = sample.vn_m_s;
    const double v_E = sample.ve_m_s;
    const double gnss_speed_meas = std::hypot(v_N, v_E);

    if (antenna_id == AntennaId::GNSS1)
    {
        last_gnss1_speed_ = gnss_speed_meas;
        last_ekf_speed_gnss1_ = current_ekf_speed();

        last_gnss1_speed_valid_ = sample.vel_valid;

        last_gnss1_vn_ = sample.vn_m_s;
        last_gnss1_ve_ = sample.ve_m_s;
        last_gnss1_vd_ = sample.vd_m_s;
    }
    else
    {
        last_gnss2_speed_ = gnss_speed_meas;
        last_ekf_speed_gnss2_ = current_ekf_speed();

        last_gnss2_speed_valid_ = sample.vel_valid;

        last_gnss2_vn_ = sample.vn_m_s;
        last_gnss2_ve_ = sample.ve_m_s;
        last_gnss2_vd_ = sample.vd_m_s;
    }

    if (sample.vel_valid)
    {
        // -----SPEED update-----
        speed_update_result = ekf_.update_gnss_speed_magnitude(
            gnss_speed_meas, gnss_speed_sigma,
            gnss_speed_magnitude_confidence_level_);

        update_ekf_output();

        if (!result.has_state)
        {
            result.has_state = true;
        }

        result.state = latest_output_;

        update_debug_statuses(speed_meas_id, speed_update_result);

        // -----SPEED update-----

        if (!alpha_course_aligned_)
        {
            const StateVec& x_before = ekf_.state();

            const double vx_body_before = x_before(StateIndex::VX);
            const double vy_body_before = x_before(StateIndex::VY);
            const double yaw_before = x_before(StateIndex::YAW);
            const double ekf_speed_before = current_ekf_speed();

            // -----ALPHA course over ground alignment-----
            if (gnss_speed_meas >= alpha_alignment_min_speed_ &&
                ekf_speed_before >= alpha_alignment_min_speed_)
            {
                const double course_ned = std::atan2(v_E, v_N);
                const double course_body =
                    std::atan2(vy_body_before, vx_body_before);

                const double alpha_meas =
                    wrap_angle(course_ned - yaw_before - course_body);

                StateVec x_new = x_before;
                x_new(StateIndex::ALPHA) = alpha_meas;

                StateMat P_new = ekf_.covariance();
                P_new.row(StateIndex::ALPHA).setZero();
                P_new.col(StateIndex::ALPHA).setZero();
                P_new(StateIndex::ALPHA, StateIndex::ALPHA) = 0.2 * 0.2;

                ekf_.reset(x_new, P_new);

                alpha_course_aligned_ = true;

                // update initial heading for gnss
                gnss_initial_heading_ned_ = alpha_meas;
            }
            // -----ALPHA course over ground alignment-----
        }

        else
        {
            // -----VELOCITY update-----
            bool gnss_vel_enabled =
                gnss_speed_meas >= gnss_velocity_ned_min_speed_;

            bool gnss_vel_accepted = false;

            if (gnss_vel_enabled)
            {
                vel_update_result = ekf_.update_gnss_velocity_ned(
                    v_N, v_E, corrected_yaw_rate_vehicle_frd_,
                    gnss_offset_x_vehicle_frd, gnss_offset_y_vehicle_frd,
                    gnss_vel_sigma, gnss_velocity_ned_confidence_level_);
                gnss_vel_accepted = vel_update_result.accepted;

                update_ekf_output();

                if (!result.has_state)
                {
                    result.has_state = true;
                }

                result.state = latest_output_;
            }
            update_debug_statuses(vel_meas_id, vel_update_result);

            // -----VELOCITY update-----
        }
    }

    if (sample.pos_valid)
    {
        bool gnss_pos_accepted = false;
        const double point_lat_deg = sample.lat_deg;
        const double point_lon_deg = sample.lon_deg;
        const double point_alt_m = sample.alt_m;

        Eigen::Vector3d gps_point_lla;
        gps_point_lla << point_lat_deg, point_lon_deg, point_alt_m;

        const Eigen::Vector2d gps_point_ned_2d =
            lla_to_ned_2d(gps_point_lla, gnss_pos_origin_lla);

        // -----POSITION update-----
        if (alpha_course_aligned_)
        {
            pos_update_result = ekf_.update_gnss_position_ned(
                gps_point_ned_2d, gnss_offset_x_vehicle_frd,
                gnss_offset_y_vehicle_frd, gnss_initial_heading_ned_,
                gnss_pos_sigma, gnss_pos_ned_confidence_level_);

            const bool gnss_pos_enabled = pos_update_result.attempted;
            gnss_pos_accepted = pos_update_result.accepted;

            if (antenna_id == AntennaId::GNSS1)
            {
                last_gnss1_pn_ = gps_point_ned_2d(0);
                last_gnss1_pe_ = gps_point_ned_2d(1);
                last_gnss1_pos_valid_ = true;
            }
            else
            {
                last_gnss2_pn_ = gps_point_ned_2d(0);
                last_gnss2_pe_ = gps_point_ned_2d(1);
                last_gnss2_pos_valid_ = true;
            }

            update_ekf_output();

            if (!result.has_state)
            {
                result.has_state = true;
            }

            result.state = latest_output_;
        }
        // -----POSITION update-----
        else
        {
            if (antenna_id == AntennaId::GNSS1)
            {
                last_gnss1_pn_ = gps_point_ned_2d(0);
                last_gnss1_pe_ = gps_point_ned_2d(1);
                last_gnss1_pos_valid_ = true;
            }
            else
            {
                last_gnss2_pn_ = gps_point_ned_2d(0);
                last_gnss2_pe_ = gps_point_ned_2d(1);
                last_gnss2_pos_valid_ = true;
            }
        }
    }
    else
    {
        if (antenna_id == AntennaId::GNSS1)
        {
            last_gnss1_pos_valid_ = false;
        }
        else
        {
            last_gnss2_pos_valid_ = false;
        }
    }

    update_debug_statuses(pos_meas_id, pos_update_result);
    return true;
}

double EkfManager::current_ekf_speed() const
{
    const StateVec& x = ekf_.state();

    const double vx = x(StateIndex::VX);
    const double vy = x(StateIndex::VY);

    return std::sqrt(vx * vx + vy * vy);
}

EkfStepResult EkfManager::handle_dual_gnss(DualGnssSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    EkfStepResult result;

    if (!last_dual_gnss_time_startup_ns_)
    {
        UpdateResult default_false_result;
        last_dual_gnss_time_startup_ns_ = sample.time_startup_ns;
        update_debug_statuses(MeasurementId::GNSS1_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_POS_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_POS_NED,
                              default_false_result);
        return result;
    }

    last_dual_gnss_time_startup_ns_ = sample.time_startup_ns;

    const double gnss1_speed_sigma =
        choose_sigma(gnss1_speed_update_sigma_floor_, sample.gnss1.velu_m_s);
    const double gnss1_vel_sigma =
        choose_sigma(gnss1_vel_update_sigma_floor_, sample.gnss1.velu_m_s);
    const double gnss1_pos_sigma =
        choose_sigma(gnss1_pos_update_sigma_floor_,
                     std::hypot(sample.gnss1.posu_n_m, sample.gnss1.posu_e_m));

    const double gnss2_speed_sigma =
        choose_sigma(gnss2_speed_update_sigma_floor_, sample.gnss2.velu_m_s);
    const double gnss2_vel_sigma =
        choose_sigma(gnss2_vel_update_sigma_floor_, sample.gnss2.velu_m_s);
    const double gnss2_pos_sigma =
        choose_sigma(gnss2_pos_update_sigma_floor_,
                     std::hypot(sample.gnss2.posu_n_m, sample.gnss2.posu_e_m));

    handle_single_gnss(result, sample.gnss1, gnss1_speed_sigma, gnss1_vel_sigma,
                       gnss1_pos_sigma, AntennaId::GNSS1);
    handle_single_gnss(result, sample.gnss2, gnss2_speed_sigma, gnss2_vel_sigma,
                       gnss2_pos_sigma, AntennaId::GNSS2);

    gnss1_speed_sigma_used_ = gnss1_speed_sigma;
    gnss1_vel_sigma_used_ = gnss1_vel_sigma;
    gnss1_pos_sigma_used_ = gnss1_pos_sigma;

    gnss2_speed_sigma_used_ = gnss2_speed_sigma;
    gnss2_vel_sigma_used_ = gnss2_vel_sigma;
    gnss2_pos_sigma_used_ = gnss2_pos_sigma;

    if (should_publish_debug_snapshot(sample.time_startup_ns))
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
    // TODO: INS logic
    // intentionally left blank, might use later

    (void)sample;

    if (should_publish_debug_snapshot(sample.time_startup_ns))
    {
        update_debug_snapshot();
        result.has_debug_snapshot = true;
        result.debug_snapshot = latest_debug_snapshot_;
    }

    return result;
}

bool EkfManager::is_stationary()
{
    const bool gyro_is_stationary = std::abs(corrected_yaw_rate_vehicle_frd_) <
                                    stationary_gyro_z_threshold_;
    const bool accel_x_is_stationary =
        std::abs(corrected_accel_x_vehicle_frd_) <
        stationary_accel_xy_threshold_;
    const bool accel_y_is_stationary =
        std::abs(corrected_accel_y_vehicle_frd_) <
        stationary_accel_xy_threshold_;

    const bool gnss1_is_stationary =
        (!last_gnss1_speed_valid_) || std::abs(last_gnss1_speed_) < 0.25;
    const bool gnss2_is_stationary =
        (!last_gnss2_speed_valid_) || std::abs(last_gnss2_speed_) < 0.25;

    return gyro_is_stationary && accel_x_is_stationary &&
           accel_y_is_stationary && gnss1_is_stationary && gnss2_is_stationary;
}

double EkfManager::compute_zero_lat_sigma(const ImuSample& imu)
{
    const double speed =
        last_gnss2_speed_valid_ ? last_gnss2_speed_ : current_ekf_speed();

    if (speed < zero_lat_min_speed_)
    {
        return zero_lat_disabled_sigma_;
    }

    double sigma = zero_lat_base_sigma_;

    sigma *= 1.0 + zero_lat_yaw_rate_gain_ *
                       std::abs(corrected_yaw_rate_vehicle_frd_);
    sigma *= 1.0 + zero_lat_lateral_accel_gain_ *
                       std::abs(corrected_accel_y_vehicle_frd_);

    return std::clamp(sigma, zero_lat_base_sigma_, zero_lat_disabled_sigma_);
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

void EkfManager::handle_gnss_stationary_init(GnssSample& sample, int antenna_id)
{
    if (!sample.pos_valid)
    {
        return;
    }

    if (antenna_id == AntennaId::GNSS1)
    {
        if (gnss1_stationary_init_done_)
        {
            return;
        }

        gnss1_origin_lla_(0) += sample.lat_deg;
        gnss1_origin_lla_(1) += sample.lon_deg;
        gnss1_origin_lla_(2) += sample.alt_m;

        gnss1_stationary_sample_count_ += 1;

        if (gnss1_stationary_sample_count_ < gnss1_stationary_required_samples_)
        {
            if (gnss1_stationary_sample_count_ % 5 == 0)
            {
                spdlog::info(
                    "**gnss1 stationary init progress: {}",
                    static_cast<double>(gnss1_stationary_sample_count_) /
                        gnss1_stationary_required_samples_ * 100.0);
            }
            return;
        }
    }
    else if (antenna_id == AntennaId::GNSS2)
    {
        if (gnss2_stationary_init_done_)
        {
            return;
        }

        gnss2_origin_lla_(0) += sample.lat_deg;
        gnss2_origin_lla_(1) += sample.lon_deg;
        gnss2_origin_lla_(2) += sample.alt_m;

        gnss2_stationary_sample_count_ += 1;

        if (gnss2_stationary_sample_count_ < gnss2_stationary_required_samples_)
        {
            if (gnss2_stationary_sample_count_ % 5 == 0)
            {
                spdlog::info(
                    "^^gnss2 stationary init progress: {}",
                    static_cast<double>(gnss2_stationary_sample_count_) /
                        gnss2_stationary_required_samples_ * 100.0);
            }
            return;
        }
    }
    else
    {
        spdlog::warn("UNKNOWN ANTENNA ID");
        return;
    }

    // collected enough data points, find origin
    if (antenna_id == AntennaId::GNSS1)
    {
        const double count =
            static_cast<double>(gnss1_stationary_sample_count_);

        gnss1_origin_lla_(0) = gnss1_origin_lla_(0) / count;
        gnss1_origin_lla_(1) = gnss1_origin_lla_(1) / count;
        gnss1_origin_lla_(2) = gnss1_origin_lla_(2) / count;

        gnss1_stationary_init_done_ = true;

        spdlog::info("GNSS1 stationary initialization COMPLETE");

        return;
    }
    else
    {
        const double count =
            static_cast<double>(gnss2_stationary_sample_count_);

        gnss2_origin_lla_(0) = gnss2_origin_lla_(0) / count;
        gnss2_origin_lla_(1) = gnss2_origin_lla_(1) / count;
        gnss2_origin_lla_(2) = gnss2_origin_lla_(2) / count;

        gnss2_stationary_init_done_ = true;

        spdlog::info("GNSS2 stationary initialization COMPLETE");

        return;
    }
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

void EkfManager::determine_gnss_validities(GnssSample& sample, int antenna_id)
{
    sample.vel_valid = false;
    sample.pos_valid = false;

    if (antenna_id == AntennaId::GNSS1)
    {
        last_gnss1_pos_valid_ = false;
        last_gnss1_speed_valid_ = false;
        last_gnss1_vel_valid_ = false;
    }
    else if (antenna_id == AntennaId::GNSS2)
    {
        last_gnss2_pos_valid_ = false;
        last_gnss2_speed_valid_ = false;
        last_gnss2_vel_valid_ = false;
    }

    const double gnss_speed = std::hypot(sample.vn_m_s, sample.ve_m_s);

    if (std::isfinite(sample.vn_m_s) && std::isfinite(sample.ve_m_s) &&
        std::isfinite(sample.velu_m_s))
    {
        if (sample.fix >= 2 && gnss_speed <= 50.0 && sample.velu_m_s <= 2.0)
        {
            sample.vel_valid = true;

            if (antenna_id == AntennaId::GNSS1)
            {
                last_gnss1_speed_valid_ = true;
                last_gnss1_vel_valid_ = true;
            }
            else if (antenna_id == AntennaId::GNSS2)
            {
                last_gnss2_speed_valid_ = true;
                last_gnss2_vel_valid_ = true;
            }
        }
    }

    if (std::isfinite(sample.lat_deg) && std::isfinite(sample.lon_deg) &&
        std::isfinite(sample.posu_n_m) && std::isfinite(sample.posu_e_m))
    {
        const double posu_2d_ned = std::hypot(sample.posu_n_m, sample.posu_e_m);

        if (sample.fix >= 2 && posu_2d_ned <= 10.0)
        {
            sample.pos_valid = true;

            if (antenna_id == AntennaId::GNSS1)
            {
                last_gnss1_pos_valid_ = true;
            }
            else if (antenna_id == AntennaId::GNSS2)
            {
                last_gnss2_pos_valid_ = true;
            }
        }
    }
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

            zero_lat_vel_residual_ = 0.0;

            if (update_result.residual.size() >= 1)
            {
                zero_lat_vel_residual_ = update_result.residual(0);
            }

            break;

        case MeasurementId::GNSS1_SPEED_MAG:

            gnss1_speed_update_enabled_ = update_result.attempted;
            gnss1_speed_update_accepted_ = update_result.accepted;
            gnss1_speed_nis_ = update_result.nis;
            gnss1_speed_gate_ = update_result.threshold;

            gnss1_speed_residual_ = 0.0;

            if (update_result.residual.size() >= 1)
            {
                gnss1_speed_residual_ = update_result.residual(0);
            }

            break;

        case MeasurementId::GNSS1_VEL_NED:

            gnss1_vel_update_enabled_ = update_result.attempted;
            gnss1_vel_update_accepted_ = update_result.accepted;
            gnss1_vel_nis_ = update_result.nis;
            gnss1_vel_gate_ = update_result.threshold;

            gnss1_vel_ned_n_residual_ = 0.0;
            gnss1_vel_ned_e_residual_ = 0.0;

            if (update_result.residual.size() >= 2)
            {
                gnss1_vel_ned_n_residual_ = update_result.residual(0);
                gnss1_vel_ned_e_residual_ = update_result.residual(1);
            }

            break;

        case MeasurementId::GNSS1_POS_NED:

            gnss1_pos_update_enabled_ = update_result.attempted;
            gnss1_pos_update_accepted_ = update_result.accepted;
            gnss1_pos_nis_ = update_result.nis;
            gnss1_pos_gate_ = update_result.threshold;

            gnss1_pos_ned_n_residual_ = 0.0;
            gnss1_pos_ned_e_residual_ = 0.0;

            if (update_result.residual.size() >= 2)
            {
                gnss1_pos_ned_n_residual_ = update_result.residual(0);
                gnss1_pos_ned_e_residual_ = update_result.residual(1);
            }

            break;

        case MeasurementId::GNSS2_SPEED_MAG:

            gnss2_speed_update_enabled_ = update_result.attempted;
            gnss2_speed_update_accepted_ = update_result.accepted;
            gnss2_speed_nis_ = update_result.nis;
            gnss2_speed_gate_ = update_result.threshold;

            gnss2_speed_residual_ = 0.0;

            if (update_result.residual.size() >= 1)
            {
                gnss2_speed_residual_ = update_result.residual(0);
            }

            break;

        case MeasurementId::GNSS2_VEL_NED:

            gnss2_vel_update_enabled_ = update_result.attempted;
            gnss2_vel_update_accepted_ = update_result.accepted;
            gnss2_vel_nis_ = update_result.nis;
            gnss2_vel_gate_ = update_result.threshold;

            gnss2_vel_ned_n_residual_ = 0.0;
            gnss2_vel_ned_e_residual_ = 0.0;

            if (update_result.residual.size() >= 2)
            {
                gnss2_vel_ned_n_residual_ = update_result.residual(0);
                gnss2_vel_ned_e_residual_ = update_result.residual(1);
            }

            break;

        case MeasurementId::GNSS2_POS_NED:

            gnss2_pos_update_enabled_ = update_result.attempted;
            gnss2_pos_update_accepted_ = update_result.accepted;
            gnss2_pos_nis_ = update_result.nis;
            gnss2_pos_gate_ = update_result.threshold;

            gnss2_pos_ned_n_residual_ = 0.0;
            gnss2_pos_ned_e_residual_ = 0.0;

            if (update_result.residual.size() >= 2)
            {
                gnss2_pos_ned_n_residual_ = update_result.residual(0);
                gnss2_pos_ned_e_residual_ = update_result.residual(1);
            }

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

    latest_debug_snapshot_.latest_zero_lat_vel_residual =
        zero_lat_vel_residual_;

    latest_debug_snapshot_.latest_gnss1_speed_residual = gnss1_speed_residual_;
    latest_debug_snapshot_.latest_gnss1_vel_ned_n_residual =
        gnss1_vel_ned_n_residual_;
    latest_debug_snapshot_.latest_gnss1_vel_ned_e_residual =
        gnss1_vel_ned_e_residual_;
    latest_debug_snapshot_.latest_gnss1_pos_ned_n_residual =
        gnss1_pos_ned_n_residual_;
    latest_debug_snapshot_.latest_gnss1_pos_ned_e_residual =
        gnss1_pos_ned_e_residual_;

    latest_debug_snapshot_.latest_gnss2_speed_residual = gnss2_speed_residual_;
    latest_debug_snapshot_.latest_gnss2_vel_ned_n_residual =
        gnss2_vel_ned_n_residual_;
    latest_debug_snapshot_.latest_gnss2_vel_ned_e_residual =
        gnss2_vel_ned_e_residual_;
    latest_debug_snapshot_.latest_gnss2_pos_ned_n_residual =
        gnss2_pos_ned_n_residual_;
    latest_debug_snapshot_.latest_gnss2_pos_ned_e_residual =
        gnss2_pos_ned_e_residual_;

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

    latest_debug_snapshot_.latest_gnss1_vel_valid = last_gnss1_vel_valid_;
    latest_debug_snapshot_.latest_gnss1_pos_valid = last_gnss1_pos_valid_;
    latest_debug_snapshot_.latest_gnss2_vel_valid = last_gnss2_vel_valid_;
    latest_debug_snapshot_.latest_gnss2_pos_valid = last_gnss2_pos_valid_;
}

double EkfManager::choose_sigma(double floor_sigma, double reported_sigma)
{
    if (!use_gnss_reported_uncertainty_)
    {
        return floor_sigma;
    }

    if (!std::isfinite(reported_sigma) || reported_sigma <= 0.0)
    {
        return floor_sigma;
    }

    return std::max(floor_sigma, reported_sigma);
}

void EkfManager::check_gnss_stale_status()
{
    UpdateResult default_false_result;
    if (!last_dual_gnss_time_startup_ns_)
    {
        update_debug_statuses(MeasurementId::GNSS1_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_POS_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_POS_NED,
                              default_false_result);

        last_gnss1_speed_valid_ = false;
        last_gnss1_vel_valid_ = false;
        last_gnss1_pos_valid_ = false;
        last_gnss2_speed_valid_ = false;
        last_gnss2_vel_valid_ = false;
        last_gnss2_pos_valid_ = false;

        last_gnss1_speed_ = 0.0;
        last_gnss2_speed_ = 0.0;

        return;
    }

    double age = static_cast<double>(last_imu_time_startup_ns_.value() -
                                     last_dual_gnss_time_startup_ns_.value()) *
                 1e-9;

    if (age > gnss_stale_timeout_s_)
    {
        update_debug_statuses(MeasurementId::GNSS1_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS1_POS_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_SPEED_MAG,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_VEL_NED,
                              default_false_result);
        update_debug_statuses(MeasurementId::GNSS2_POS_NED,
                              default_false_result);

        last_gnss1_speed_valid_ = false;
        last_gnss1_vel_valid_ = false;
        last_gnss1_pos_valid_ = false;
        last_gnss2_speed_valid_ = false;
        last_gnss2_vel_valid_ = false;
        last_gnss2_pos_valid_ = false;

        last_gnss1_speed_ = 0.0;
        last_gnss2_speed_ = 0.0;

        return;
    }
}

}  // namespace htx_ekf