#pragma once

#include <sys/types.h>

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include "htx_ekf/ekf.hpp"
#include "htx_ekf/state.hpp"
#include "htx_ekf/types.hpp"

namespace htx_ekf
{

enum InitStatus
{
    Unspecified = 0,
    Waiting = 1,
    InProgress = 2,
    Done = 3,
    Failed = 4
};

struct AntennaId
{
    static constexpr int GNSS1 = 0;
    static constexpr int GNSS2 = 1;
};

struct MeasurementId
{
    static constexpr int ZERO_VEL = 0;
    static constexpr int ZERO_LAT_VEL = 1;
    static constexpr int GNSS1_SPEED_MAG = 2;
    static constexpr int GNSS2_SPEED_MAG = 3;
    static constexpr int GNSS1_VEL_NED = 4;
    static constexpr int GNSS2_VEL_NED = 5;
    static constexpr int GNSS1_POS_NED = 6;
    static constexpr int GNSS2_POS_NED = 7;
};

struct EkfManagerConfig
{
    // Initial covariance std devs
    double init_std_pn = 1.0;
    double init_std_pe = 1.0;
    double init_std_yaw = 0.10;
    double init_std_vx = 0.5;
    double init_std_vy = 0.5;
    double init_std_alpha = 3.14;
    double init_std_bg = 0.001;
    double init_std_bax = 0.05;
    double init_std_bay = 0.05;

    double alpha_alignment_min_speed = 1.0;

    // GNSS antenna offsets in vehicle FRD frame
    double gnss1_offset_x_vehicle_frd = 1.8;
    double gnss1_offset_y_vehicle_frd = 0.0;
    double gnss2_offset_x_vehicle_frd = -1.9;
    double gnss2_offset_y_vehicle_frd = 0.0;

    // EKF process noise
    EkfProcConfig proc;

    // Measurement sigmas
    double zero_vel_update_sigma = 0.05;

    double gnss1_speed_update_sigma_floor = 0.4;
    double gnss1_vel_update_sigma_floor = 0.5;
    double gnss1_pos_update_sigma_floor = 3.0;
    double gnss2_speed_update_sigma_floor = 0.4;
    double gnss2_vel_update_sigma_floor = 0.5;
    double gnss2_pos_update_sigma_floor = 3.0;

    bool use_gnss_reported_uncertainty = true;

    double zero_lat_base_sigma = 0.2;
    double zero_lat_disabled_sigma = 6.0;
    double zero_lat_yaw_rate_gain = 15.0;
    double zero_lat_lateral_accel_gain = 5.0;

    // NIS gate confidence levels
    double zero_vel_confidence_level = 0.999;
    double zero_lat_velocity_confidence_level = 0.99;
    double gnss_speed_magnitude_confidence_level = 0.99;
    double gnss_velocity_ned_confidence_level = 0.99;
    double gnss_pos_ned_confidence_level = 0.99;

    // Stationary init
    std::size_t imu_stationary_required_samples = 1200;
    std::size_t gnss1_stationary_required_samples = 25;
    std::size_t gnss2_stationary_required_samples = 25;

    // Stationary detection
    double stationary_gyro_z_threshold = 0.05;
    double stationary_accel_threshold = 0.50;
    double stationary_speed_threshold_m_s = 0.25;
    std::size_t stationary_required_samples = 25;

    // Measurement enable thresholds
    double zero_lat_min_speed = 1.0;
    double gnss_velocity_ned_min_speed = 1.0;

    // GNSS stale timeout
    double gnss_stale_timeout_s = 0.5;
};

struct EkfOutput
{
    uint64_t time_startup_ns = 0;

    // // sensor aligned FRD
    // double vx_sensor_frd_m_s = 0.0;
    // double vy_sensor_frd_m_s = 0.0;
    // double yaw_rate_corrected_sensor_frd_rad_s = 0.0;
    // double ax_corrected_sensor_frd_m_s2 = 0.0;
    // double ay_corrected_sensor_frd_m_s2 = 0.0;

    // vehicle aligned FRD
    double vx_vehicle_frd_m_s = 0.0;
    double vy_vehicle_frd_m_s = 0.0;
    double yaw_rate_corrected_vehicle_frd_rad_s = 0.0;
    double ax_corrected_vehicle_frd_m_s2 = 0.0;
    double ay_corrected_vehicle_frd_m_s2 = 0.0;

    // vehicle aligned FLU
    double vx_vehicle_flu_m_s = 0.0;
    double vy_vehicle_flu_m_s = 0.0;
    double yaw_rate_corrected_vehicle_flu_rad_s = 0.0;
    double ax_corrected_vehicle_flu_m_s2 = 0.0;
    double ay_corrected_vehicle_flu_m_s2 = 0.0;

    double position_n_m = 0.0;
    double position_e_m = 0.0;

    double yaw_vehicle_frd_rad = 0.0;
    double yaw_vehicle_flu_rad = 0.0;

    double alpha_startup_to_ned_rad = 0.0;

    double bg_rad_s = 0.0;
    double bax_m_s2 = 0.0;
    double bay_m_s2 = 0.0;

    double last_imu_dt_s = 0.0;
    bool initialized = false;
};

struct EkfDebugSnapshot
{
    uint64_t time_startup_ns = 0;

    InitStatus imu_status = InitStatus::Waiting;
    InitStatus gnss1_status = InitStatus::Waiting;
    InitStatus gnss2_status = InitStatus::Waiting;
    InitStatus alpha_status = InitStatus::Waiting;

    uint32_t imu_sample_count = 0;
    uint32_t imu_required_samples = 0;
    double imu_init_progress = 0.0;

    uint32_t gnss1_sample_count = 0;
    uint32_t gnss1_required_samples = 0;
    double gnss1_init_progress = 0.0;

    uint32_t gnss2_sample_count = 0;
    uint32_t gnss2_required_samples = 0;
    double gnss2_init_progress = 0.0;

    bool stationary_detected = false;

    bool latest_zero_vel_update_enabled = false;
    bool latest_zero_vel_update_accepted = false;
    double latest_zero_vel_nis = 0.0;
    double latest_zero_vel_gate = 0.0;

    bool latest_zero_lat_vel_update_enabled = false;
    bool latest_zero_lat_vel_update_accepted = false;
    double latest_zero_lat_vel_nis = 0.0;
    double latest_zero_lat_vel_gate = 0.0;
    double latest_zero_lat_vel_sigma_used = 0.0;
    double latest_zero_lat_vel_residual = 0.0;

    bool latest_gnss1_speed_update_enabled = false;
    bool latest_gnss1_speed_update_accepted = false;
    double latest_gnss1_speed_nis = 0.0;
    double latest_gnss1_speed_gate = 0.0;
    double latest_gnss1_speed_residual = 0.0;

    bool latest_gnss1_vel_update_enabled = false;
    bool latest_gnss1_vel_update_accepted = false;
    double latest_gnss1_vel_nis = 0.0;
    double latest_gnss1_vel_gate = 0.0;
    double latest_gnss1_vel_ned_n_residual = 0.0;
    double latest_gnss1_vel_ned_e_residual = 0.0;

    bool latest_gnss1_pos_update_enabled = false;
    bool latest_gnss1_pos_update_accepted = false;
    double latest_gnss1_pos_nis = 0.0;
    double latest_gnss1_pos_gate = 0.0;
    double latest_gnss1_pos_ned_n_residual = 0.0;
    double latest_gnss1_pos_ned_e_residual = 0.0;

    bool latest_gnss2_speed_update_enabled = false;
    bool latest_gnss2_speed_update_accepted = false;
    double latest_gnss2_speed_nis = 0.0;
    double latest_gnss2_speed_gate = 0.0;
    double latest_gnss2_speed_residual = 0.0;

    bool latest_gnss2_vel_update_enabled = false;
    bool latest_gnss2_vel_update_accepted = false;
    double latest_gnss2_vel_nis = 0.0;
    double latest_gnss2_vel_gate = 0.0;
    double latest_gnss2_vel_ned_n_residual = 0.0;
    double latest_gnss2_vel_ned_e_residual = 0.0;

    bool latest_gnss2_pos_update_enabled = false;
    bool latest_gnss2_pos_update_accepted = false;
    double latest_gnss2_pos_nis = 0.0;
    double latest_gnss2_pos_gate = 0.0;
    double latest_gnss2_pos_ned_n_residual = 0.0;
    double latest_gnss2_pos_ned_e_residual = 0.0;

    double latest_gnss1_speed_sigma_used = 0.0;
    double latest_gnss1_vel_sigma_used = 0.0;
    double latest_gnss1_pos_sigma_used = 0.0;

    double latest_gnss2_speed_sigma_used = 0.0;
    double latest_gnss2_vel_sigma_used = 0.0;
    double latest_gnss2_pos_sigma_used = 0.0;

    bool latest_gnss1_vel_valid = false;
    bool latest_gnss1_pos_valid = false;

    bool latest_gnss2_vel_valid = false;
    bool latest_gnss2_pos_valid = false;
};

struct EkfStepResult
{
    bool has_state = false;
    bool has_debug_snapshot = false;

    EkfOutput state;
    EkfDebugSnapshot debug_snapshot;
};

class EkfManager
{
   public:
    explicit EkfManager(const EkfManagerConfig& config = EkfManagerConfig{});

    EkfStepResult handle_imu(const ImuSample& sample);
    EkfStepResult handle_dual_gnss(DualGnssSample& sample);
    EkfStepResult handle_ins(const InsSample& sample);

    EkfOutput get_latest_output() const;

    void hard_reset();

   private:
    void load_config(const EkfManagerConfig& config);
    void load_ekf_proc_config(const EkfProcConfig& config);

    // EKF initialization
    StateVec make_initial_state();
    StateMat make_initial_covariance();
    void initialize_ekf();

    void handle_imu_stationary_init(const ImuSample& imu);
    void handle_gnss_stationary_init(GnssSample& sample, int antenna_id);

    bool is_stationary(const ImuSample& imu);

    bool handle_single_gnss(EkfStepResult& result, GnssSample& sample,
                            double gnss_speed_sigma, double gnss_vel_sigma,
                            double gnss_pos_sigma, int antenna_id);

    double compute_zero_lat_sigma(const ImuSample& imu);

    double choose_sigma(double floor_sigma, double reported_sigma);

    void check_gnss_stale_status();

    void update_ekf_output();
    void update_debug_snapshot();

    bool should_publish_debug_snapshot(uint64_t vn_time_startup_ns);

    InitStatus compute_imu_status() const;
    InitStatus compute_gnss1_status() const;
    InitStatus compute_gnss2_status() const;
    InitStatus compute_alpha_status() const;

    void determine_gnss_validities(GnssSample& sample, int antenna_id);

    double progress_ratio(std::size_t count, std::size_t required);

    void update_debug_statuses(int measurement_id,
                               const UpdateResult& update_result);

    double current_ekf_speed() const;

    mutable std::mutex mutex_;

    Ekf ekf_;

    std::optional<uint64_t> last_imu_time_startup_ns_;
    double last_imu_dt_s_ = 0.0;

    // stationary init member variables
    std::size_t imu_stationary_required_samples_ = 1200;
    std::size_t gnss1_stationary_required_samples_ = 25;
    std::size_t gnss2_stationary_required_samples_ = 25;

    bool imu_stationary_init_done_ = false;
    bool gnss1_stationary_init_done_ = false;
    bool gnss2_stationary_init_done_ = false;

    std::size_t imu_stationary_sample_count_ = 0;
    std::size_t gnss1_stationary_sample_count_ = 0;
    std::size_t gnss2_stationary_sample_count_ = 0;

    double gz_sum_ = 0.0;
    double ax_sum_ = 0.0;
    double ay_sum_ = 0.0;

    // stationary gnss member variables
    Eigen::Vector3d gnss1_origin_lla_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gnss2_origin_lla_ = Eigen::Vector3d::Zero();

    // corrected imu values
    double corrected_yaw_rate_vehicle_frd_ = 0.0;
    double corrected_accel_x_vehicle_frd_ = 0.0;
    double corrected_accel_y_vehicle_frd_ = 0.0;

    // initial covariance member variables
    double init_std_pn_ = 0.0;
    double init_std_pe_ = 0.0;
    double init_std_yaw_ = 0.0;
    double init_std_vx_ = 0.0;
    double init_std_vy_ = 0.0;
    double init_std_alpha_ = 0.0;
    double init_std_bg_ = 0.0;
    double init_std_bax_ = 0.0;
    double init_std_bay_ = 0.0;

    // measurement sigma member variables
    double zero_vel_update_sigma_ = 0.05;  // m/s

    double gnss1_speed_update_sigma_floor_ = 0.2;
    double gnss1_vel_update_sigma_floor_ = 0.2;
    double gnss1_pos_update_sigma_floor_ = 1.5;
    double gnss2_speed_update_sigma_floor_ = 0.2;
    double gnss2_vel_update_sigma_floor_ = 0.2;
    double gnss2_pos_update_sigma_floor_ = 1.5;

    bool zero_lat_update_enabled_ = false;

    double zero_lat_base_sigma_ = 0.20;
    double zero_lat_disabled_sigma_ = 6.0;
    double zero_lat_yaw_rate_gain_ = 15.0;
    double zero_lat_lateral_accel_gain_ = 5.0;
    double zero_lat_current_sigma_ = 6.0;

    // confidence levels
    double zero_vel_confidence_level_ = 0.99;
    double zero_lat_velocity_confidence_level_ = 0.99;
    double gnss_speed_magnitude_confidence_level_ = 0.95;
    double gnss_velocity_ned_confidence_level_ = 0.95;
    double gnss_pos_ned_confidence_level_ = 0.95;

    // GNSS antenna offsets
    double gnss1_offset_x_vehicle_frd_ = 0.0;
    double gnss1_offset_y_vehicle_frd_ = 0.0;
    double gnss2_offset_x_vehicle_frd_ = 0.0;
    double gnss2_offset_y_vehicle_frd_ = 0.0;

    // GNSS course-over-ground alignment
    bool alpha_course_aligned_ = false;
    double alpha_alignment_min_speed_ = 1.0;
    double gnss_initial_heading_ned_ = 0.0;

    // Stationary detection
    double stationary_gyro_z_threshold_ = 0.05;
    double stationary_accel_threshold_ = 0.50;
    double stationary_speed_threshold_m_s_ = 0.25;

    std::size_t stationary_count_ = 0;
    std::size_t stationary_required_samples_ = 25;
    bool stationary_detected_ = false;

    // GNSS uncertainty behavior
    bool use_gnss_reported_uncertainty_ = true;

    // Zero lateral velocity threshold
    double zero_lat_min_speed_ = 1.0;

    // GNSS velocity threshold
    double gnss_velocity_ned_min_speed_ = 1.0;

    // update enabled/attempted status
    bool zero_vel_update_enabled_ = false;
    bool zero_lat_vel_update_enabled_ = false;
    bool gnss1_speed_update_enabled_ = false;
    bool gnss1_vel_update_enabled_ = false;
    bool gnss1_pos_update_enabled_ = false;
    bool gnss2_speed_update_enabled_ = false;
    bool gnss2_vel_update_enabled_ = false;
    bool gnss2_pos_update_enabled_ = false;

    // update accepted status
    bool zero_vel_update_accepted_ = false;
    bool zero_lat_vel_update_accepted_ = false;
    bool gnss1_speed_update_accepted_ = false;
    bool gnss1_vel_update_accepted_ = false;
    bool gnss1_pos_update_accepted_ = false;
    bool gnss2_speed_update_accepted_ = false;
    bool gnss2_vel_update_accepted_ = false;
    bool gnss2_pos_update_accepted_ = false;

    // gnss1 debug values
    double last_gnss1_speed_ = 0.0;
    double last_ekf_speed_gnss1_ = 0.0;
    bool last_gnss1_speed_valid_ = false;

    double last_gnss1_vn_ = 0.0;
    double last_gnss1_ve_ = 0.0;
    double last_gnss1_vd_ = 0.0;
    bool last_gnss1_vel_valid_ = false;

    double last_gnss1_pn_ = 0.0;
    double last_gnss1_pe_ = 0.0;
    bool last_gnss1_pos_valid_ = false;

    // gnss2 debug values
    double last_gnss2_speed_ = 0.0;
    double last_ekf_speed_gnss2_ = 0.0;
    bool last_gnss2_speed_valid_ = false;

    double last_gnss2_vn_ = 0.0;
    double last_gnss2_ve_ = 0.0;
    double last_gnss2_vd_ = 0.0;
    bool last_gnss2_vel_valid_ = false;

    double last_gnss2_pn_ = 0.0;
    double last_gnss2_pe_ = 0.0;
    bool last_gnss2_pos_valid_ = false;

    // NIS/gate
    double zero_vel_nis_ = 0.0;
    double zero_vel_gate_ = 0.0;

    double zero_lat_vel_nis_ = 0.0;
    double zero_lat_vel_gate_ = 0.0;

    double gnss1_speed_nis_ = 0.0;
    double gnss1_speed_gate_ = 0.0;
    double gnss1_vel_nis_ = 0.0;
    double gnss1_vel_gate_ = 0.0;
    double gnss1_pos_nis_ = 0.0;
    double gnss1_pos_gate_ = 0.0;

    double gnss2_speed_nis_ = 0.0;
    double gnss2_speed_gate_ = 0.0;
    double gnss2_vel_nis_ = 0.0;
    double gnss2_vel_gate_ = 0.0;
    double gnss2_pos_nis_ = 0.0;
    double gnss2_pos_gate_ = 0.0;

    // residuals
    double zero_lat_vel_residual_ = 0.0;

    double gnss1_speed_residual_ = 0.0;
    double gnss1_vel_ned_n_residual_ = 0.0;
    double gnss1_vel_ned_e_residual_ = 0.0;
    double gnss1_pos_ned_n_residual_ = 0.0;
    double gnss1_pos_ned_e_residual_ = 0.0;

    double gnss2_speed_residual_ = 0.0;
    double gnss2_vel_ned_n_residual_ = 0.0;
    double gnss2_vel_ned_e_residual_ = 0.0;
    double gnss2_pos_ned_n_residual_ = 0.0;
    double gnss2_pos_ned_e_residual_ = 0.0;

    // dynamic sigma
    double zero_lat_vel_sigma_used_ = 0.0;

    double gnss1_speed_sigma_used_ = 0.0;
    double gnss1_vel_sigma_used_ = 0.0;
    double gnss1_pos_sigma_used_ = 0.0;

    double gnss2_speed_sigma_used_ = 0.0;
    double gnss2_vel_sigma_used_ = 0.0;
    double gnss2_pos_sigma_used_ = 0.0;

    // GNSS stale timeout
    double gnss_stale_timeout_s_ = 0.5;
    std::optional<uint64_t> last_dual_gnss_time_startup_ns_;

    EkfOutput latest_output_;
    EkfDebugSnapshot latest_debug_snapshot_;

    std::optional<uint64_t> last_debug_snapshot_publish_time_ns_;
    uint64_t debug_snapshot_period_ns_ = 100000000ULL;  // 10 Hzj
};
}  // namespace htx_ekf
