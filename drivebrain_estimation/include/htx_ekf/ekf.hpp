#pragma once
#include <boost/math/distributions/chi_squared.hpp>
#include <cstdint>

#include "Eigen/Dense"
#include "htx_ekf/types.hpp"

namespace htx_ekf
{

struct ImuSample
{
    uint64_t time_startup_ns = 0;

    double dt_s = 0.0;

    // vehicle aligned FRD
    double ax_frd_m_s2 = 0.0;
    double ay_frd_m_s2 = 0.0;
    double gz_frd_rad_s = 0.0;
};

struct GnssSample
{
    uint64_t tow_ns = 0;
    uint32_t week = 0;
    uint32_t num_sats = 0;
    uint32_t fix = 0;

    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;

    double vn_m_s = 0.0;
    double ve_m_s = 0.0;
    double vd_m_s = 0.0;

    double posu_n_m = 0.0;
    double posu_e_m = 0.0;
    double posu_d_m = 0.0;

    double velu_m_s = 0.0;

    bool pos_valid = false;
    bool vel_valid = false;
};

struct DualGnssSample
{
    uint64_t time_startup_ns = 0;

    GnssSample gnss1;
    GnssSample gnss2;
};

struct InsSample
{
    uint64_t time_startup_ns = 0;
    uint32_t ins_status = 0;

    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;

    // Vehicle aligned FRD
    double vx_body_frd_m_s = 0.0;
    double vy_body_frd_m_s = 0.0;

    double vn_m_s = 0.0;
    double ve_m_s = 0.0;
    double vd_m_s = 0.0;

    double posu_m = 0.0;
    double velu_m_s = 0.0;
};

// struct ImuInput
// {
//     double accel_x_body = 0.0;
//     double accel_y_body = 0.0;
//     double gyro_z_body = 0.0;
// };

struct EkfProcConfig
{
    double q_std_pn = 0.5;
    double q_std_pe = 0.5;
    double q_std_yaw = 1e-4;
    double q_std_vx = 0.5;
    double q_std_vy = 0.5;
    double q_std_alpha = 1e-6;
    double q_std_bg = 1e-8;
    double q_std_bax = 1e-6;
    double q_std_bay = 1e-6;
};

struct UpdateResult
{
    bool attempted = false;
    bool accepted = false;
    double nis = 0.0;
    double threshold = 0.0;
    Eigen::VectorXd residual;
};

struct NisGateResult
{
    double nis = 0.0;
    double threshold = 0.0;
    bool passed = false;
};

class Ekf
{
   public:
    Ekf();

    void set_proc_config(const EkfProcConfig& config);

    void reset(const StateVec& x0, const StateMat& P0);

    // prediction based on IMU
    void predict_imu(const ImuSample& imu);

    // measurement update functions
    UpdateResult update_zero_vel(double sigma, double confidence_level);

    UpdateResult update_zero_lat_velocity(double sigma,
                                          double confidence_level);

    UpdateResult update_gnss_speed_magnitude(double speed_meas, double sigma,
                                             double confidence_level);

    UpdateResult update_gnss_velocity_ned(
        double v_n, double v_e, double imu_yaw_rate, double gnss_offset_x_frd,
        double gnss_offset_y_frd, double sigma, double confidence_level);

    UpdateResult update_gnss_position_ned(Eigen::Vector2d point_ned_2d,
                                          double gnss_offset_x_frd,
                                          double gnss_offset_y_frd,
                                          double gnss_initial_heading_ned,
                                          double sigma,
                                          double confidence_level);

    // may want to constrain vy?
    void constrain_vy();

    // state and covariance getters
    const StateVec& state() const;
    const StateMat& covariance() const;
    const StateMat& process() const;

   private:
    UpdateResult update_generic(const Eigen::VectorXd& residual,
                                const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& R,
                                double confidence_level);

    NisGateResult pass_nis_gate(const Eigen::VectorXd& residual,
                                const Eigen::MatrixXd& S,
                                double confidence_level);

    double get_chi_square_threshold(int meas_dim, double confidence_level);

    void wrap_state_angles();

    void enforce_covariance_safety();

    StateVec x_;
    StateMat P_;
    StateMat Q_;

    EkfProcConfig config_;

    bool initialized_ = false;

    const double max_vy_mag_ = 2.0;
};

}  // namespace htx_ekf