
#include "htx_ekf/ekf.hpp"

#include <algorithm>
#include <cmath>

#include "htx_ekf/math_utils.hpp"
#include "htx_ekf/state.hpp"

namespace htx_ekf
{

Ekf::Ekf()
{
    x_.setZero();

    P_.setIdentity();
    P_ *= 10.0;

    set_proc_config(EkfProcConfig{});

    // Q_.setZero();
    // Q_(StateIndex::PN, StateIndex::PN) = 0.5;        // PN process noise
    // Q_(StateIndex::PE, StateIndex::PE) = 0.5;        // PE process noise
    // Q_(StateIndex::YAW, StateIndex::YAW) = 0.1;      // YAW process noise
    // Q_(StateIndex::VX, StateIndex::VX) = 0.25;       // VX process noise
    // Q_(StateIndex::VY, StateIndex::VY) = 0.25;       // VY process noise
    // Q_(StateIndex::ALPHA, StateIndex::ALPHA) = 0.5;  // ALPHA process noise
    // Q_(StateIndex::BG, StateIndex::BG) = 0.15;       // BG process noise
    // Q_(StateIndex::BAX, StateIndex::BAX) = 0.25;     // BAX process noise
    // Q_(StateIndex::BAY, StateIndex::BAY) = 0.25;     // BAY process noise

    initialized_ = false;
}

void Ekf::set_proc_config(const EkfProcConfig& config)
{
    config_ = config;

    Q_.setZero();

    Q_(StateIndex::PN, StateIndex::PN) = config.q_std_pn * config.q_std_pn;
    Q_(StateIndex::PE, StateIndex::PE) = config.q_std_pe * config.q_std_pe;
    Q_(StateIndex::YAW, StateIndex::YAW) = config.q_std_yaw * config.q_std_yaw;
    Q_(StateIndex::VX, StateIndex::VX) = config.q_std_vx * config.q_std_vx;
    Q_(StateIndex::VY, StateIndex::VY) = config.q_std_vy * config.q_std_vy;
    Q_(StateIndex::ALPHA, StateIndex::ALPHA) =
        config.q_std_alpha * config.q_std_alpha;
    Q_(StateIndex::BG, StateIndex::BG) = config.q_std_bg * config.q_std_bg;
    Q_(StateIndex::BAX, StateIndex::BAX) = config.q_std_bax * config.q_std_bax;
    Q_(StateIndex::BAY, StateIndex::BAY) = config.q_std_bay * config.q_std_bay;
}

void Ekf::reset(const StateVec& x0, const StateMat& P0)
{
    x_ = x0;
    P_ = P0;
    initialized_ = true;
    wrap_state_angles();
    enforce_covariance_safety();
}

const StateVec& Ekf::state() const { return x_; }

const StateMat& Ekf::covariance() const { return P_; }

const StateMat& Ekf::process() const { return Q_; }

void Ekf::wrap_state_angles()
{
    x_(StateIndex::YAW) = wrap_angle(x_(StateIndex::YAW));
    x_(StateIndex::ALPHA) = wrap_angle(x_(StateIndex::ALPHA));
}

void Ekf::enforce_covariance_safety()
{
    // enforce symmetry
    P_ = 0.5 * (P_ + P_.transpose());

    // diagonal entires should be >= 0
    P_.diagonal() = P_.diagonal().cwiseMax(1e-9);

    // restrict diagonal entries from growing infinitely
    P_.diagonal() = P_.diagonal().cwiseMin(1e6);
}

void Ekf::predict_imu(const ImuSample& raw_imu_sample)
{
    const double dt = raw_imu_sample.dt_s;

    if (!initialized_)
    {
        return;
    }

    if (dt <= 0.0)
    {
        return;
    }

    if (dt >= 0.1)
    {
        return;
    }

    const double yaw = x_(StateIndex::YAW);
    const double vx = x_(StateIndex::VX);
    const double vy = x_(StateIndex::VY);

    const double alpha = x_(StateIndex::ALPHA);

    const double bg = x_(StateIndex::BG);
    const double bax = x_(StateIndex::BAX);
    const double bay = x_(StateIndex::BAY);

    const double r_corr = raw_imu_sample.gz_frd_rad_s - bg;
    const double ax_corr = raw_imu_sample.ax_frd_m_s2 - bax;
    const double ay_corr = raw_imu_sample.ay_frd_m_s2 - bay;

    const double theta = yaw + alpha;

    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    // const double cy = std::cos(yaw);
    // const double sy = std::sin(yaw);

    StateVec x_dot;
    x_dot.setZero();

    x_dot(StateIndex::PN) = ct * vx - st * vy;
    x_dot(StateIndex::PE) = st * vx + ct * vy;
    x_dot(StateIndex::YAW) = r_corr;
    x_dot(StateIndex::VX) = ax_corr + r_corr * vy;
    x_dot(StateIndex::VY) = ay_corr - r_corr * vx;

    x_ = x_ + dt * x_dot;

    wrap_state_angles();

    StateMat F;
    F.setIdentity();

    // d(pn_next)_d(x)
    F(StateIndex::PN, StateIndex::YAW) = -dt * st * vx - dt * ct * vy;
    F(StateIndex::PN, StateIndex::VX) = dt * ct;
    F(StateIndex::PN, StateIndex::VY) = -dt * st;
    F(StateIndex::PN, StateIndex::ALPHA) = -dt * st * vx - dt * ct * vy;

    // d(pe_next)_d(x)
    F(StateIndex::PE, StateIndex::YAW) = dt * ct * vx - dt * st * vy;
    F(StateIndex::PE, StateIndex::VX) = dt * st;
    F(StateIndex::PE, StateIndex::VY) = dt * ct;
    F(StateIndex::PE, StateIndex::ALPHA) = dt * ct * vx - dt * st * vy;

    // d(yaw_next)_d(x)
    F(StateIndex::YAW, StateIndex::BG) = -dt;

    // d(vx_next)_d(x)
    F(StateIndex::VX, StateIndex::VY) = r_corr * dt;
    F(StateIndex::VX, StateIndex::BG) = -vy * dt;
    F(StateIndex::VX, StateIndex::BAX) = -dt;

    // d(vy_next)_d(x)
    F(StateIndex::VY, StateIndex::VX) = -r_corr * dt;
    F(StateIndex::VY, StateIndex::BG) = vx * dt;
    F(StateIndex::VY, StateIndex::BAY) = -dt;

    // rest are covered by setIdentity()

    P_ = F * P_ * F.transpose() + Q_ * dt;

    enforce_covariance_safety();

    // constrain_vy();
}

UpdateResult Ekf::update_zero_vel(double sigma, double confidence_level)
{
    UpdateResult update_result;
    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(2);

        return update_result;
    }

    // zero velocity measurement
    Eigen::Matrix<double, 2, 1> measurement;
    measurement.setZero();

    Eigen::Matrix<double, 2, 1> z_hat;
    z_hat << x_(StateIndex::VX), x_(StateIndex::VY);

    Eigen::Matrix<double, 2, 1> residual;
    residual = measurement - z_hat;

    // measurement matrix
    Eigen::Matrix<double, 2, StateIndex::SIZE> H;
    H.setZero();
    H(0, StateIndex::VX) = 1.0;
    H(1, StateIndex::VY) = 1.0;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity();
    R *= sigma * sigma;

    update_result = update_generic(residual, H, R, confidence_level);

    return update_result;
}

UpdateResult Ekf::update_zero_lat_velocity(double sigma,
                                           double confidence_level)
{
    UpdateResult update_result;
    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(1);

        return update_result;
    }

    // zero lateral velocity pseudo-measurement
    Eigen::Matrix<double, 1, 1> measurement;
    measurement.setZero();

    Eigen::Matrix<double, 1, 1> z_hat;
    z_hat << x_(StateIndex::VY);

    Eigen::Matrix<double, 1, 1> residual;
    residual = measurement - z_hat;

    // measurement matrix
    Eigen::Matrix<double, 1, StateIndex::SIZE> H;
    H.setZero();
    H(0, StateIndex::VY) = 1.0;

    Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Identity();
    R *= sigma * sigma;

    update_result = update_generic(residual, H, R, confidence_level);

    return update_result;
}

UpdateResult Ekf::update_gnss_speed_magnitude(double speed_meas, double sigma,
                                              double confidence_level)
{
    UpdateResult update_result;
    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(1);

        return update_result;
    }

    Eigen::Matrix<double, 1, 1> measurement;
    measurement << speed_meas;

    const double vx = x_(StateIndex::VX);
    const double vy = x_(StateIndex::VY);

    const double speed_pred = std::sqrt(vx * vx + vy * vy);

    if (speed_pred < 0.05)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(1);

        return update_result;
    }

    Eigen::Matrix<double, 1, 1> z_hat;
    z_hat << speed_pred;

    Eigen::Matrix<double, 1, 1> residual;
    residual = measurement - z_hat;

    Eigen::Matrix<double, 1, StateIndex::SIZE> H;
    H.setZero();
    H(0, StateIndex::VX) = vx / speed_pred;
    H(0, StateIndex::VY) = vy / speed_pred;

    Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Identity();
    R *= sigma * sigma;

    update_result = update_generic(residual, H, R, confidence_level);

    // constrain_vy();
    return update_result;
}

UpdateResult Ekf::update_gnss_velocity_ned(
    double v_n, double v_e, double imu_yaw_rate, bool alpha_course_aligned,
    double gnss_offset_x_frd, double gnss_offset_y_frd, double sigma,
    double confidence_level)
{
    UpdateResult update_result;
    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(2);

        return update_result;
    }

    const double vx = x_(StateIndex::VX);
    const double vy = x_(StateIndex::VY);
    const double yaw = x_(StateIndex::YAW);
    const double alpha = x_(StateIndex::ALPHA);

    const double speed_meas = std::sqrt(v_n * v_n + v_e * v_e);

    if (speed_meas < 1.0)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(2);

        return update_result;
    }

    Eigen::Matrix<double, 2, 1> measurement;
    measurement << v_n, v_e;

    const double theta = yaw + alpha;

    const Eigen::Matrix<double, 2, 2> rot_mat = rot_mat_by_angle(theta);

    Eigen::Matrix<double, 2, 1> v_frd;
    v_frd.setZero();
    v_frd << vx - imu_yaw_rate * gnss_offset_y_frd,
        vy + imu_yaw_rate * gnss_offset_x_frd;

    Eigen::Matrix<double, 2, 1> z_hat;
    z_hat.setZero();
    z_hat = rot_mat * v_frd;

    Eigen::Matrix<double, 2, 1> residual;
    residual = measurement - z_hat;

    const double c = std::cos(theta);
    const double s = std::sin(theta);

    Eigen::Matrix<double, 2, StateIndex::SIZE> H;
    H.setZero();

    const double vbx = v_frd(0);
    const double vby = v_frd(1);

    H(0, StateIndex::YAW) = -vbx * s - vby * c;
    H(0, StateIndex::VX) = c;
    H(0, StateIndex::VY) = -s;
    H(0, StateIndex::ALPHA) = -vbx * s - vby * c;

    H(1, StateIndex::YAW) = vbx * c - vby * s;
    H(1, StateIndex::VX) = s;
    H(1, StateIndex::VY) = c;
    H(1, StateIndex::ALPHA) = vbx * c - vby * s;

    if (alpha_course_aligned)
    {
        H(0, StateIndex::ALPHA) = 0.0;
        H(1, StateIndex::ALPHA) = 0.0;
    }

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity();
    R *= sigma * sigma;

    update_result = update_generic(residual, H, R, confidence_level);

    return update_result;
}

UpdateResult Ekf::update_gnss_position_ned(
    Eigen::Vector2d point_ned_2d, bool alpha_course_aligned,
    double gnss_offset_x_frd, double gnss_offset_y_frd,
    double gnss_initial_heading_ned, double sigma, double confidence_level)
{
    UpdateResult update_result;
    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = Eigen::VectorXd::Zero(2);

        return update_result;
    }

    const double p_n_pred = x_(StateIndex::PN);
    const double p_e_pred = x_(StateIndex::PE);
    const double yaw_w = x_(StateIndex::YAW);
    const double alpha = x_(StateIndex::ALPHA);
    const double gnss_now_heading_ned = yaw_w + alpha;

    const Eigen::Matrix<double, 2, 2> frd_in_ned_now =
        rot_mat_by_angle(gnss_now_heading_ned);
    Eigen::Vector2d gnss_offset_vec_frd = Eigen::Vector2d::Zero();
    gnss_offset_vec_frd << gnss_offset_x_frd, gnss_offset_y_frd;

    const Eigen::Vector2d gnss_offset_vec_ned_now =
        frd_in_ned_now * gnss_offset_vec_frd;

    const Eigen::Matrix<double, 2, 2> frd_in_ned_initial =
        rot_mat_by_angle(gnss_initial_heading_ned);
    const Eigen::Vector2d gnss_offset_vec_ned_initial =
        frd_in_ned_initial * gnss_offset_vec_frd;

    Eigen::Vector2d measurement;
    measurement = point_ned_2d;

    Eigen::Matrix<double, 2, 1> z_hat;
    z_hat.setZero();
    z_hat << (p_n_pred + gnss_offset_vec_ned_now(0) -
              gnss_offset_vec_ned_initial(0)),
        (p_e_pred + gnss_offset_vec_ned_now(1) -
         gnss_offset_vec_ned_initial(1));

    Eigen::Matrix<double, 2, 1> residual;
    residual = measurement - z_hat;

    Eigen::Matrix<double, 2, StateIndex::SIZE> H;
    H.setZero();

    const double c = std::cos(gnss_now_heading_ned);
    const double s = std::sin(gnss_now_heading_ned);

    const double lx = gnss_offset_x_frd;
    const double ly = gnss_offset_y_frd;

    const double d_n_d_theta = -s * lx - c * ly;
    const double d_e_d_theta = c * lx - s * ly;

    H(0, StateIndex::PN) = 1.0;
    H(0, StateIndex::YAW) = d_n_d_theta;
    H(0, StateIndex::ALPHA) = d_n_d_theta;

    H(1, StateIndex::PE) = 1.0;
    H(1, StateIndex::YAW) = d_e_d_theta;
    H(1, StateIndex::ALPHA) = d_e_d_theta;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity();
    R *= sigma * sigma;

    if (alpha_course_aligned)
    {
        H(0, StateIndex::ALPHA) = 0.0;
        H(1, StateIndex::ALPHA) = 0.0;
    }

    update_result = update_generic(residual, H, R, confidence_level);

    return update_result;
}

UpdateResult Ekf::update_generic(const Eigen::VectorXd& residual,
                                 const Eigen::MatrixXd& H,
                                 const Eigen::MatrixXd& R,
                                 double confidence_level)
{
    UpdateResult update_result;

    if (!initialized_)
    {
        update_result.attempted = false;
        update_result.accepted = false;
        update_result.nis = 0.0;
        update_result.threshold = 0.0;
        update_result.residual = residual;

        return update_result;
    }

    // innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    NisGateResult gate_result = pass_nis_gate(residual, S, confidence_level);

    update_result.attempted = true;
    update_result.accepted = gate_result.passed;
    update_result.nis = gate_result.nis;
    update_result.threshold = gate_result.threshold;
    update_result.residual = residual;

    if (!gate_result.passed)
    {
        wrap_state_angles();
        enforce_covariance_safety();

        return update_result;
    }

    // kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // correction vector
    Eigen::VectorXd correction = K * residual;

    // state update
    x_ = x_ + correction;

    // wrap state angles
    wrap_state_angles();

    // Joseph form covariance update
    Eigen::Matrix<double, StateIndex::SIZE, StateIndex::SIZE> I;
    I.setIdentity();
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();

    // check covariance
    enforce_covariance_safety();

    return update_result;
}

NisGateResult Ekf::pass_nis_gate(const Eigen::VectorXd& residual,
                                 const Eigen::MatrixXd& S,
                                 double confidence_level)
{
    NisGateResult gate_result;

    const double nis = residual.transpose() * S.inverse() * residual;
    double threshold =
        get_chi_square_threshold(residual.rows(), confidence_level);

    gate_result.nis = nis;
    gate_result.threshold = threshold;

    if (nis <= threshold)
    {
        gate_result.passed = true;
    }
    else
    {
        gate_result.passed = false;
    }

    return gate_result;
}

double Ekf::get_chi_square_threshold(int meas_dim, double confidence_level)
{
    boost::math::chi_squared dist(meas_dim);

    confidence_level = std::clamp(confidence_level, 1e-6, 1.0 - 1e-6);

    return boost::math::quantile(dist, confidence_level);
}

void Ekf::constrain_vy()
{
    if (x_(StateIndex::VY) > max_vy_mag_)
    {
        x_(StateIndex::VY) = max_vy_mag_;
    }

    if (x_(StateIndex::VY) < -max_vy_mag_)
    {
        x_(StateIndex::VY) = -max_vy_mag_;
    }
}

}  // namespace htx_ekf
