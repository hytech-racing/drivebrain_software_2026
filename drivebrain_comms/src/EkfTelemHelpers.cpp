#include "EkfTelemHelpers.hpp"

#include <memory>

#include "Telemetry.hpp"
#include "htx_ekf/ekf_manager.hpp"
#include "hytech_msgs.pb.h"

namespace comms
{

namespace
{
hytech_msgs::EkfInitStatus to_proto_init_status(htx_ekf::InitStatus status)
{
    switch (status)
    {
        case htx_ekf::InitStatus::Waiting:
            return hytech_msgs::EKF_INIT_STATUS_WAITING;

        case htx_ekf::InitStatus::InProgress:
            return hytech_msgs::EKF_INIT_STATUS_IN_PROGRESS;

        case htx_ekf::InitStatus::Done:
            return hytech_msgs::EKF_INIT_STATUS_DONE;

        case htx_ekf::InitStatus::Failed:
            return hytech_msgs::EKF_INIT_STATUS_FAILED;

        case htx_ekf::InitStatus::Unspecified:
            return hytech_msgs::EKF_INIT_STATUS_UNSPECIFIED;

        default:
            return hytech_msgs::EKF_INIT_STATUS_UNSPECIFIED;
    }
}
}  // namespace

std::shared_ptr<hytech_msgs::EkfState> make_ekf_state_msg(
    const htx_ekf::EkfOutput& state)
{
    auto msg = std::make_shared<hytech_msgs::EkfState>();

    msg->set_time_startup_ns(state.time_startup_ns);

    msg->set_initialized(state.initialized);

    msg->set_vx_vehicle_frd_m_s(state.vx_vehicle_frd_m_s);
    msg->set_vy_vehicle_frd_m_s(state.vy_vehicle_frd_m_s);
    msg->set_yaw_rate_corrected_vehicle_frd_rad_s(
        state.yaw_rate_corrected_vehicle_frd_rad_s);

    msg->set_vx_vehicle_flu_m_s(state.vx_vehicle_flu_m_s);
    msg->set_vy_vehicle_flu_m_s(state.vy_vehicle_flu_m_s);
    msg->set_yaw_rate_corrected_vehicle_flu_rad_s(
        state.yaw_rate_corrected_vehicle_flu_rad_s);

    msg->set_position_n_m(state.position_n_m);
    msg->set_position_e_m(state.position_e_m);

    msg->set_yaw_vehicle_frd_rad(state.yaw_vehicle_frd_rad);
    msg->set_yaw_vehicle_flu_rad(state.yaw_vehicle_flu_rad);

    msg->set_alpha_startup_to_ned_rad(state.alpha_startup_to_ned_rad);

    msg->set_bias_gyro_vehicle_frd_rad_s(state.bg_rad_s);
    msg->set_bias_ax_vehicle_frd_m_s2(state.bax_m_s2);
    msg->set_bias_ay_vehicle_frd_m_s2(state.bay_m_s2);

    msg->set_ax_corrected_vehicle_frd_m_s2(state.ax_corrected_vehicle_frd_m_s2);
    msg->set_ay_corrected_vehicle_frd_m_s2(state.ay_corrected_vehicle_frd_m_s2);

    msg->set_ax_corrected_vehicle_flu_m_s2(state.ax_corrected_vehicle_flu_m_s2);
    msg->set_ay_corrected_vehicle_flu_m_s2(state.ay_corrected_vehicle_flu_m_s2);

    msg->set_last_imu_dt_s(state.last_imu_dt_s);

    return msg;
}

std::shared_ptr<hytech_msgs::EkfDebugSnapshot> make_ekf_debug_snapshot_msg(
    const htx_ekf::EkfDebugSnapshot& debug)
{
    auto msg = std::make_shared<hytech_msgs::EkfDebugSnapshot>();

    msg->set_time_startup_ns(debug.time_startup_ns);

    // initialization status
    msg->set_imu_status(to_proto_init_status(debug.imu_status));
    msg->set_gnss1_status(to_proto_init_status(debug.gnss1_status));
    msg->set_gnss2_status(to_proto_init_status(debug.gnss2_status));
    msg->set_alpha_status(to_proto_init_status(debug.alpha_status));

    // IMU initialization progress
    msg->set_imu_sample_count(static_cast<uint32_t>(debug.imu_sample_count));
    msg->set_imu_required_samples(
        static_cast<uint32_t>(debug.imu_required_samples));
    msg->set_imu_init_progress(debug.imu_init_progress);

    // GNSS1 initialization progress
    msg->set_gnss1_sample_count(
        static_cast<uint32_t>(debug.gnss1_sample_count));
    msg->set_gnss1_required_samples(
        static_cast<uint32_t>(debug.gnss1_required_samples));
    msg->set_gnss1_init_progress(debug.gnss1_init_progress);

    // GNSS2 initialization progress
    msg->set_gnss2_sample_count(
        static_cast<uint32_t>(debug.gnss2_sample_count));
    msg->set_gnss2_required_samples(
        static_cast<uint32_t>(debug.gnss2_required_samples));
    msg->set_gnss2_init_progress(debug.gnss2_init_progress);

    // stationary detection
    msg->set_stationary_detected(debug.stationary_detected);

    // zero-velocity update debug
    msg->set_latest_zero_vel_update_enabled(
        debug.latest_zero_vel_update_enabled);
    msg->set_latest_zero_vel_update_accepted(
        debug.latest_zero_vel_update_accepted);
    msg->set_latest_zero_vel_nis(debug.latest_zero_vel_nis);
    msg->set_latest_zero_vel_gate(debug.latest_zero_vel_gate);

    // zero-lat-velocity update debug
    msg->set_latest_zero_lat_vel_update_enabled(
        debug.latest_zero_lat_vel_update_enabled);
    msg->set_latest_zero_lat_vel_update_accepted(
        debug.latest_zero_lat_vel_update_accepted);
    msg->set_latest_zero_lat_vel_nis(debug.latest_zero_lat_vel_nis);
    msg->set_latest_zero_lat_vel_gate(debug.latest_zero_lat_vel_gate);
    msg->set_latest_zero_lat_vel_sigma_used(
        debug.latest_zero_lat_vel_sigma_used);
    msg->set_latest_zero_lat_vel_residual(debug.latest_zero_lat_vel_residual);

    // GNSS1 speed update debug
    msg->set_latest_gnss1_speed_update_enabled(
        debug.latest_gnss1_speed_update_enabled);
    msg->set_latest_gnss1_speed_update_accepted(
        debug.latest_gnss1_speed_update_accepted);
    msg->set_latest_gnss1_speed_nis(debug.latest_gnss1_speed_nis);
    msg->set_latest_gnss1_speed_gate(debug.latest_gnss1_speed_gate);
    msg->set_latest_gnss1_speed_residual(debug.latest_gnss1_speed_residual);

    // GNSS1 velocity update debug
    msg->set_latest_gnss1_vel_update_enabled(
        debug.latest_gnss1_vel_update_enabled);
    msg->set_latest_gnss1_vel_update_accepted(
        debug.latest_gnss1_vel_update_accepted);
    msg->set_latest_gnss1_vel_nis(debug.latest_gnss1_vel_nis);
    msg->set_latest_gnss1_vel_gate(debug.latest_gnss1_vel_gate);
    msg->set_latest_gnss1_vel_ned_n_residual(
        debug.latest_gnss1_vel_ned_n_residual);
    msg->set_latest_gnss1_vel_ned_e_residual(
        debug.latest_gnss1_vel_ned_e_residual);

    // GNSS1 position update debug
    msg->set_latest_gnss1_pos_update_enabled(
        debug.latest_gnss1_pos_update_enabled);
    msg->set_latest_gnss1_pos_update_accepted(
        debug.latest_gnss1_pos_update_accepted);
    msg->set_latest_gnss1_pos_nis(debug.latest_gnss1_pos_nis);
    msg->set_latest_gnss1_pos_gate(debug.latest_gnss1_pos_gate);
    msg->set_latest_gnss1_pos_ned_n_residual(
        debug.latest_gnss1_pos_ned_n_residual);
    msg->set_latest_gnss1_pos_ned_e_residual(
        debug.latest_gnss1_pos_ned_e_residual);

    // GNSS2 speed update debug
    msg->set_latest_gnss2_speed_update_enabled(
        debug.latest_gnss2_speed_update_enabled);
    msg->set_latest_gnss2_speed_update_accepted(
        debug.latest_gnss2_speed_update_accepted);
    msg->set_latest_gnss2_speed_nis(debug.latest_gnss2_speed_nis);
    msg->set_latest_gnss2_speed_gate(debug.latest_gnss2_speed_gate);
    msg->set_latest_gnss2_speed_residual(debug.latest_gnss2_speed_residual);

    // GNSS2 velocity update debug
    msg->set_latest_gnss2_vel_update_enabled(
        debug.latest_gnss2_vel_update_enabled);
    msg->set_latest_gnss2_vel_update_accepted(
        debug.latest_gnss2_vel_update_accepted);
    msg->set_latest_gnss2_vel_nis(debug.latest_gnss2_vel_nis);
    msg->set_latest_gnss2_vel_gate(debug.latest_gnss2_vel_gate);
    msg->set_latest_gnss2_vel_ned_n_residual(
        debug.latest_gnss2_vel_ned_n_residual);
    msg->set_latest_gnss2_vel_ned_e_residual(
        debug.latest_gnss2_vel_ned_e_residual);

    // GNSS2 position update debug
    msg->set_latest_gnss2_pos_update_enabled(
        debug.latest_gnss2_pos_update_enabled);
    msg->set_latest_gnss2_pos_update_accepted(
        debug.latest_gnss2_pos_update_accepted);
    msg->set_latest_gnss2_pos_nis(debug.latest_gnss2_pos_nis);
    msg->set_latest_gnss2_pos_gate(debug.latest_gnss2_pos_gate);
    msg->set_latest_gnss2_pos_ned_n_residual(
        debug.latest_gnss2_pos_ned_n_residual);
    msg->set_latest_gnss2_pos_ned_e_residual(
        debug.latest_gnss2_pos_ned_e_residual);

    // measurement sigma actually used
    msg->set_latest_gnss1_speed_sigma_used(debug.latest_gnss1_speed_sigma_used);
    msg->set_latest_gnss1_vel_sigma_used(debug.latest_gnss1_vel_sigma_used);
    msg->set_latest_gnss1_pos_sigma_used(debug.latest_gnss1_pos_sigma_used);

    msg->set_latest_gnss2_speed_sigma_used(debug.latest_gnss2_speed_sigma_used);
    msg->set_latest_gnss2_vel_sigma_used(debug.latest_gnss2_vel_sigma_used);
    msg->set_latest_gnss2_pos_sigma_used(debug.latest_gnss2_pos_sigma_used);

    // latest GNSS measurement validity flags
    msg->set_latest_gnss1_vel_valid(debug.latest_gnss1_vel_valid);
    msg->set_latest_gnss1_pos_valid(debug.latest_gnss1_pos_valid);

    msg->set_latest_gnss2_vel_valid(debug.latest_gnss2_vel_valid);
    msg->set_latest_gnss2_pos_valid(debug.latest_gnss2_pos_valid);

    return msg;
}

void publish_ekf_step_result(const htx_ekf::EkfStepResult& result)
{
    if (result.has_state)
    {
        std::shared_ptr<hytech_msgs::EkfState> state_msg =
            make_ekf_state_msg(result.state);

        core::log(
            std::static_pointer_cast<google::protobuf::Message>(state_msg));
    }

    if (result.has_debug_snapshot)
    {
        std::shared_ptr<hytech_msgs::EkfDebugSnapshot> debug_msg =
            make_ekf_debug_snapshot_msg(result.debug_snapshot);

        core::log(
            std::static_pointer_cast<google::protobuf::Message>(debug_msg));
    }
}
}  // namespace comms