#include <StateTracker.hpp>
#include <memory>
#include <mutex>

#include "Telemetry.hpp"
#include "hytech_msgs.pb.h"

using namespace core;

/****************************************************************
 * Public class methods
 ****************************************************************/
void core::StateTracker::create()
{
    StateTracker* expected = nullptr;
    StateTracker* local = new StateTracker();
    if (!_s_instance.compare_exchange_strong(expected, local,
                                             std::memory_order_release,
                                             std::memory_order_relaxed))
    {
        // Already initialized, delete local instance
        delete local;
    }
}

core::StateTracker& core::StateTracker::instance()
{
    StateTracker* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "StateTracker has not been initialized");
    return *instance;
}

void core::StateTracker::destroy()
{
    StateTracker* instance =
        _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance)
    {
        delete instance;
    }
}

void StateTracker::set_previous_control_output(
    core::ControllerOutput& prev_controller_output)
{
    std::unique_lock lk(_state_mutex);
    _vehicle_state.prev_controller_output = prev_controller_output;
}

void StateTracker::handle_receive_protobuf_message(
    std::shared_ptr<google::protobuf::Message> msg)
{
    if (msg->GetDescriptor() == hytech_msgs::VnImuData::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VnImuData>(msg);

        xyz_vec<float> body_accel_flu_mss = {
            in_msg->comp_accel_vehicle_flu_m_ss().x(),
            in_msg->comp_accel_vehicle_flu_m_ss().y(),
            in_msg->comp_accel_vehicle_flu_m_ss().z()};

        xyz_vec<float> body_gyro_rate_flu_rads = {
            in_msg->comp_gyro_vehicle_flu_rad_s().x(),
            in_msg->comp_gyro_vehicle_flu_rad_s().y(),
            in_msg->comp_gyro_vehicle_flu_rad_s().z()};

        {
            std::unique_lock lk(_state_mutex);

            _vehicle_state.current_body_accel_mss = body_accel_flu_mss;
            _vehicle_state.current_angular_rate_rads.x =
                body_gyro_rate_flu_rads.x;  // only use x and y from vn
            _vehicle_state.current_angular_rate_rads.y =
                body_gyro_rate_flu_rads.y;
        }
    }
    else if (msg->GetDescriptor() == hytech_msgs::VnYprData::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VnYprData>(msg);

        ypr_vec<float> ypr_deg = {in_msg->ypr_deg().yaw(),
                                  in_msg->ypr_deg().pitch(),
                                  in_msg->ypr_deg().roll()};

        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.current_ypr_deg = ypr_deg;
        }
    }
    else if (msg->GetDescriptor() == hytech_msgs::VnInsData::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VnInsData>(msg);

        xyz_vec<float> body_vel_flu_ms = {
            in_msg->vel_body_vehicle_flu_m_s().x(),
            in_msg->vel_body_vehicle_flu_m_s().y(),
            in_msg->vel_body_vehicle_flu_m_s().z(),
        };

        auto ins_mode_int = in_msg->ins_status().ins_mode_int();
        auto vel_u = in_msg->ins_status().ins_vel_u();

        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.current_body_vel_ms.z =
                body_vel_flu_ms.z;  // only use z from vn
            _vehicle_state.ins_status.status_mode = ins_mode_int;
            _vehicle_state.ins_status.vel_uncertainty = vel_u;
        }
    }
    else if (msg->GetDescriptor() == hytech_msgs::VNData::descriptor())
    {
        // the general VNData shouldn't be in use anymore
    }
    else if (msg->GetDescriptor() == hytech_msgs::VCRData_s::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VCRData_s>(msg);
        bool is_rtd = (in_msg->status().vehicle_state() ==
                       hytech_msgs::VehicleState_e::READY_TO_DRIVE);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.is_ready_to_drive = is_rtd;
        }
    }
    else if (msg->GetDescriptor() == hytech_msgs::ACUAllData::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::ACUAllData>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.acc_data.min_cell_voltage =
                in_msg->core_data().min_cell_voltage();
        }
    }
    else if (msg->GetDescriptor() == hytech_msgs::EkfState::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::EkfState>(msg);
        {
            if (in_msg->initialized())
            {
                std::unique_lock lk(_state_mutex);
                _vehicle_state.current_body_vel_ms.x =
                    in_msg->vx_vehicle_flu_m_s();
                // _vehicle_state.current_body_vel_ms.x = 1.50;
                _vehicle_state.current_body_vel_ms.y =
                    in_msg->vy_vehicle_flu_m_s();
                _vehicle_state.current_angular_rate_rads.z =
                    in_msg->yaw_rate_corrected_vehicle_flu_rad_s();
                // omit yaw angle for now and use vectornav's, maybe in the
                // future separate vn states and ekf states

                // omit both accel for now and use vectornav's
            }
        }

        bool state_is_valid = _validate_timestamps(_timestamp_array);

        core::VehicleState snapshot;
        {
            std::unique_lock lk(_state_mutex);
            snapshot = _vehicle_state;
        }

        snapshot.state_is_valid = state_is_valid;

        auto vehicle_data_msg = std::make_shared<hytech_msgs::VehicleData>();

        vehicle_data_msg->set_is_ready_to_drive(snapshot.is_ready_to_drive);
        vehicle_data_msg->set_state_is_valid(snapshot.state_is_valid);
        vehicle_data_msg->set_prev_mcu_recv_millis(
            snapshot.prev_MCU_recv_millis);
        vehicle_data_msg->set_steering_angle_deg(snapshot.steering_angle_deg);
        vehicle_data_msg->set_old_energy_meter_kw(snapshot.old_energy_meter_kw);
        vehicle_data_msg->set_electrical_power_watts(
            snapshot.old_energy_meter_kw * 1000.0f);
        // vehicle_data_msg->set_is_using_torque_controller(
        //     std::holds_alternative<core::TorqueControlOut>(snapshot.prev_controller_output));

        // set current inputs submessage
        auto* input_msg = vehicle_data_msg->mutable_current_inputs();
        input_msg->set_accel_percent(snapshot.input.requested_accel);
        input_msg->set_brake_percent(snapshot.input.requested_brake);

        auto* current_body_vel_msg =
            vehicle_data_msg->mutable_current_body_vel_ms();
        current_body_vel_msg->set_x(snapshot.current_body_vel_ms.x);
        current_body_vel_msg->set_y(snapshot.current_body_vel_ms.y);
        current_body_vel_msg->set_z(snapshot.current_body_vel_ms.z);

        auto* current_body_accel_msg =
            vehicle_data_msg->mutable_current_body_accel_mss();
        current_body_accel_msg->set_x(snapshot.current_body_accel_mss.x);
        current_body_accel_msg->set_y(snapshot.current_body_accel_mss.y);
        current_body_accel_msg->set_z(snapshot.current_body_accel_mss.z);

        auto* current_angular_rate_msg =
            vehicle_data_msg->mutable_current_angular_rate_rads();
        current_angular_rate_msg->set_x(snapshot.current_angular_rate_rads.x);
        current_angular_rate_msg->set_y(snapshot.current_angular_rate_rads.y);
        current_angular_rate_msg->set_z(snapshot.current_angular_rate_rads.z);

        constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
        auto* current_ypr_msg = vehicle_data_msg->mutable_current_ypr_rad();
        current_ypr_msg->set_yaw(snapshot.current_ypr_deg.yaw * kDegToRad);
        current_ypr_msg->set_pitch(snapshot.current_ypr_deg.pitch * kDegToRad);
        current_ypr_msg->set_roll(snapshot.current_ypr_deg.roll * kDegToRad);

        auto* current_rpms_msg = vehicle_data_msg->mutable_current_rpms();
        current_rpms_msg->set_fl(snapshot.current_rpms.FL);
        current_rpms_msg->set_fr(snapshot.current_rpms.FR);
        current_rpms_msg->set_rl(snapshot.current_rpms.RL);
        current_rpms_msg->set_rr(snapshot.current_rpms.RR);

        auto* tire_dyn_msg = vehicle_data_msg->mutable_tire_dynamics();
        auto* tire_forces_msg = tire_dyn_msg->mutable_tire_forces_n();
        tire_forces_msg->mutable_fl()->set_x(
            snapshot.tire_dynamics.tire_forces_n.FL.x);
        tire_forces_msg->mutable_fl()->set_y(
            snapshot.tire_dynamics.tire_forces_n.FL.y);
        tire_forces_msg->mutable_fl()->set_z(
            snapshot.tire_dynamics.tire_forces_n.FL.z);
        tire_forces_msg->mutable_fr()->set_x(
            snapshot.tire_dynamics.tire_forces_n.FR.x);
        tire_forces_msg->mutable_fr()->set_y(
            snapshot.tire_dynamics.tire_forces_n.FR.y);
        tire_forces_msg->mutable_fr()->set_z(
            snapshot.tire_dynamics.tire_forces_n.FR.z);
        tire_forces_msg->mutable_rl()->set_x(
            snapshot.tire_dynamics.tire_forces_n.RL.x);
        tire_forces_msg->mutable_rl()->set_y(
            snapshot.tire_dynamics.tire_forces_n.RL.y);
        tire_forces_msg->mutable_rl()->set_z(
            snapshot.tire_dynamics.tire_forces_n.RL.z);
        tire_forces_msg->mutable_rr()->set_x(
            snapshot.tire_dynamics.tire_forces_n.RR.x);
        tire_forces_msg->mutable_rr()->set_y(
            snapshot.tire_dynamics.tire_forces_n.RR.y);
        tire_forces_msg->mutable_rr()->set_z(
            snapshot.tire_dynamics.tire_forces_n.RR.z);

        auto* tire_moments_msg = tire_dyn_msg->mutable_tire_moments_nm();
        tire_moments_msg->mutable_fl()->set_x(
            snapshot.tire_dynamics.tire_moments_nm.FL.x);
        tire_moments_msg->mutable_fl()->set_y(
            snapshot.tire_dynamics.tire_moments_nm.FL.y);
        tire_moments_msg->mutable_fl()->set_z(
            snapshot.tire_dynamics.tire_moments_nm.FL.z);
        tire_moments_msg->mutable_fr()->set_x(
            snapshot.tire_dynamics.tire_moments_nm.FR.x);
        tire_moments_msg->mutable_fr()->set_y(
            snapshot.tire_dynamics.tire_moments_nm.FR.y);
        tire_moments_msg->mutable_fr()->set_z(
            snapshot.tire_dynamics.tire_moments_nm.FR.z);
        tire_moments_msg->mutable_rl()->set_x(
            snapshot.tire_dynamics.tire_moments_nm.RL.x);
        tire_moments_msg->mutable_rl()->set_y(
            snapshot.tire_dynamics.tire_moments_nm.RL.y);
        tire_moments_msg->mutable_rl()->set_z(
            snapshot.tire_dynamics.tire_moments_nm.RL.z);
        tire_moments_msg->mutable_rr()->set_x(
            snapshot.tire_dynamics.tire_moments_nm.RR.x);
        tire_moments_msg->mutable_rr()->set_y(
            snapshot.tire_dynamics.tire_moments_nm.RR.y);
        tire_moments_msg->mutable_rr()->set_z(
            snapshot.tire_dynamics.tire_moments_nm.RR.z);

        auto* accel_sat_msg = tire_dyn_msg->mutable_accel_saturation_nm();
        accel_sat_msg->set_fl(snapshot.tire_dynamics.accel_saturation_nm.FL);
        accel_sat_msg->set_fr(snapshot.tire_dynamics.accel_saturation_nm.FR);
        accel_sat_msg->set_rl(snapshot.tire_dynamics.accel_saturation_nm.RL);
        accel_sat_msg->set_rr(snapshot.tire_dynamics.accel_saturation_nm.RR);

        auto* brake_sat_msg = tire_dyn_msg->mutable_brake_saturation_nm();
        brake_sat_msg->set_fl(snapshot.tire_dynamics.brake_saturation_nm.FL);
        brake_sat_msg->set_fr(snapshot.tire_dynamics.brake_saturation_nm.FR);
        brake_sat_msg->set_rl(snapshot.tire_dynamics.brake_saturation_nm.RL);
        brake_sat_msg->set_rr(snapshot.tire_dynamics.brake_saturation_nm.RR);

        vehicle_data_msg->set_v_y_lm(snapshot.tire_dynamics.v_y_lm);
        vehicle_data_msg->set_psi_dot_lm_rad_s(
            snapshot.tire_dynamics.psi_dot_lm_rad_s);

        auto* driver_torque_msg = vehicle_data_msg->mutable_driver_torque();
        driver_torque_msg->set_fl(snapshot.driver_torque.FL);
        driver_torque_msg->set_fr(snapshot.driver_torque.FR);
        driver_torque_msg->set_rl(snapshot.driver_torque.RL);
        driver_torque_msg->set_rr(snapshot.driver_torque.RR);

        auto* tv_status_msg = vehicle_data_msg->mutable_tv_status();
        auto* tv_torque_additional_msg =
            tv_status_msg->mutable_torque_additional_nm();
        // tv_torque_additional_msg->set_fl(snapshot.matlab_math_temp_out.torque_vectoring_status.torque_additional_nm.FL);
        // tv_torque_additional_msg->set_fr(snapshot.matlab_math_temp_out.torque_vectoring_status.torque_additional_nm.FR);
        // tv_torque_additional_msg->set_rl(snapshot.matlab_math_temp_out.torque_vectoring_status.torque_additional_nm.RL);
        // tv_torque_additional_msg->set_rr(snapshot.matlab_math_temp_out.torque_vectoring_status.torque_additional_nm.RR);
        // tv_status_msg->set_additional_mz_moment_nm(snapshot.matlab_math_temp_out.torque_vectoring_status.additional_mz_moment_nm);
        // tv_status_msg->set_des_psi_dot(snapshot.matlab_math_temp_out.torque_vectoring_status.des_psi_dot);
        // tv_status_msg->set_psi_dot_err(snapshot.matlab_math_temp_out.torque_vectoring_status.psi_dot_err);
        // tv_status_msg->set_perceived_vx(snapshot.matlab_math_temp_out.torque_vectoring_status.perceived_vx);
        // tv_status_msg->set_integral_yaw_rate_err(snapshot.matlab_math_temp_out.torque_vectoring_status.integral_yaw_rate_err);
        // tv_status_msg->set_perceived_psi_dot(snapshot.matlab_math_temp_out.torque_vectoring_status.perceived_psi_dot);
        // tv_status_msg->set_psi_dot_gain(snapshot.matlab_math_temp_out.torque_vectoring_status.psi_dot_gain);
        // tv_status_msg->set_vy_vn_gain(snapshot.matlab_math_temp_out.torque_vectoring_status.vy_vn_gain);
        // tv_status_msg->set_perceived_vy(snapshot.matlab_math_temp_out.torque_vectoring_status.perceived_vy);

        auto* power_limit_msg = vehicle_data_msg->mutable_power_limit_status();
        power_limit_msg->set_power_limit_status(false);

        auto* suspension_msg =
            vehicle_data_msg->mutable_suspension_potentiometers_mm();
        suspension_msg->set_fl(snapshot.suspension_potentiometers_mm.FL);
        suspension_msg->set_fr(snapshot.suspension_potentiometers_mm.FR);
        suspension_msg->set_rl(snapshot.suspension_potentiometers_mm.RL);
        suspension_msg->set_rr(snapshot.suspension_potentiometers_mm.RR);

        auto* current_torques_msg =
            vehicle_data_msg->mutable_current_torques_nm();
        current_torques_msg->set_fl(snapshot.current_torques_nm.FL);
        current_torques_msg->set_fr(snapshot.current_torques_nm.FR);
        current_torques_msg->set_rl(snapshot.current_torques_nm.RL);
        current_torques_msg->set_rr(snapshot.current_torques_nm.RR);

        core::log(std::static_pointer_cast<google::protobuf::Message>(
            vehicle_data_msg));
    }
    else
    {
        _receive_low_level_state(msg);
    }
}

std::pair<core::VehicleState, bool>
StateTracker::get_latest_state_and_validity()
{
    auto state_is_valid = _validate_timestamps(_timestamp_array);

    VehicleState current_state = {};

    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state.state_is_valid = state_is_valid;
        current_state = _vehicle_state;
    }

    return {current_state, state_is_valid};
}

/****************************************************************
 * Private class methods
 ****************************************************************/
template <size_t ind, typename inverter_dynamics_message>
void StateTracker::_handle_set_inverter_dynamics(
    std::shared_ptr<google::protobuf::Message> msg)
{
    auto in_msg = std::static_pointer_cast<inverter_dynamics_message>(msg);
    {
        std::unique_lock lk(_state_mutex);
        _raw_input_data.raw_inverter_torques.set_from_index<ind>(
            in_msg->actual_torque_nm());
        _raw_input_data.raw_inverter_power.set_from_index<ind>(
            in_msg->actual_power_w());
        _vehicle_state.current_rpms.set_from_index<ind>(
            in_msg->actual_speed_rpm());
        _vehicle_state.current_torques_nm =
            _raw_input_data.raw_inverter_torques;
    }
}

template <size_t ind, typename inverter_temps_message>
void StateTracker::_handle_set_inverter_temps(
    std::shared_ptr<google::protobuf::Message> msg)
{
    auto in_msg = std::static_pointer_cast<inverter_temps_message>(msg);
    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state.dt_data.inverter_igbt_temps_c.set_from_index<ind>(
            in_msg->igbt_temp());
        _vehicle_state.dt_data.inverter_temps_c.set_from_index<ind>(
            in_msg->inverter_temp());
        _vehicle_state.dt_data.inverter_motor_temps_c.set_from_index<ind>(
            in_msg->motor_temp());
    }
}

void StateTracker::_receive_low_level_state(
    std::shared_ptr<google::protobuf::Message> msg)
{
    if (msg->GetDescriptor() == hytech::rear_suspension::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::rear_suspension>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[0] =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now()
                        .time_since_epoch());
            _raw_input_data.raw_load_cell_values.RL = in_msg->rl_load_cell();
            _raw_input_data.raw_load_cell_values.RR = in_msg->rr_load_cell();
            _raw_input_data.raw_shock_pot_values.RL = in_msg->rl_shock_pot();
            _raw_input_data.raw_shock_pot_values.RR = in_msg->rr_shock_pot();
            _vehicle_state.loadcells = _raw_input_data.raw_load_cell_values;
            _vehicle_state.suspension_potentiometers_mm =
                _raw_input_data.raw_shock_pot_values;
        }
    }
    else if (msg->GetDescriptor() == hytech::front_suspension::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::front_suspension>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[1] =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now()
                        .time_since_epoch());
            _raw_input_data.raw_load_cell_values.FL = in_msg->fl_load_cell();
            _raw_input_data.raw_load_cell_values.FR = in_msg->fr_load_cell();
            _raw_input_data.raw_shock_pot_values.FL = in_msg->fl_shock_pot();
            _raw_input_data.raw_shock_pot_values.FR = in_msg->fr_shock_pot();
            _vehicle_state.loadcells = _raw_input_data.raw_load_cell_values;
            _vehicle_state.suspension_potentiometers_mm =
                _raw_input_data.raw_shock_pot_values;
        }
    }
    else if (msg->GetDescriptor() == hytech::pedals_system_data::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::pedals_system_data>(msg);
        core::DriverInput input = {(in_msg->accel_pedal()),
                                   (in_msg->brake_pedal())};
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[2] =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now()
                        .time_since_epoch());
            _vehicle_state.input = input;
        }
    }
    else if (msg->GetDescriptor() == hytech::steering_data::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::steering_data>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[3] =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now()
                        .time_since_epoch());
            _raw_input_data.raw_steering_analog = in_msg->steering_analog_raw();
            _raw_input_data.raw_steering_digital =
                in_msg->steering_digital_raw();
            _vehicle_state.steering_angle_deg =
                _raw_input_data.raw_steering_analog;
        }
    }
    else if (msg->GetDescriptor() == hytech::em_measurement::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::em_measurement>(msg);
        float em_voltage = in_msg->em_voltage();
        float em_current = in_msg->em_current();
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.old_energy_meter_kw =
                (-1.0f * em_voltage * em_current / 1000.0f);
        }
    }
    else
    {
        _receive_inverter_states(msg);
    }
}

void StateTracker::_receive_inverter_states(
    std::shared_ptr<google::protobuf::Message> msg)
{
    auto descriptor = msg->GetDescriptor();
    if (descriptor == hytech::inv1_dynamics::descriptor())
    {
        _handle_set_inverter_dynamics<0, hytech::inv1_dynamics>(msg);
    }
    else if (descriptor == hytech::inv2_dynamics::descriptor())
    {
        _handle_set_inverter_dynamics<1, hytech::inv2_dynamics>(msg);
    }
    else if (descriptor == hytech::inv3_dynamics::descriptor())
    {
        _handle_set_inverter_dynamics<2, hytech::inv3_dynamics>(msg);
    }
    else if (descriptor == hytech::inv4_dynamics::descriptor())
    {
        _handle_set_inverter_dynamics<3, hytech::inv4_dynamics>(msg);
    }
    else if (descriptor == hytech::inv1_temps::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv1_temps>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.dt_data.inverter_igbt_temps_c.FL =
                in_msg->igbt_temp();
            _vehicle_state.dt_data.inverter_temps_c.FL =
                in_msg->inverter_temp();
            _vehicle_state.dt_data.inverter_motor_temps_c.FL =
                in_msg->motor_temp();
        }
    }
    else if (descriptor == hytech::inv2_temps::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv2_temps>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.dt_data.inverter_igbt_temps_c.FR =
                in_msg->igbt_temp();
            _vehicle_state.dt_data.inverter_temps_c.FR =
                in_msg->inverter_temp();
            _vehicle_state.dt_data.inverter_motor_temps_c.FR =
                in_msg->motor_temp();
        }
    }
    else if (descriptor == hytech::inv3_temps::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv3_temps>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.dt_data.inverter_igbt_temps_c.RL =
                in_msg->igbt_temp();
            _vehicle_state.dt_data.inverter_temps_c.RL =
                in_msg->inverter_temp();
            _vehicle_state.dt_data.inverter_motor_temps_c.RL =
                in_msg->motor_temp();
        }
    }
    else if (descriptor == hytech::inv4_temps::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv4_temps>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.dt_data.inverter_igbt_temps_c.RR =
                in_msg->igbt_temp();
            _vehicle_state.dt_data.inverter_temps_c.RR =
                in_msg->inverter_temp();
            _vehicle_state.dt_data.inverter_motor_temps_c.RR =
                in_msg->motor_temp();
        }
    }
    else if (descriptor == hytech::inv1_overload::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv1_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.FL =
                in_msg->motor_overload_percentage();
        }
    }
    else if (descriptor == hytech::inv2_overload::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv2_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.FR =
                in_msg->motor_overload_percentage();
        }
    }
    else if (descriptor == hytech::inv3_overload::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv3_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.RL =
                in_msg->motor_overload_percentage();
        }
    }
    else if (descriptor == hytech::inv4_overload::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv4_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.RR =
                in_msg->motor_overload_percentage();
        }
    }
    else if (descriptor == hytech::inv1_status::descriptor())
    {
        auto in_msg = std::static_pointer_cast<hytech::inv1_status>(msg);
        auto voltage = in_msg->dc_bus_voltage();
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.acc_data.pack_voltage = static_cast<float>(voltage);
        }
    }
}

template <size_t arr_len>
bool StateTracker::_validate_timestamps(
    const std::array<std::chrono::microseconds, arr_len>& timestamp_arr)
{
    std::array<std::chrono::microseconds, arr_len> timestamp_array_to_sort;

    {
        std::unique_lock lk(_state_mutex);
        timestamp_array_to_sort = timestamp_arr;
    }

    auto debug_copy = timestamp_array_to_sort;
    const std::chrono::microseconds threshold(
        500000);  // 500 milliseconds in microseconds

    // Sort the array
    std::sort(timestamp_array_to_sort.begin(), timestamp_array_to_sort.end());

    // Check if the range between the smallest and largest timestamps is within
    // the threshold
    auto min_stamp = timestamp_array_to_sort.front();
    auto max_stamp = timestamp_array_to_sort.back();

    bool within_threshold = (max_stamp - min_stamp) <= threshold;

    auto curr_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch());

    constexpr std::chrono::seconds debug_print_period(1);

    bool all_members_received =
        min_stamp.count() > 0;  // count here is the count in microseconds
    bool last_update_recent_enough =
        (std::chrono::duration_cast<std::chrono::microseconds>(
            curr_time - max_stamp)) < threshold;

    return within_threshold && all_members_received &&
           last_update_recent_enough;
}
