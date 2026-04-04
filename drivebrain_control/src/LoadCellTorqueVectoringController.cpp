#include "LoadCellVectoringTorqueController.hpp"
#include "SimplePowerLimiter.hpp"

#include <variant>

#include <VehicleDataTypes.hpp>
#include <JSONUtils.hpp>

#include <spdlog/spdlog.h>

void control::LoadCellVectoringTorqueController::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map) {
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(max_torque, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(max_regen_torque, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(rear_torque_scale, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(regen_torque_scale, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(positive_speed_set, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(max_power_kw, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(dt_rate_hz, int, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(apply_vectoring_in_regen, bool, _config, _config_mutex, new_param_map);
}

bool control::LoadCellVectoringTorqueController::init()
{
    LOAD_LIVE_PARAM_OR_FAIL(max_torque, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(max_regen_torque, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(rear_torque_scale, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(regen_torque_scale, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(positive_speed_set, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(max_power_kw, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(dt_rate_hz, int, _config);
    LOAD_LIVE_PARAM_OR_FAIL(apply_vectoring_in_regen, bool, _config);
    param_update_handler_sig.connect(boost::bind(&control::LoadCellVectoringTorqueController::_handle_param_updates, this, std::placeholders::_1));
    
    set_configured();
    return true;
}

core::ControllerOutput control::LoadCellVectoringTorqueController::step_controller(const core::VehicleState &in)
{
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }

    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = (in.input.requested_accel) - (in.input.requested_brake);

    veh_vec<float> current_rpms = in.current_rpms;

    torque_nm torqueRequest = {};

    core::SpeedControlOut type_set = {};
    core::ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<core::SpeedControlOut>(cmd_out.out);
    speed_out = {};
    speed_out.mcu_recv_millis = in.prev_MCU_recv_millis; // heartbeat TODO this isnt needed any more, prob should remove
    
    
    float sum_normal = in.loadcells.FL + in.loadcells.FR + in.loadcells.RL+ in.loadcells.RR;
    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = ((float)accelRequest) * cur_config.max_torque;
        
        float accel_torque_pool = accelRequest * cur_config.max_torque * 4; 
        
        
        auto max_rpm = cur_config.positive_speed_set * constants::METERS_PER_SECOND_TO_RPM;
        speed_out.desired_rpms.FL = max_rpm;
        speed_out.desired_rpms.FR = max_rpm;
        speed_out.desired_rpms.RL = max_rpm;
        speed_out.desired_rpms.RR = max_rpm;

        speed_out.torque_lim_nm.FL = ((2.0 - cur_config.rear_torque_scale) * accel_torque_pool * (in.loadcells.FL / sum_normal) );
        speed_out.torque_lim_nm.FR = ((2.0 - cur_config.rear_torque_scale) * accel_torque_pool * (in.loadcells.FR / sum_normal) );
        speed_out.torque_lim_nm.RL = (cur_config.rear_torque_scale * accel_torque_pool * (in.loadcells.RL / sum_normal) );
        speed_out.torque_lim_nm.RR = (cur_config.rear_torque_scale * accel_torque_pool * (in.loadcells.RR / sum_normal) );
        cmd_out.out = control::util::apply_power_limit(speed_out, in.current_rpms, cur_config.max_power_kw);
    }
    else
    {
        // Negative torque request
        
        float regen_torque_pool = accelRequest * cur_config.max_regen_torque * 4 * -1.0; 
        
        speed_out.desired_rpms.FL = 0;
        speed_out.desired_rpms.FR = 0;
        speed_out.desired_rpms.RL = 0;
        speed_out.desired_rpms.RR = 0;
        
        if(cur_config.apply_vectoring_in_regen)
        {
            speed_out.torque_lim_nm.FL = (regen_torque_pool * (in.loadcells.FL / sum_normal) * (2.0 - cur_config.rear_torque_scale));
            speed_out.torque_lim_nm.FR = (regen_torque_pool * (in.loadcells.FR / sum_normal) * (2.0 - cur_config.rear_torque_scale));
            speed_out.torque_lim_nm.RL = (regen_torque_pool * (in.loadcells.RL / sum_normal) * cur_config.rear_torque_scale);
            speed_out.torque_lim_nm.RR = (regen_torque_pool * (in.loadcells.RR / sum_normal) * cur_config.rear_torque_scale);
            cmd_out.out = speed_out; // no need to apply power limit for regen request
        } else {
            float reg_torq = -1.0 * accelRequest * cur_config.max_regen_torque;
            speed_out.torque_lim_nm.FL = ( reg_torq * ( 2.0 - cur_config.rear_torque_scale) );
            speed_out.torque_lim_nm.FR = ( reg_torq * ( 2.0 - cur_config.rear_torque_scale) );
            speed_out.torque_lim_nm.RL = ( reg_torq * cur_config.rear_torque_scale);
            speed_out.torque_lim_nm.RR = ( reg_torq * cur_config.rear_torque_scale);
            cmd_out.out = speed_out; // no need to apply power limit for regen request
        }
        
    }

    return cmd_out;
}