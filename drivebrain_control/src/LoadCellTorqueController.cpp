#include "LoadCellTorqueController.hpp"
#include "SimplePowerLimiter.hpp"
#include <variant>
#include <spdlog/spdlog.h>

void control::LoadCellTorqueController::_handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map) {


    if (auto pval = std::get_if<float>(&new_param_map.at("max_torque"))) {
        std::unique_lock lk(_config_mutex);
        _config.max_torque = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("max_regen_torque"))) {
        std::unique_lock lk(_config_mutex);
        _config.max_regen_torque = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("rear_torque_scale"))) {
        std::unique_lock lk(_config_mutex);
        _config.rear_torque_scale = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("regen_torque_scale"))) {
        std::unique_lock lk(_config_mutex);
        _config.regen_torque_scale = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("positive_speed_set"))) {
        std::unique_lock lk(_config_mutex);
        _config.positive_speed_set = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("max_power_kw"))) {
        std::unique_lock lk(_config_mutex);
        _config.max_power_kw = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("dt_rate_hz"))) {
        std::unique_lock lk(_config_mutex);
        _config.dt_rate_hz = *pval;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("apply_vectoring_in_regen"))) {
        std::unique_lock lk(_config_mutex);
        _config.apply_vectoring_in_regen = *pval;
    }

    
}

bool control::LoadCellTorqueController::init()
{
    auto opt_max_torque = FoxgloveServer::instance().get_param<float>("max_torque");
    auto opt_max_regen_torque = FoxgloveServer::instance().get_param<float>("max_regen_torque");
    auto opt_rear_torque_scale = FoxgloveServer::instance().get_param<float>("rear_torque_scale");
    auto opt_regen_torque_scale = FoxgloveServer::instance().get_param<float>("regen_torque_scale");
    auto opt_positive_speed_set = FoxgloveServer::instance().get_param<float>("positive_speed_set");
    auto opt_max_power_kw = FoxgloveServer::instance().get_param<float>("max_power_kw");
    auto opt_dt_rate_hz = FoxgloveServer::instance().get_param<float>("dt_rate_hz");
    auto opt_apply_vectoring_in_regen = FoxgloveServer::instance().get_param<float>("apply_vectoring_in_regen");

    if (!(opt_max_torque && opt_max_regen_torque && opt_rear_torque_scale && opt_regen_torque_scale &&
        opt_positive_speed_set && opt_max_power_kw && opt_dt_rate_hz && opt_apply_vectoring_in_regen)) {
        spdlog::error("Couldn't load all params for loadcell torque vectoring controller.");
        return false;
    }

    _config.max_torque = opt_max_torque.value(); 
    _config.max_regen_torque = opt_max_regen_torque.value(); 
    _config.rear_torque_scale = opt_rear_torque_scale.value(); 
    _config.regen_torque_scale = opt_regen_torque_scale.value(); 
    _config.positive_speed_set = opt_positive_speed_set.value(); 
    _config.max_power_kw = opt_max_power_kw.value(); 
    _config.dt_rate_hz = opt_dt_rate_hz.value(); 
    _config.apply_vectoring_in_regen = opt_apply_vectoring_in_regen.value(); 


    FoxgloveServer::instance().register_param_callback(std::bind(&control::LoadCellTorqueController::_handle_param_updates, this, std::placeholders::_1));
    
    return true;
}

ControllerOutput control::LoadCellTorqueController::step_controller(const VehicleState &in)
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

    SpeedControlOut type_set = {};
    ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<SpeedControlOut>(cmd_out.out);
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
        cmd_out.out = control::apply_power_limit(speed_out, in.current_rpms, cur_config.max_power_kw);
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