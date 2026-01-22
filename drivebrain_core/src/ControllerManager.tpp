#include "ControllerManager.hpp"

template <typename ControllerType, size_t NumControllers>
bool control::ControllerManager<ControllerType, NumControllers>::init()
{

    //todo: update the config json with the real values.
    //todo: implement configurable loading from config.json

    ControllerManagerParams params;
    //todo: implement register_all
    params.register_all(this); 

    if(params.max_accel_switch_float > 1.0){
        std::cout << "ERROR: max accel switch float > 1.0" << std::endl;
        return false;
    }
    //assign params to member variables
    _max_switch_rpm = params.max_controller_switch_speed_ms.val * constants::METERS_PER_SECOND_TO_RPM;
    _max_torque_switch = params.max_torque_switch_nm.val;
    _max_accel_switch_req = params.max_accel_switch_float.val;
    _max_requested_rpm = params.max_requested_rpm.val;

    // Initialize state
    _current_ctr_manager_state = {
        .current_status = core::control::ControllerManagerStatus::NO_ERROR,
        .current_controller_output = std::monostate{}
    };
    
    return true;

}


template<typename ControllerType, size_t NumControllers>
core::control::ControllerManagerStatus control::ControllerManager<ControllerType, NumControllers>::_can_switch_controller(const core::VehicleState &current_state,
                                                                                                                       const core::ControllerOutput &previous_output,
                                                                                                                       const core::ControllerOutput &next_controller_output)
{

    
    
}


template <typename ControllerType, size_t NumControllers>
bool control::ControllerManager<ControllerType, NumControllers>::swap_active_controller(size_t new_controller_index, const core::VehicleState& input)
{
    
}

