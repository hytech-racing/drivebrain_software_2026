#pragma once
//copied over from old drivebrain core


// STORY:

// handling inputs:
// I want to be able to have both "reactionary" or "sample-based" modes of communication between the input and the
    // reactionary: upon arrival of the data the controller executes ~~
        // this probably wont be a thing for us to do, NOTODO
    // sample-based: fixed rate per-each controller

// we need to have a core piece of the controller be that the inputs must be coming in at a reasonable time period
    // as a part of the config for each controller we need to have a maximum input period that can ellapse before
    // controller disables output

// we want to be able to have controllers that require the drivetrain to be in a certain state / control mode
// we want to be able to output different types of output from each controller.
// the controller should have access to the parameter server and be "Configurable"
namespace control
{
    template <typename ControllerResult, typename ControllerInput>
    class Controller
    {
    public:
        Controller() = default;
        /// @brief get the desired time delta / discrete time step length in seconds of the controller
        /// @return time delta in seconds
        virtual float get_dt_sec() = 0;

        /// @brief step the controller
        /// @param in the input of the controller
        /// @return the result of the controller
        virtual ControllerResult step_controller(const ControllerInput& in) = 0;
    };
}