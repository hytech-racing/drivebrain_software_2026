#pragma once
//copied over from old drivebrain core

namespace control
{
    template <typename ControllerResult, typename ControllerInput>
    class Controller {
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