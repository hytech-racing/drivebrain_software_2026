#pragma once
#include <array>
#include <vector>
#include <functional>
#include <variant>

// from drivebrain_core
#include <Configurable.hpp>
#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <DriverBus.hpp>

#include <google/protobuf/message.h>


// TODO:
// - [ ] look into renaming if using the same class for foxglove server for live data too
// - [ ] write tests for type conversion between foxgloe and drivebrain types and the conditions that they need to handle
// - [ ] write tests for properly handling namespaced cache of parameters

namespace core
{
    class FoxgloveWSServer
    {
    public:
        FoxgloveWSServer() = delete;

        FoxgloveWSServer(std::vector<std::weak_ptr<core::common::Configurable>> configurable_components);
        void send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg);
        ~FoxgloveWSServer()
        {
            spdlog::info("stopping foxglove server");
            _server->stop();
            spdlog::info("stopped foxglove server and destructed foxglove server");
        }

    private:
        /// @brief gets a vector of foxglove parameters from all components, parameter name is name-spaced to each component
        /// @return vector of foxglove params
        std::vector<foxglove::Parameter> _get_current_params();
        
        /// @brief gets a converted to foxglove parameter from a drivebrain parameter type
        /// @param set_name the final name to set set the foxglove parameter to
        /// @param param the drivebrain parameter
        /// @return foxglove parameter type
        foxglove::Parameter _get_foxglove_param(const std::string& set_name, core::common::Configurable::ParamTypes param);
        
        /// @brief gets a drivebrain parameter from a foxglove parameter
        /// @param param the parameter to convert
        /// @return drivebrain param
        core::common::Configurable::ParamTypes _get_db_param(foxglove::Parameter param);

        /// @brief function to handle execution of the parameter updating internally for a single foxglove parameter
        /// @param param_update the new parameter value that has been properly converted to the expected type that is currently in the parameter cache
        void _set_db_param(foxglove::Parameter param_update);
        
        /// @brief handles conversion of input foxglove parameter as the foxglove parameters incoming from the server arent guaranteed to stay the same type (float -> int, int-> float, etc.)
        /// @param curr_param_val the internal parameter value of the drivebrain type
        /// @param incoming_param the new foxglove parameter value that may need conversion
        /// @return optional foxglove parameter, if the type is un-supported for conversion this is a nullopt and the parameter update should not occur
        std::optional<foxglove::Parameter> _convert_foxglove_param(core::common::Configurable::ParamTypes curr_param_val, foxglove::Parameter incoming_param);
        void _handle_foxglove_send();
    private:
        std::vector<std::weak_ptr<core::common::Configurable>> _components;
        std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
        std::function<void(foxglove::WebSocketLogLevel, char const *)> _log_handler;
        foxglove::ServerOptions _server_options;
        std::unordered_map<std::string, foxglove::ChannelId> _id_name_map;

    };
}