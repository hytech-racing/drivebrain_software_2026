#pragma once

#include <exception>
#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <boost/signals2/connection.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <boost/signals2.hpp>
#include <MCAPLogger.hpp>
#include <vector> 
#include <string> 
#include <fstream> 
#include <thread>
#include <atomic>
#include <cassert>
#include <optional>
#include <spdlog/spdlog.h>

#include "hytech_msgs.pb.h"

namespace core {
    class FoxgloveServer {
        public: 
            /**
             *  Initialzes a new Foxglove server singleton instance 
             * 
             * @param parameters_file the json containing parameters for putting on foxglove
             */
            static void create(const std::string &parameters_file);

            /**
             * Fetches Foxglove server singleton instance
             *
             * @retun FoxgloveServer instance
             */
            static FoxgloveServer& instance();

            /**
             * Destroys the Foxglove server singleton instance, stopping the server and freeing resources
             */
            static void destroy();
            
            /**
             * Destructs the foxglove server instance by stopping the server. 
             */
            ~FoxgloveServer() {
                _server->stop(); 
                spdlog::info("Destructed and stopped foxglove websocket server"); 
            }
            
            /**
             * Sends a protobuf to be viewed in foxglove. 
             * broadcastMessage() is thread safe so this method can be called
             * by different threads without a mutex.
             * 
             * @param msg the message to be sent
             */
            void send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg);

            /**
             * Registers a callback function to be run whenever a parameter is updated in Foxglove.
             * @param The function to be reigstered, which takes the form of (const string, param) -> void
             * @return The newly created boost connection
            */
            boost::signals2::connection register_param_callback(std::function<void(const std::unordered_map<std::string, foxglove::ParameterValue>&)> callback);

            /**
             * Returns all the current parameter values in a JSON format. Mostly used to log the current parameters in an MCAP file
            */
            nlohmann::json get_all_params();

            /**
             * Thread-safe method to get a foxglove param
             * 
             * @param param_name name of the parameter the user wants to get
             * @return the parameter value
             */
            template <typename param_type> 
            std::optional<param_type> get_param(std::string param_name) {
                
                std::unique_lock lock(_parameter_mutex); 
                std::transform(param_name.begin(), param_name.end(), param_name.begin(),
                    [](unsigned char c){ return static_cast<unsigned char>(std::tolower(c)); });

                if (_foxglove_params_map.find(param_name) == _foxglove_params_map.end()) {
                    spdlog::warn("The following parameter was not found in the params json: " + param_name);
                    return std::nullopt;
                }

                try {
                    return _foxglove_params_map[param_name].getValue<param_type>();
                } catch (const std::exception& e) {
                    spdlog::warn("Incorrect parameter type for param {}: {}", param_name, e.what());
                    return std::nullopt;
                }
            }

        private: 
            FoxgloveServer(std::string parameters_file);

            /* Registers JSON params on init. Recursively called to support multi-level JSON */
            void _init_params(const nlohmann::json &json_obj, const std::string &prefix);

            /* Singleton move semantics */
            FoxgloveServer(const FoxgloveServer&) = delete;
            FoxgloveServer& operator=(const FoxgloveServer&) = delete;

            /* Singleton instance */
            inline static std::atomic<FoxgloveServer*> _s_instance;

            /* Boost signal for parameter updates */
            boost::signals2::signal<void(const std::unordered_map<std::string, foxglove::ParameterValue>&)> _param_update_signal;

            /* Method to handle converting foxglove params to avoid type conflicts */
            std::optional<foxglove::Parameter> _convert_foxglove_parameter(foxglove::Parameter current_param, foxglove::Parameter incoming_param);
            
            std::unordered_map<std::string, foxglove::ParameterValue> _foxglove_params_map; 
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            
            std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
            foxglove::ServerOptions _server_options;

            std::mutex _parameter_mutex; 
    };
}

