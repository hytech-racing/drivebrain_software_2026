#pragma once

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <boost/signals2.hpp>
#include <vector> 
#include <string> 
#include <fstream> 
#include <thread>
#include <atomic>
#include <cassert>

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
             * Destructs the foxglove server instance by stopping the server. 
             */
            ~FoxgloveServer() {
                _server->stop(); 
                std::cout << "Destructed and stopped foxglove websocket server" << std::endl; 
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
             // TODO: investigate the type conversion conflicts that arrise from this (i.e. int64_t vs plain int)
            template <typename param_type> 
            param_type get_param(std::string param_name) {
                std::unique_lock lock(_parameter_mutex); 

                // if (_foxglove_params_map.find(param_name) == _foxglove_params_map.end()) {
                //     return NULL;
                // }
                
                return _foxglove_params_map.at(param_name).getValue<param_type>();
            }

        private: 
            FoxgloveServer(std::string parameters_file);

            /* Singleton move semantics */
            FoxgloveServer(const FoxgloveServer&) = delete;
            FoxgloveServer& operator=(const FoxgloveServer&) = delete;

            /* Singleton instance */
            inline static std::atomic<FoxgloveServer*> _s_instance;

            /* Boost signal for parameter updates */
            boost::signals2::signal<void(const std::unordered_map<std::string, foxglove::ParameterValue>&)> _param_update_signal;
            
            std::unordered_map<std::string, foxglove::ParameterValue> _foxglove_params_map; 
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            
            std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
            foxglove::ServerOptions _server_options;

            std::mutex _parameter_mutex; 
    };
}

