#ifndef ETHERNET_RECEIVE_COMMS_H
#define ETHERNET_RECEIVE_COMMS_H

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <vector> 
#include <string> 
#include <fstream> 
#include <thread>

#include "hytech_msgs.pb.h"

namespace core {
    class FoxgloveServer {
        public: 
            /**
             * Instantiates the foxglove server instance
             * 
             * @param parameters_file the json containing parameters for putting on foxglove
             */
            FoxgloveServer(std::string parameters_file);

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
             * Thread-safe method to get a foxglove param
             * 
             * @param param_name name of the parameter the user wants to get
             * @return the parameter value
             */
            template <typename param_type> 
            param_type get_param(std::string param_name) {
                std::unique_lock lock(_parameter_mutex); 

                // if (_foxglove_params_map.find(param_name) == _foxglove_params_map.end()) {
                //     return NULL;
                // }
                
                return _foxglove_params_map.at(param_name).getValue<param_type>();
            }
            

        private: 
            std::unordered_map<std::string, foxglove::ParameterValue> _foxglove_params_map; 
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            
            
            std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
            foxglove::ServerOptions _server_options;

            std::mutex _parameter_mutex; 
    };
}

#endif // ETHERNET_RECEIVE_COMMS_H