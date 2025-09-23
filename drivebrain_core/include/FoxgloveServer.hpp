#ifndef ETHERNET_RECEIVE_COMMS_H
#define ETHERNET_RECEIVE_COMMS_H

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include "hytech_msgs.pb.h"

#include <vector> 
#include <string> 
#include <fstream> 

namespace core {
    class FoxgloveServer {
        public: 
            FoxgloveServer(std::string parameters_file, std::vector<std::function<void(std::unordered_map<std::string, foxglove::ParameterValue>)>> parameter_change_callbacks);
            ~FoxgloveServer() {
                _server->stop(); 
                std::cout << "Destructed and stopped foxglove ws server" << std::endl; 
            }

            void send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg);

        private: 
            std::unordered_map<std::string, foxglove::ParameterValue> _foxglove_params_map; 
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
            foxglove::ServerOptions _server_options; 
            std::vector<std::function<void(std::unordered_map<std::string, foxglove::ParameterValue>)>> _parameter_change_callbacks;
    };
}

#endif // ETHERNET_RECEIVE_COMMS_H