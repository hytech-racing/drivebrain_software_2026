// Communication Driver for USB Serial Aero Pressure Sensors
// Surrey Sensors mus8
#pragma once

// protobuf
#include "FoxgloveServer.hpp"
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>
#include "hytech_msgs.pb.h"

// boost
#include <boost/asio.hpp>
// #include <boost/array.hpp>

using namespace as::xplat;
using namespace as::protocol::uart;
using SerialPort = boost::asio::serial_port;


namespace comms {
    class AeroSenseDriver
    {
        public:
            AeroSenseDriver(boost::asio::io_context &io_context); 
            ~AeroSenseDriver() {
                spdlog::info("destructed %s");
            }
            bool init();
            struct config {
                std::string device_name;
                int baud_rate;
            };

        private:
        // vn::protocol::uart::PacketFinder _processor;
        boost::array<std::uint8_t, 512> _output_buff;
        boost::array<std::uint8_t, 512> _input_buff;
        SerialPort _serial;
        config _config;

        public:
            void log_proto_message(std::shared_ptr<google::protobuf::Message> msg);
        private:
            void _start_recieve();
            // void _handle_recieve();
            
    }
}