#include "ETHSendComms.hpp"
#include "ETHRecvComms.hpp"
#include <boost/asio/io_context.hpp>
#include <chrono>
#include <csignal>
#include <exception>
#include <google/protobuf/message.h>
#include <iostream>
#include <atomic> 
#include <hytech_msgs.pb.h>
#include <hytech.pb.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <mcap/writer.hpp>
#include <CANComms.hpp> 
#include <memory>
#include <thread>
#include <optional>
#include <filesystem>

std::atomic<bool> running = true;
std::optional<std::string> json_file; 
std::optional<std::string> dbc_file;

int parse_arguments(int &argc, char* argv[]) {
    int opt;

    while ((opt = getopt(argc, argv, "c:d:")) != -1) {
        switch (opt) {
            case 'c': 
                if (optarg != nullptr && strlen(optarg) > 0 && std::filesystem::exists(optarg)) {
                    json_file = optarg;
                } else {
                    spdlog::error("Invalid json file provided: {}", optarg);
                    return 1;
                }
                break;
            case 'd': 
                if (optarg != nullptr && strlen(optarg) > 0 && std::filesystem::exists(optarg)) {
                    dbc_file = optarg;
                } else {
                    spdlog::error("Invalid dbc file provided: {}", optarg);
                    return 1;
                }
                break;
            case '?':
                spdlog::error("Unknown command line option: -{}", static_cast<char>(optopt));
                return 1;
            default: 
                spdlog::error("Could not parse command line arguments.");
                return 1;
        }
    }

    if (!json_file) {
        spdlog::error("Did not receive the required json file argument.");
        return 1;
    }

    if (!dbc_file) {
        spdlog::error("Did not receive the required dbc file argument.");
        return 1;
    }

    return 0;
}

void sig_handler(int signal) {
    if(signal == SIGINT) {
        std::cout << "halting\n";
        running = false;
    }
}

int main(int argc, char* argv[]) {
    // Argument Handling
    int return_code = parse_arguments(argc, argv);
    if (return_code != 0) {
        spdlog::error("Expected Usage: ./drivebrain -c path/to/config.json -d path/to/hytech.dbc");
        return return_code;
    }

    comms::CANComms primary_can("can0", dbc_file.value());

    // Singleton Creation
    core::FoxgloveServer::create(json_file.value());
    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), json_file.value());
    core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
    core::MCAPLogger::instance().init_logging();

    std::signal(SIGINT, sig_handler);
    while(running) {
        std::shared_ptr<hytech::drivebrain_speed_set_input> speed_msg = std::make_shared<hytech::drivebrain_speed_set_input>(); 
        speed_msg->set_drivebrain_set_rpm_fl(1.0);
        speed_msg->set_drivebrain_set_rpm_fr(2.0);
        speed_msg->set_drivebrain_set_rpm_rl(4.0);
        speed_msg->set_drivebrain_set_rpm_rr(8.0);
        primary_can.send_message(speed_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    
    core::MCAPLogger::instance().close_active_mcap();
    core::MCAPLogger::destroy();
    core::FoxgloveServer::destroy();
}
