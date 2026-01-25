#include <chrono>
#include <csignal>
#include <cstdint>
#include <google/protobuf/message.h>
#include <iostream>
#include <atomic> 
#include <hytech_msgs.pb.h>
#include <hytech.pb.h>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <mcap/writer.hpp>
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

void get_param_task(int wait_time, core::MsgType msg) {
    while(running) {
        std::optional<int64_t> param_value1 = core::FoxgloveServer::instance().get_param<int64_t>("rpm_limit");
        std::optional<double> param_value = core::FoxgloveServer::instance().get_param<int>("level_1/kI");
        if (param_value) {
            std::cout << param_value.value() << std::endl;
        } 
        // core::MCAPLogger::instance().log_msg(static_cast<core::MsgType>(msg));
        std::this_thread::sleep_for((std::chrono::milliseconds(wait_time)));
    }
}

int main(int argc, char* argv[]) {

    // Argument Handling
    int return_code = parse_arguments(argc, argv);
    if (return_code != 0) {
        return return_code;
    }

    // Singleton Creation
    core::FoxgloveServer::create(json_file.value());
    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), json_file.value());
    core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
    core::MCAPLogger::instance().init_logging();

    std::signal(SIGINT, sig_handler);
    
    auto vel_msg = std::make_shared<hytech::velocities>();
    vel_msg->set_velocity_x(1000);
    vel_msg->set_velocity_y(10000);

    auto acu_data = std::make_shared<hytech_msgs::ACUAllData>();
    acu_data->set_max_cell_temp_id(676767);

    // Spawning main threads
    std::thread t1(get_param_task, 20, vel_msg);
    std::thread t2(get_param_task, 40, acu_data);

    if(t1.joinable()) t1.join();
    if(t2.joinable()) t2.join();
    core::MCAPLogger::instance().close_active_mcap();
}
