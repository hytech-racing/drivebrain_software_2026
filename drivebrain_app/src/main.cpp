#include "EthernetComms.hpp"
#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
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

std::atomic<bool> running = true;
boost::asio::io_context io_context;
comms::ETHDriver<> eth_sender{io_context, 2222, "127.0.0.1"};
comms::ETHDriver<hytech_msgs::ACUAllData> eth_recver{io_context, 2222};

void sig_handler(int signal) {
    if(signal == SIGINT) {
        std::cout << "halting\n";
        running = false;
    }
}

void eth_send_msg() {
    while(running) {
        auto msg = std::make_shared<hytech_msgs::ACUAllData>();
        msg->set_soc(67.0);
        std::this_thread::sleep_for((std::chrono::seconds(1)));
        eth_sender.enqueue_msg_send(msg);
    }
}

int main(int argc, char* argv[]) {
    core::FoxgloveServer::create(argv[1]);
    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), argv[1]);
    core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
    core::MCAPLogger::instance().init_logging();

    std::signal(SIGINT, sig_handler);
    
    // auto vel_msg = std::make_shared<hytech::velocities>();
    // vel_msg->set_velocity_x(1000);
    // vel_msg->set_velocity_y(10000);

    // auto acu_data = std::make_shared<hytech_msgs::ACUAllData>();
    // acu_data->set_max_cell_temp_id(676767);

    auto work_guard = boost::asio::make_work_guard(io_context);
    std::thread io_thread([&]() {
        io_context.run();
    });

    std::thread t1(eth_send_msg);

    if(t1.joinable()) t1.join();
    work_guard.reset();
    io_context.stop();
    if(io_thread.joinable()) io_thread.join();
    core::MCAPLogger::instance().close_active_mcap();
}
