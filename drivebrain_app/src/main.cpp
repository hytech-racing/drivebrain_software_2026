#include "EthernetComms.hpp"
#include <boost/asio/io_context.hpp>
#include <chrono>
#include <csignal>
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
comms::ETHDriver<> eth_sender{io_context, 1155, "127.0.0.1"};
comms::ETHDriver<hytech_msgs::ACUAllData> eth_recver{io_context, 1155};

void sig_handler(int signal) {
    if(signal == SIGINT) {
        std::cout << "halting\n";
        running = false;
    }
}

void get_param_task(int wait_time, core::MsgType msg) {
    while(running) {
        core::MCAPLogger::instance().log_msg(static_cast<core::MsgType>(msg));
        std::this_thread::sleep_for((std::chrono::milliseconds(wait_time)));
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

// void eth_recv_msg() {
//     while(running) {
//         eth_recver
//     }
// }

int main(int argc, char* argv[]) {
    

    core::FoxgloveServer::create(argv[1]);
    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""));
    core::MCAPLogger::instance().open_new_mcap("test_1.mcap");
    core::MCAPLogger::instance().init_logging();

    std::signal(SIGINT, sig_handler);
    
    // auto vel_msg = std::make_shared<hytech::velocities>();
    // vel_msg->set_velocity_x(1000);
    // vel_msg->set_velocity_y(10000);

    // auto acu_data = std::make_shared<hytech_msgs::ACUAllData>();
    // acu_data->set_max_cell_temp_id(676767);

    std::thread t1(eth_send_msg);

    if(t1.joinable()) t1.join();
    core::MCAPLogger::instance().close_active_mcap();

}
