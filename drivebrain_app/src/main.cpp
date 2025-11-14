#include <iostream>
#include <atomic> 
#include <hytech_msgs.pb.h>
#include <hytech.pb.h>
#include <FoxgloveServer.hpp>

std::atomic<bool> running = true;

int main(int argc, char* argv[]) {
    
    hytech::inv1_status status;
    hytech_msgs::ACUAllData data;

    core::FoxgloveServer server(argv[1]);

    while (running) {

        std::cout << server.get_param<int>("sdfsdfsdf") << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}