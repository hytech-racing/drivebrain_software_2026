#include <iostream>
#include <atomic> 
#include <hytech_msgs.pb.h>
#include <FoxgloveServer.hpp>

std::atomic<bool> running = true;

void test_m(std::unordered_map<std::string, foxglove::ParameterValue> parameter_map) {
    std::cout << "params changed" << std::endl;
}

int main(int argc, char* argv[]) {
    std::vector<std::function<void(std::unordered_map<std::string, foxglove::ParameterValue>)>> parameter_change_callbacks;
    parameter_change_callbacks.push_back(test_m);
    core::FoxgloveServer foxglove_server(argv[1], parameter_change_callbacks);


    float i = 0;
    hytech_msgs::ACUAllData test;
    while (running) {

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}