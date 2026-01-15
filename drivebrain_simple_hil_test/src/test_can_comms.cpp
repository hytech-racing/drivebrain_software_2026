#include <CANComms.hpp> 
#include <hytech.pb.h>
#include <thread> 

void thread1_send() {
    
}

void thread2_send() {

}

int main() {

    // Initialize can driver (second command line argument is the dbc file path)
    comms::CANComms primary_can("can_primary", argv[2]);

    std::thread thread1(thread1_send);
    std::thread thread2(thread2_send);

    // TODO

    thread1.join(); 
    thread2.join(); 

    return 0; 
}