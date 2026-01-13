#include <CANComms.hpp> 
#include <hytech.pb.h>

void test_send() {
    
}

void test_receive() {

}

int main() {

    // Initialize can driver (second command line argument is the dbc file path)
    comms::CANComms primary_can("can_primary", argv[2]);



    return 0; 
}