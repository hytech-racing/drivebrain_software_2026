#include <CANComms.hpp>


/****************************************************************
 * HELPER METHODS
 ****************************************************************/
void string_to_lowercase(std::string input_string) {
    std::transform(input_string.begin(), input_string.end(), input_string.begin(),
                   [](unsigned char c){ return std::tolower(c); });
}

std::shared_ptr<google::protobuf::Message> interpret_can_frame(struct can_frame& frame) {
    auto iter = _messages.find(frame.can_id);
    if (iter != _messages.end()) {
        const dbcppp::IMessage* msg = iter->second;
    } else {
        // TODO handle conversion to protobuf
        return nullptr;
    }
}

can_frame protobuf_to_can_frame(std::shared_ptr<google::protobuf::Message> message) {
    dbcppp::ISignal
}

/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
comms::CANComms::CANComms(const std::string &device_name, const std::string &dbc_file_path) {
    _init(device_name, dbc_file_path); 
}

int comms::CANComms::send_message(std::shared_ptr<google::protobuf::Message> message) {
    return 0;
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
int comms::CANComms::_init(const std::string &device_name, const std::string &dbc_file_path) {
    // Initialize the CAN socket
    struct sockaddr_can addr;
    struct ifreq ifr;

    _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, device_name.c_str());
    ioctl(_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(_socket, (struct sockaddr *)&addr, sizeof(addr));

    // Set up DBC parsing
    {
        std::ifstream idbc(dbc_file_path.c_str());
        _net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    for (const dbcppp::IMessage& msg : _net->Messages()) {
        _messages.insert(std::make_pair(msg.Id(), &msg));
        _names_to_can_id.insert(std::make_pair(string_to_lowercase(msg.Name()), msg.Id()));
    }

    // Spawn reader thread
    _reader_thread = std::thread(&comms::CANComms::_receive_handler, this);

    return 0;
}

void comms::CANComms::_receive_handler() {
    int nbytes;

    while(true) {
        nbytes = read(_socket, &_frame, sizeof(can_frame));

        if (nbytes < 0) {
                std::cout << "Error reading from can socket." << std::endl;
        }

        if (nbytes < sizeof(struct can_frame)) {
                std::cout << "Error reading from can socket. Incomplete CAN frame." << std::endl;
        }
    }
}