#include <CANComms.hpp>

/****************************************************************
 * HELPER METHODS
 ****************************************************************/
std::string string_to_lowercase(std::string input_string) {
    std::transform(input_string.begin(), input_string.end(), input_string.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    return input_string;
}

/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
comms::CANComms::CANComms(const std::string &device_name, const std::string &dbc_file_path) {
    _init(device_name, dbc_file_path); 
}

void comms::CANComms::send_message(std::shared_ptr<google::protobuf::Message> message) {

    can_frame frame{};
    int return_code = _encode_can_frame(message, &frame);
    if (return_code < 0) {
        std::cout << "Failed to create CAN message from protobuf" << std::endl;
        return;
    }

    int nbytes = write(_socket, &frame, sizeof(struct can_frame));
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
int comms::CANComms::_init(const std::string &device_name, const std::string &dbc_file_path) {
    // Initialize the CAN socket
    struct sockaddr_can addr;
    struct ifreq ifr;

    _socket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW); // TODO Make sure this works

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

        std::shared_ptr<google::protobuf::Message> dmsg = _decode_can_frame(_frame);
    }
}

std::shared_ptr<google::protobuf::Message> comms::CANComms::_get_proto_message_from_name(const std::string &name) {
    std::shared_ptr<google::protobuf::Message> proto_message; 
    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName("hytech." + name);
    if (!desc) {
        return nullptr;
    }
    proto_message.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());
    if (!proto_message) {
        return nullptr;
    }
    return proto_message;
}


std::shared_ptr<google::protobuf::Message> comms::CANComms::_decode_can_frame(struct can_frame &frame) {
    const dbcppp::IMessage* dbc_msg = _messages[frame.can_id];
    
    if (dbc_msg == nullptr) {
        return nullptr; 
    }

    std::shared_ptr<google::protobuf::Message> proto_message = _get_proto_message_from_name(string_to_lowercase(dbc_msg->Name()));
    if (proto_message == nullptr) {
        return nullptr;
    }

    const google::protobuf::Descriptor *descriptor = proto_message->GetDescriptor();
    const google::protobuf::Reflection *reflection = proto_message->GetReflection(); 

    for (const dbcppp::ISignal &sig : dbc_msg->Signals()) {
        const dbcppp::ISignal *mux_sig = dbc_msg->MuxSignal();

        if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue || (mux_sig && mux_sig->Decode(frame.data) == sig.MultiplexerSwitchValue())) {
            dbcppp::ISignal::raw_t raw_value = sig.Decode(frame.data);
            double value = sig.RawToPhys(raw_value);

            const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(sig.Name());
            switch (field->type()) {
                case google::protobuf::FieldDescriptor::TYPE_ENUM: 
                    reflection->SetEnumValue(proto_message.get(), field, (int)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_FLOAT:
                    reflection->SetFloat(proto_message.get(), field, (float)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_BOOL: 
                    reflection->SetBool(proto_message.get(), field, (bool)value);
                    break;
                case google::protobuf::FieldDescriptor::TYPE_INT32: 
                    reflection->SetInt32(proto_message.get(), field, (int32_t)value);
                    break;
                default: 
                    break;
            }
        }
    }

    return proto_message; 

}

int comms::CANComms::_encode_can_frame(std::shared_ptr<google::protobuf::Message> proto_message, can_frame* frame) {

    std::string type_url = proto_message->GetTypeName();
    std::string messageTypeName = type_url.substr(type_url.find_last_of('.') + 1); // TODO validate this is not scuffed

    const google::protobuf::Descriptor *descriptor = proto_message->GetDescriptor();
    const google::protobuf::Reflection *reflection = proto_message->GetReflection(); 

    if (_names_to_can_id.find(messageTypeName) == _names_to_can_id.end()) {
        return -EINVAL;
    }
        
    uint64_t id = _names_to_can_id[messageTypeName]; 
    auto dbc_message = _messages[id]->Clone();   
    frame->can_id = id;
    frame->len = dbc_message->MessageSize();

    for (const auto &sig : dbc_message->Signals()) {
        const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(sig.Name());

        switch(field->cpp_type()) {
            case google::protobuf::FieldDescriptor::CPPTYPE_INT32: {
                int32_t value = reflection->GetInt32(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_INT64: {
                int64_t value = reflection->GetInt64(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: {
                uint32_t value = reflection->GetUInt32(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: {
                uint64_t value = reflection->GetUInt64(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
                double value = reflection->GetDouble(*proto_message, field); 
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
                double value = reflection->GetFloat(*proto_message, field);
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
                bool value = reflection->GetBool(*proto_message, field);
                sig.Encode(sig.PhysToRaw(value), frame->data);
                break;
            } default: 
                break;       
        }        
    }

    return 0;

}