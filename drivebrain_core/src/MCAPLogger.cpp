#include <MCAPLogger.hpp>

/****************************************************************
 * HELPER METHODS
 ****************************************************************/
static std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames) {
    std::vector<const google::protobuf::FileDescriptor *> descriptors;

    for (const auto &name : filenames) {
        const google::protobuf::FileDescriptor *file_descriptor = google::protobuf::DescriptorPool::generated_pool()->FindFileByName(name);

        if (!file_descriptor) {
            std::cout << "File descriptor not found" << std::endl;
        }
        else {
            descriptors.push_back(file_descriptor);
        }
    }
    return descriptors;
}

static std::string serialize_fd_set(const google::protobuf::Descriptor *toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor *> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while (!toAdd.empty())
    {
        const google::protobuf::FileDescriptor *next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for (int i = 0; i < next->dependency_count(); ++i)
        {
            const auto &dep = next->dependency(i);
            if (seenDependencies.find(dep->name()) == seenDependencies.end())
            {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}
 
/****************************************************************
 * PUBLIC CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
core::MCAPLogger::MCAPLogger(const std::string &base_dir, const mcap::McapWriterOptions &options) : _options(options) {
}

int core::MCAPLogger::open_new_mcap(const std::string &name) {
    std::cout << "Attempting to open new MCAP file" << std::endl;

    const auto res = _writer.open(name, _options);
    if (!res.ok()) {
        std::cout << "Failed to open MCAP :(" << std::endl;
        return -1;
    }

    std::vector<std::string> proto_names = {"hytech_msgs.proto", "hytech.proto"};
    auto descriptors = get_pb_descriptors(proto_names);

    for (const auto &file_descriptor : descriptors) {
        for (int i = 1; i <= file_descriptor->message_type_count(); i++) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            _name_to_id_map[message_descriptor->name()] = i;
            mcap::Schema schema(message_descriptor->full_name(), "protobuf", foxglove::base64Encode(serialize_fd_set(message_descriptor)));
            _writer.addSchema(schema);
            mcap::Channel channel(message_descriptor->name(), "protobuf", schema.id);
            _writer.addChannel(channel);
        }
    }

    std::cout << "Successfully opened and added message descriptions to mcap" << std::endl;
    return 0;
}

int core::MCAPLogger::close_active_mcap() {
    std::cout << "Closing mcap" << std::endl;
    _writer.close(); 
}

int core::MCAPLogger::log_protobuf_message(std::shared_ptr<google::protobuf::Message> message) {
    mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    RawMessage_s new_message; 
    new_message.log_time = log_time;
    new_message.serialized_data = message->SerializeAsString(); 
    new_message.message_name = message->GetDescriptor()->name();
    {
        std::unique_lock lock(_input_buffer_mutex);
        _input_buffer.push_back(new_message); 
        _input_buffer_cv.notify_one(); 
    }

    return 0;
}

/****************************************************************
 * PRIVATE CLASS METHOD IMPLEMENTATIONS
 ****************************************************************/
void core::MCAPLogger::_handle_log_to_file() {
    static std::deque<RawMessage_s> write_buffer; // The buffer to be copied over to
    
    while (true) {
        {
            std::unique_lock lock(_input_buffer_mutex);
            _input_buffer_cv.wait(lock, [this](){ return !_input_buffer.empty() || !_running;});
            if (_running && _input_buffer.empty()) {
                break;
            }

            write_buffer = std::move(_input_buffer);
        }

        for (auto &msg : write_buffer) {
            mcap::Message msg_to_log;
            msg_to_log.data = reinterpret_cast<const std::byte *>(msg.serialized_data.data());
            msg_to_log.dataSize = msg.serialized_data.size();

            msg_to_log.logTime = msg.log_time;
            msg_to_log.publishTime = msg.log_time;
    
            msg_to_log.channelId = _name_to_id_map[msg.message_name];
            auto write_res = _writer.write(msg_to_log);

        }
    }
}   


