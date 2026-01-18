#include <FoxgloveServer.hpp>
#include <boost/signals2/connection.hpp>
#include <foxglove/websocket/parameter.hpp>

static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count());
}


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

static std::string SerializeFdSet(const google::protobuf::Descriptor *toplevelDescriptor) {
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

void core::FoxgloveServer::create(const std::string &parameters_file) {
    FoxgloveServer* expected = nullptr;
    FoxgloveServer* local = new FoxgloveServer(parameters_file);
    if(!_s_instance.compare_exchange_strong(expected, local, std::memory_order_release, std::memory_order_relaxed)) {
        // Already initialized, delete local instance
        delete local;
    }
}

core::FoxgloveServer& core::FoxgloveServer::instance() {
    FoxgloveServer* instance = _s_instance.load(std::memory_order_acquire);
    assert(instance != nullptr && "Foxglove server has not been initialized");
    return *instance;
}

core::FoxgloveServer::FoxgloveServer(std::string file_name) {

    // Read params data
    std::fstream params_file(file_name);
    nlohmann::json init_param_data = nlohmann::json::parse(params_file); 

    for (auto &[key, value]: init_param_data.items()) {
        std::string param_name = key;
        foxglove::ParameterValue param_value; 
        if (value.type() == nlohmann::detail::value_t::boolean) {
            bool raw_value = value.get<bool>(); 
            param_value = foxglove::ParameterValue(raw_value);
        } else if (value.type() == nlohmann::detail::value_t::number_float) {
            float raw_value = value.get<float>(); 
            param_value = foxglove::ParameterValue(raw_value);
        } else if (value.type() == nlohmann::detail::value_t::number_integer || value.type() == nlohmann::detail::value_t::number_unsigned) {
            int64_t raw_value = value.get<int64_t>();
            param_value = foxglove::ParameterValue(raw_value);
        } else {
            std::cerr << "Invalid parameter config type: " << value.type_name() << std::endl;
            return;
        }

        _foxglove_params_map[param_name] = param_value;
        std::cout << "Added parameter: " << param_name << std::endl;
    }

    // Instantiate handlers and create foxglove server 
    const auto logHandler = [](foxglove::WebSocketLogLevel, char const *msg) {
        std::cout << msg << std::endl;
    };

    _server_options.capabilities.push_back("parameters");
    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("Zephyr_Bridge", logHandler, _server_options);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " subscribed to " << chanId << std::endl;
    };

    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " unsubscribed from " << chanId << std::endl;
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter> &params, const std::optional<std::string> &request_id, foxglove::ConnHandle clientHandle) 
    {

        std::unordered_map<std::string, foxglove::ParameterValue> param_copy;
        {
            std::unique_lock lock(_parameter_mutex);
            for (const auto &param_to_change : params) {
                _foxglove_params_map[param_to_change.getName()] = param_to_change.getValue();
            }
            param_copy = _foxglove_params_map;
        }
        _param_update_signal(param_copy);
    };

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle)
    {
        std::vector<foxglove::Parameter> foxglove_params; 
        for (auto &[key, value] : _foxglove_params_map) {
            foxglove::Parameter param(key, value);
            foxglove_params.push_back(param);
        }
        _server->publishParameterValues(clientHandle, foxglove_params, request_id);
    };

    auto descriptors = get_pb_descriptors({"hytech_msgs.proto"});
    std::vector<foxglove::ChannelWithoutId> channels;


    for (const auto &file_descriptor : descriptors) {

        for (int i = 1; i <= file_descriptor->message_type_count(); ++i) {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = message_descriptor->name();
            server_channel.encoding = "protobuf";
            server_channel.schemaName = message_descriptor->full_name();
            server_channel.schema = foxglove::base64Encode(SerializeFdSet(message_descriptor));
            _name_to_id_map[server_channel.topic] = i;
            channels.push_back(server_channel);
        }
    }

    auto res_ids = _server->addChannels(channels);

    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);

    params_file.close(); 
}

boost::signals2::connection core::FoxgloveServer::register_param_callback(std::function<void (const std::unordered_map<std::string, foxglove::ParameterValue> &)> callback) {
    return _param_update_signal.connect(callback);
}

nlohmann::json core::FoxgloveServer::get_all_params() {
    nlohmann::json params_json;
    params_json["type"] = "object";
    
    for (const auto& [name, param_value] : _foxglove_params_map) {
        auto type = param_value.getType();

        if (type == foxglove::ParameterType::PARAMETER_BOOL) {
            params_json[name] = param_value.getValue<bool>();
        } 
        else if (type == foxglove::ParameterType::PARAMETER_DOUBLE) {
            params_json[name] = param_value.getValue<double>();
        } 
        else if (type == foxglove::ParameterType::PARAMETER_INTEGER) {
            params_json[name] = param_value.getValue<int64_t>();
        } 
        else if (type == foxglove::ParameterType::PARAMETER_STRING) {
            params_json[name] = param_value.getValue<std::string>();
        }
    }

    return params_json;
}
void core::FoxgloveServer::send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg) {
    auto msg_chan_id = _name_to_id_map[msg->GetDescriptor()->name()];
    const auto serialized_msg = msg->SerializeAsString(); 
    const auto now = nanosecondsSinceEpoch();
    _server->broadcastMessage(msg_chan_id, now, reinterpret_cast<const uint8_t *>(serialized_msg.data()), serialized_msg.size());
}


