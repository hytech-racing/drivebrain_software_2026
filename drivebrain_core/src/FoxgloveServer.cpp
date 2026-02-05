#include <FoxgloveServer.hpp>
#include <foxglove/websocket/parameter.hpp>
#include <nlohmann/detail/value_t.hpp>
#include <fstream>
#include <spdlog/spdlog.h>


static std::string to_lowercase(std::string s) {
   std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return static_cast<unsigned char>(std::tolower(c)); });
   return s;
}

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

void core::FoxgloveServer::destroy() {
    FoxgloveServer* instance = _s_instance.exchange(nullptr, std::memory_order_acq_rel);
    if (instance) {
        delete instance;
    }
}

void core::FoxgloveServer::_init_params(const nlohmann::json &json_obj, const std::string &prefix) {
    for (auto &[key, value] : json_obj.items()) {
        std::string param_name = prefix.empty() ? to_lowercase(key) : prefix + "/" + to_lowercase(key);
        
        if (value.type() == nlohmann::detail::value_t::object) {
            _init_params(value, param_name);
        } else {
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
            } else if (value.type() == nlohmann::detail::value_t::string) {
                std::string raw_value = value.get<std::string>();
                param_value = foxglove::ParameterValue(raw_value);
            } else {
                spdlog::error("Invalid parameter config type: {} for key: {}", value.type_name(), param_name);
                continue;
            }

            if (_foxglove_params_map.find(param_name) != _foxglove_params_map.end()) {
                spdlog::warn("Duplicate parameter detected: {}", param_name);
            } else {
                _foxglove_params_map[param_name] = param_value;
                std::cout << "Added parameter: " << param_name << std::endl;
            }
        }
    }
}

core::FoxgloveServer::FoxgloveServer(std::string file_name) {

    // Read params data
    std::fstream params_file(file_name);
    nlohmann::json init_param_data = nlohmann::json::parse(params_file); 

    _init_params(init_param_data, "");

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
            for (const auto &incoming_param : params) {
                if (_foxglove_params_map.find(incoming_param.getName()) == _foxglove_params_map.end()) {
                    spdlog::warn("Couldn't find updated param in params map. Please get good.");
                    continue;
                }
                foxglove::Parameter current_param = foxglove::Parameter(incoming_param.getName(), _foxglove_params_map[incoming_param.getName()]);
                std::optional<foxglove::Parameter> converted_param = _convert_foxglove_parameter(current_param, incoming_param);
                if (converted_param) {
                    _foxglove_params_map[incoming_param.getName()] = converted_param.value().getValue();
                }
            }
            param_copy = _foxglove_params_map;
        }
        MCAPLogger::instance().log_params(get_all_params()); /* Needed for showing param updates in the outputted MCAP file */
        _param_update_signal(param_copy);
    };

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle) {
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
    std::unique_lock lock(_parameter_mutex);
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

std::optional<foxglove::Parameter> core::FoxgloveServer::_convert_foxglove_parameter(foxglove::Parameter current_param, foxglove::Parameter incoming_param) {
    if (current_param.getType() == incoming_param.getType()) {
        return incoming_param; 
    } else if (current_param.getType() == foxglove::ParameterType::PARAMETER_INTEGER && incoming_param.getType() == foxglove::ParameterType::PARAMETER_DOUBLE) {
        return foxglove::Parameter(current_param.getName(), static_cast<int64_t>(incoming_param.getValue().getValue<double>()));
    } else if (current_param.getType() == foxglove::ParameterType::PARAMETER_DOUBLE && incoming_param.getType() == foxglove::ParameterType::PARAMETER_INTEGER) {
        return foxglove::Parameter(current_param.getName(), static_cast<double>(incoming_param.getValue().getValue<int64_t>()));
    } else {
        spdlog::warn("Invalid parameter type conversion!");
        return std::nullopt;
    }
}

void core::FoxgloveServer::send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg) {
    auto msg_chan_id = _name_to_id_map[msg->GetDescriptor()->name()];
    const auto serialized_msg = msg->SerializeAsString(); 
    const auto now = nanosecondsSinceEpoch();
    _server->broadcastMessage(msg_chan_id, now, reinterpret_cast<const uint8_t *>(serialized_msg.data()), serialized_msg.size());
}


