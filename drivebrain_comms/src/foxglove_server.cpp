
#include <foxglove_server.hpp>
#include <memory>
#include <variant>
#include <hytech_msgs.pb.h>
#include <ProtobufUtils.hpp>

#include <queue>
#include <unordered_map>

#include <string>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <algorithm>

#include <spdlog/spdlog.h>

static uint64_t nanosecondsSinceEpoch()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count());
}

core::FoxgloveWSServer::FoxgloveWSServer(std::vector<std::weak_ptr<core::common::Configurable>> configurable_components) : _components(configurable_components)
{
    _log_handler = [](foxglove::WebSocketLogLevel, char const *msg)
    {
        spdlog::warn("{}", msg);
    };

    _server_options.capabilities.push_back("parameters");

    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "T.A.R.S", _log_handler, _server_options);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle)
    {
        std::vector<foxglove::Parameter> params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, params_vec, request_id);
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter> &params, const std::optional<std::string> &request_id, foxglove::ConnHandle clientHandle)
    {
        // loop through all of the params we are trying to change

        for (const auto &param_to_change : params)
        {
            // sets the drivebrain parameters
            _set_db_param(param_to_change);
        }
        // get the updated params
        std::vector<foxglove::Parameter> fxglove_params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, fxglove_params_vec, request_id);
    };

    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle)
    {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::warn("Client {} subscribed to {}", clientStr, chanId);
    };

    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle)
    {
        const auto clientStr = _server->remoteEndpointString(clientHandle);
        spdlog::warn("Client {} unsubscribed from {}", clientStr, chanId);
    };

    // TODO make the .proto file name a parameter
    std::vector<std::string> proto_names = {"hytech_msgs.proto", "hytech.proto"};

    auto potential_id_map = util::generate_name_to_id_map(proto_names);
    if (potential_id_map)
    {
        _id_name_map = *potential_id_map;
    }

    auto descriptors = util::get_pb_descriptors(proto_names);

    std::vector<foxglove::ChannelWithoutId> channels;

    for (const auto &file_descriptor : descriptors)
    {

        for (int i = 0; i < file_descriptor->message_type_count(); ++i)
        {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            foxglove::ChannelWithoutId server_channel;
            server_channel.topic = message_descriptor->name();
            server_channel.encoding = "protobuf";
            server_channel.schemaName = message_descriptor->full_name();
            server_channel.schema = foxglove::base64Encode(util::build_file_descriptor_set(message_descriptor).SerializeAsString());
            channels.push_back(server_channel);
        }
    }

    
    auto res_ids = _server->addChannels(channels);
    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 5555);
}

void core::FoxgloveWSServer::send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg)
{
    if (_id_name_map.find(msg->GetDescriptor()->name()) != _id_name_map.end())
    {
        auto msg_chan_id = _id_name_map[msg->GetDescriptor()->name()];
        const auto serializedMsg = msg->SerializeAsString();
        const auto now = nanosecondsSinceEpoch();
        _server->broadcastMessage(msg_chan_id, now, reinterpret_cast<const uint8_t *>(serializedMsg.data()),
                                  serializedMsg.size());
    }
}

foxglove::Parameter core::FoxgloveWSServer::_get_foxglove_param(const std::string &set_name, core::common::Configurable::ParamTypes param)
{
    if (std::holds_alternative<bool>(param))
    {
        spdlog::warn("{} Variant holds a bool: {}", set_name, std::get<bool>(param));

        return foxglove::Parameter(set_name, std::get<bool>(param));
    }
    else if (std::holds_alternative<int>(param))
    {
        // std::cout << set_name << " Variant holds a int: " << std::get<int>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<int>(param));
    }
    else if (std::holds_alternative<float>(param))
    {
        // std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, ((double)std::get<float>(param)));
    }
    else if (std::holds_alternative<double>(param))
    {
        // std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<double>(param));
    }
    else if (std::holds_alternative<std::string>(param))
    {
        // std::cout << set_name << " Variant holds a string: " << std::get<std::string>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<std::string>(param));
    }
    else
    {
        spdlog::warn("{} unknown param variant type", set_name);
    }

    return {};
}

core::common::Configurable::ParamTypes core::FoxgloveWSServer::_get_db_param(foxglove::Parameter param)
{

    if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_BOOL)
    {
        return param.getValue().getValue<bool>();
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_INTEGER)
    {
        return ((int)param.getValue().getValue<int64_t>());
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_DOUBLE)
    {
        return ((float)param.getValue().getValue<double>());
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_STRING)
    {
        return param.getValue().getValue<std::string>();
    }
    else
    {
        spdlog::warn("unsupported param type");
    }

    return std::monostate();
}

std::optional<foxglove::Parameter> core::FoxgloveWSServer::_convert_foxglove_param(core::common::Configurable::ParamTypes curr_param_val, foxglove::Parameter incoming_param)
{
    foxglove::ParameterType current_param_type = _get_foxglove_param(incoming_param.getName(), curr_param_val).getValue().getType();
    foxglove::ParameterType type = incoming_param.getValue().getType();

    using fpt = foxglove::ParameterType;
    if (_get_foxglove_param(incoming_param.getName(), curr_param_val).getValue().getType() == incoming_param.getValue().getType())
    {
        return incoming_param;
    }
    else if (current_param_type == fpt::PARAMETER_DOUBLE && type == fpt::PARAMETER_INTEGER)
    {
        return foxglove::Parameter(incoming_param.getName(), static_cast<double>(incoming_param.getValue().getValue<int64_t>()));
    }
    else if (current_param_type == fpt::PARAMETER_INTEGER && type == fpt::PARAMETER_DOUBLE)
    {
        return foxglove::Parameter(incoming_param.getName(), static_cast<int64_t>(incoming_param.getValue().getValue<double>()));
    }
    else
    {
        spdlog::warn("WARNING: unsupported type input, not setting");
        return std::nullopt;
    }
}

void core::FoxgloveWSServer::_set_db_param(foxglove::Parameter param_update)
{
    size_t split_pos = param_update.getName().find("/");

    std::string param_name = param_update.getName().substr(split_pos + 1);
    std::string component_name = param_update.getName().substr(0, split_pos);

    for (const auto component_locked : _components)
    {
        if(auto component = component_locked.lock())
        {
            if (component->get_name() == component_name)
            {

                core::common::Configurable::ParamTypes curr_param_val = component->get_cached_param(param_name);
                std::optional<foxglove::Parameter> converted_type = _convert_foxglove_param(curr_param_val, param_update);
                if(converted_type)
                {
                    core::common::Configurable::ParamTypes val = _get_db_param(*converted_type);
                    component->handle_live_param_update(param_name, val);
                }
                return;
            }
        }
        
    }
    spdlog::warn("WARNING: could not find component {}", component_name);

}

std::vector<foxglove::Parameter> core::FoxgloveWSServer::_get_current_params()
{
    std::vector<foxglove::Parameter> params;
    for (const auto component_locked : _components)
    {
        if(auto component = component_locked.lock())
        {
            std::unordered_map params_map = component->get_params_map();
            std::string param_parent = component->get_name();
            std::vector<std::string> param_names = component->get_param_names();
            for (const auto &component_param_name : param_names)
            {
                std::string foxglove_param_id = param_parent + "/" + component_param_name;
                foxglove::Parameter fxglove_param = _get_foxglove_param(foxglove_param_id, params_map[component_param_name]);
                params.push_back(fxglove_param);
            }
        }
        
    }
    return params;
}
