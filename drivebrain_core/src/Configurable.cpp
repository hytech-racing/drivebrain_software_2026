
#include <Configurable.hpp>
#include <unordered_map>

using namespace core::common;
std::string Configurable::get_name()
{
    return _component_name;
}

std::vector<std::string> Configurable::get_param_names()
{    
    std::vector<std::string> names;
    {
        // lock 
        std::unique_lock lk(_params.mtx);
        for (const auto &param : _params.live_param_vals)
        {
            names.push_back(param.first);
        }
    } // unlocks here
    return names;
}

Configurable::ParamTypes Configurable::get_cached_param(std::string id)
{
    // lock onto the live params mutex so nothing else can access the map of parameters
    std::unique_lock lk(_params.mtx);
    // if the param id exists within the map, return the parameter at this id, otherwise return a monostate variant type (null variant type)
    if(_params.live_param_vals.find(id) != _params.live_param_vals.end())
    {
        return _params.live_param_vals[id]; 
    } else {
        
        return std::monostate();
    }
}

std::unordered_map<std::string, Configurable::ParamTypes> Configurable::get_params_map()
{
    std::unique_lock lk(_params.mtx);
    return _params.live_param_vals;
}

std::unordered_map<std::string, Configurable::ParamTypes> Configurable::get_all_params_map()
{
    std::unique_lock lk(_params.mtx);
    return _params.all_param_vals;
}

void Configurable::handle_live_param_update(const std::string &key, Configurable::ParamTypes param_val)
{
    
    // TODO may want to handle this a little bit differently so we arent locking so much
    {
        std::unique_lock lk(_params.mtx);
        _params.live_param_vals[key] = param_val;
        _params.all_param_vals[key] = param_val;
        // call the user signals that can be optionally attached
        param_update_handler_sig(_params.live_param_vals);
    }
}

std::string Configurable::_get_json_schema_type_name(Configurable::ParamTypeEnum enum_ent)
{
    // as determined by https://json-schema.org/understanding-json-schema/reference/type
    switch(enum_ent)
    {
        case ParamTypeEnum::BOOL_TYPE:
        {
            return std::string("boolean");
        }
        case ParamTypeEnum::INT_TYPE:
        {
            return std::string("integer");
        }
        case ParamTypeEnum::FLOAT_TYPE:
        {
            return std::string("number");
        }
        case ParamTypeEnum::DOUBLE_TYPE:
        {
            return std::string("number");
        }
        case ParamTypeEnum::STRING_TYPE:
        {
            return std::string("string");
        }
        default:
            return std::string("null"); // idk should never get here
    }
}

// TODO does this need a mutex? idk beats me
nlohmann::json Configurable::get_schema() {
    nlohmann::json schema = nlohmann::json::object();
    std::cout << "getting schema for " << get_name() << std::endl;
    schema["type"] = "object";
    {
        std::unique_lock lk(_params.mtx);
        for(const auto & schema_entry : _schema_known_params) {
            schema["properties"][schema_entry.first]["type"] = _get_json_schema_type_name(schema_entry.second);
        }
    }
    return schema;
}
