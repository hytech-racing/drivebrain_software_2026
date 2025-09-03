#pragma once

// TODO make this private
#include <JsonFileHandler.hpp>
#include <spdlog/spdlog.h>

#include <boost/signals2.hpp>

#include <stdexcept>
#include <string>
#include <mutex>
#include <type_traits>
#include <unordered_map>
#include <optional>
#include <variant>
#include <mutex>
#include <vector>
#include <utility>
#include <algorithm>
#include <cctype>
#include <string>

// STORY:

// what I want to be able to have this do is be able to be part of every software component and
// it exposes functionality for handling ptree passed in parameters in a smart way.

// NOTES:

// we will need someway of ensuring that none of the component names are the same

// REQUIREMENTS:

// - [x] configuration should be nested in the following way:
// 1. each component has its own single layer scope
// 2. each scope doesnt have any further scope (one layer deep only)
// - [x] all parameter value getting has to return
// - [x] there will only be ONE config file being edited by ONE thing at a time
// this is pertinent to the parameter server for saving the parameters, for accessing at init time we will initialize before kicking off threads

// this is also pertinent to construction of the core of drivebrain since we want the components to be able to update the in-memory version of the json, but only one thing should be able to write it out to the json file once updated

// need to force each configurable instance to have a handler for the runtime-updating of each parameter? or have a run-time callable config access function
// -> this would be hard to police and ensure that the runtime calling only gets called during runtime

// what if we made all the parameter handling be within a specific function for each component
// then the initial init can call the same set param function that gets called at init time

// live parameters:
// - [x] add live parameter handling through use of boost signals

// - [x] schema creator function from json file 

// the schema will be based off of the json file that we load, however nothing will be a required field.
// this way, even if the json file that we loaded has extra fields that arent being used we will be fine
// we broadcast / record parameter data at around 1 hz OR on change of a parameter 
namespace core
{
    namespace common
    {
        /// @brief this is the (partially virtual) class that configurable components inherit from to get parameter access to the top-level ptree
        class Configurable
        {

        public:
            /// @brief helper type alias
            using ParamTypes = std::variant<bool, int, float, double, std::string, std::monostate>;
            
            enum class ParamTypeEnum 
            {
                BOOL_TYPE=0,
                INT_TYPE=1,
                FLOAT_TYPE=2,
                DOUBLE_TYPE=3,
                STRING_TYPE=4
            };
            
            
            /// @brief constructor for base class
            /// @param logger 
            /// @param json_file_handler the referrence to the json file loaded in main
            /// @param component_name name of the component (required to be unique)
            Configurable(core::JsonFileHandler &json_file_handler, const std::string &component_name)
                : _json_file_handler(json_file_handler), _component_name(component_name) {
                    std::string data = component_name;
                    std::transform(data.begin(), data.end(), data.begin(), [](unsigned char c){ return std::tolower(c); });
                    if(data == std::string("drivebrain_configuration"))
                    {
                        throw std::runtime_error("ERROR reserved configurable component name drivebrain_configuration shall not be used");
                    }
                }

            /// @brief getter for name
            /// @return name of the component
            std::string get_name();

            /// @brief gets the names of the parameters for this component
            /// @return vector of names
            std::vector<std::string> get_param_names();

            
            /// @brief gets the map of param names with live param values
            /// @return unordered map of names and vals (variant)
            std::unordered_map<std::string, ParamTypes> get_params_map();

            /// @brief gets the map of param names with ALL param values
            /// @return unordered map of names and vals (variant)
            std::unordered_map<std::string, ParamTypes> get_all_params_map();

            /// @brief external function signature for use by the parameter server for handling the parameter updates. calls the user-implemented boost signal
            /// @param key the id of the parameter that should be contained within the param map
            /// @param param_val the parameter value to change to 
            void handle_live_param_update(const std::string &key, ParamTypes param_val);

            // TODO renamd id to key to stay consistent with naming, also switch to const ref

            /// @brief getter for param value at specified id
            /// @param id map key for parameter within map
            /// @return param value
            Configurable::ParamTypes get_cached_param(std::string id);
            
            nlohmann::json get_schema();
            bool is_configured() { return _configured; }
            
        protected:
            void set_configured() { _configured = true; }
            /// @brief boost signal that the user is expected to connect their parameter update handler function for changing their internal parameter values
            boost::signals2::signal<void(const std::unordered_map<std::string, ParamTypes> &)> param_update_handler_sig;

            /// @brief virtual init function that has to be implemented. after a successful init the member var @ref _configured must be set to true.
            // it is expected that the use puts their getters for live and "static" parameters within this function. 
            /// @return false if not all params found that were expected, true if all params were good. other initialization code can be included not pertaining to configuration base class as well
            virtual bool init() = 0;


            // TODO look into making this private

            /// @brief internal type checker to ensure that param types are what they are only what we support
            /// @tparam ParamType the desired parameter type
            template <typename ParamType>
            void _handle_assert()
            {
                static_assert(
                    std::is_same_v<ParamType, bool> ||
                        std::is_same_v<ParamType, int> ||
                        std::is_same_v<ParamType, double> ||
                        std::is_same_v<ParamType, float> ||
                        std::is_same_v<ParamType, std::string>,
                    "ParamType must be bool, int, double, float, or std::string");
            }
            /// @brief Gets a parameter value within the component's scope from the shared config file, ensuring it exists with and returning std::nullopt if it does not
            /// @tparam ParamType parameter type that can be a bool, int, double, float or string
            /// @param key the id of the parameter being requested within the namespace of this component's name
            /// @return the optional config value std::nullopt if not available, otherwise it is an optional POD
            template <typename ParamType>
            std::optional<ParamType> get_parameter_value(const std::string &key)
            {
                _handle_assert<ParamType>();

                // TODO assert that the template type is only of the specific types supported by nlohmann's json lib
                nlohmann::json &config = _json_file_handler.get_config();

                // Ensure the component's section exists and if it doesnt we created it
                if (!config.contains(_component_name))
                {
                    auto log_str = std::string("config file does not contain config for component: ") + _component_name;
                    spdlog::warn(log_str);
                    return std::nullopt;
                }

                // Access the specific key within the component's section
                if (!config[_component_name].contains(key))
                {
                    auto log_str = std::string("config file does not contain config: ") + key + std::string(" for component: ") + _component_name;
                    spdlog::warn(log_str);
                    return std::nullopt;
                }
                auto type_enum = get_param_enum_type<ParamType>();
                _schema_known_params.push_back(std::make_pair(key, type_enum));
                
                {
                    std::unique_lock lk(_params.mtx);
                    spdlog::info("got param: {}", key);
                    spdlog::info(config[_component_name][key].get<ParamType>());
                    _params.all_param_vals[key] = config[_component_name][key].get<ParamType>();
                }
                
                return config[_component_name][key].get<ParamType>();
            }


            /// @brief same as the @ref get_parameter_value function, however it also registers the parameter to the internal live parameter map
            /// @tparam ParamType the parameter type
            /// @param key  the id of the parameter being requested
            /// @return the optional config value, std::nullopt if not found
            template <typename ParamType>
            std::optional<ParamType> get_live_parameter(const std::string &key)
            {
                _handle_assert<ParamType>();
                std::optional res = get_parameter_value<ParamType>(key);

                if (!res)
                {
                    return std::nullopt;
                }
                else
                {
                    {
                        std::unique_lock lk(_params.mtx);
                        _params.live_param_vals[key] = *res;
                    }
                }
                return res;
            }

            template <typename ParamType>
            ParamTypeEnum get_param_enum_type()
            {
                if constexpr (std::is_same_v<ParamType, bool>) {
                    return ParamTypeEnum::BOOL_TYPE;
                } else if(std::is_same_v<ParamType, int>) {
                    return ParamTypeEnum::INT_TYPE;
                } else if(std::is_same_v<ParamType, float>) {
                    return ParamTypeEnum::FLOAT_TYPE;
                } else if(std::is_same_v<ParamType, double>) {
                    return ParamTypeEnum::DOUBLE_TYPE;
                } else if(std::is_same_v<ParamType, std::string>) {
                    return ParamTypeEnum::STRING_TYPE;
                }
            }

            

        private:
            std::string _get_json_schema_type_name(ParamTypeEnum enum_type);
        private:
            // this vector gets appended to as the user calls the get_live_parameter() 
            // or get_parameter_value() (which calls get_parameter_value) within the init function of the
            // configurable component with the param name and type in a pair

            std::vector<std::pair<std::string, ParamTypeEnum>> _schema_known_params;
            
            bool _configured = false;
            std::string _component_name;
            core::JsonFileHandler &_json_file_handler;
            
            /// @brief anonymous struct for associating the mutex with what it is guarding specifically within this class
            struct
            {
                std::unordered_map<std::string, ParamTypes> live_param_vals;
                std::unordered_map<std::string, ParamTypes> all_param_vals;
                std::mutex mtx;
            } _params;
        };

    }

}