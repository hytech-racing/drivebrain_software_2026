#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <string>

namespace core
{
/// @brief file handler singleton class. no component should directly write json files with this, we only ever want to have one json file for all parameters. 

class JsonFileHandler {
public:
    JsonFileHandler(const std::string& configFilePath)
        : _configFilePath(configFilePath) {
        _load_config();
    }

    nlohmann::json& get_config() {
        return _config;
    }

    void save_config();

private:
    void _load_config();

    std::string _configFilePath;
    nlohmann::json _config;
};
}