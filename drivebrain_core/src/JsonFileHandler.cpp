
#include <JsonFileHandler.hpp>
#include <spdlog/spdlog.h>

void core::JsonFileHandler::save_config() {
    std::ofstream configFile(_configFilePath);
    if (configFile.is_open())
    {
        configFile << _config.dump(4); // Pretty print with an indent of 4 spaces
    }
    else
    {
        spdlog::error("Failed to open config file for writing.");
    }
}

void core::JsonFileHandler::_load_config() {

    std::ifstream configFile(_configFilePath);
    if (configFile.is_open())
    {
        configFile >> _config;
    }
    else
    {
        spdlog::error("Config file not found");
    }
}
