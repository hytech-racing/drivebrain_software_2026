
#include <Logger.hpp>
#include <iostream>
namespace core
{
    void Logger::log_string(const std::string &log_str, LogLevel lvl)
    {
        switch (lvl)
        {
        case (LogLevel::INFO):
        {
            if (_do_print(lvl))
            {
                printf("INFO: ");
                printf("%s", log_str.c_str());
                printf("\n");
            }
            break;
        }
        case (LogLevel::WARNING):
        {
            if (_do_print(lvl))
            {

                printf("\033[33mWARNING: ");
                printf("%s", log_str.c_str());
                printf("\033[0m\n");
            }
            break;
        }
        case (LogLevel::ERROR):
        {
            if (_do_print(lvl))
            {
                printf("\033[31mERROR: ");
                printf("%s", log_str.c_str());
                printf("\033[0m\n");
            }
            break;
        }
        default:
            break;
        }
    }
    bool Logger::_do_print(LogLevel lvl)
    {

        if (static_cast<int>(lvl) <= static_cast<int>(_lvl))
        {
            return true;
        }
        return false;
    }
}
