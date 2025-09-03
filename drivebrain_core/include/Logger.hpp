#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdio.h>
#include <string>

namespace core
{
    enum class LogLevel
    {
        NONE = 0,
        ERROR = 1,
        WARNING = 2,
        INFO = 3
    };

    class Logger
    {
        Logger() = delete;

    public:
        Logger(LogLevel internal_lvl) : _lvl(internal_lvl)
        {
        }
        void log_string(const std::string &log_str, LogLevel lvl);

    private:
        bool _do_print(LogLevel lvl);

    private:
        LogLevel _lvl;
    };

}
#endif // __LOGGER_H__