
#ifndef __MSG_LOGGER__
#define __MSG_LOGGER__

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include <deque>
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <filesystem>

#include <iostream>

#include <spdlog/spdlog.h>

namespace core
{
    template <typename MsgType>
    class MsgLogger
    {
    public:
        struct ThreadSafeOutput
        {
            std::mutex mtx;
            std::condition_variable cv;
            std::deque<MsgType> deque;
        };

        

        MsgLogger(const std::string &log_file_extension,
                  bool init_logging,
                  std::function<void(MsgType)> logger_msg_func,
                  std::function<void()> stop_log_func,
                  std::function<void(const std::string &)> open_log_func,
                  std::function<void(MsgType)> live_msg_output_func,
                  std::function<bool()> init_param_schema_for_mcap,
                  std::function<void()> log_params_func) : _logger_msg_function(logger_msg_func),
                                                           _live_msg_output_func(live_msg_output_func),
                                                           _log_file_extension(log_file_extension),
                                                           _stop_log_function(stop_log_func),
                                                           _open_log_function(open_log_func),
                                                           _init_param_schema_for_mcap(init_param_schema_for_mcap),
                                                           _log_params_func(log_params_func)
        {
            _running = true;

            if (init_logging)
            {
                auto log_name = _generate_log_name(_log_file_extension);
                _current_log_name = log_name;
                _open_log_function(log_name);
                if(_init_param_schema_for_mcap())
                {
                    spdlog::info("inited and logged params on creation");

                    _param_schema_written = true;
                    _log_params_func();
                } else {
                    spdlog::error("unable to init params schema for mcap, will be attempting again in params log thread");
                    throw std::runtime_error("all configurable components must be both inited and every init() function must set_configured()");
                }
                
                _logging = init_logging;
            }
        }

        void start_logging_params()
        {
            _param_log_thread = std::thread([this]() { _handle_param_log(); });
        }

        ~MsgLogger()
        {
            spdlog::info("destructing message logger");
            _logging = false;
            _running = false;
            _param_log_condition.notify_all();
            spdlog::info("notif sent to param thread to stop");
            if(_param_log_thread.joinable())
            {
                _param_log_thread.join();
            }
            std::cout << "safely exited MsgLogger" << std::endl;
        }
        void halt() 
        { 
            _running = false; 
            _logging = false; 
            _param_log_condition.notify_all();
            spdlog::info("notif sent to param thread to stop");
        }

        void log_msg(MsgType msg)
        {
            // TODO maybe make this also more thread safe ... ?
            if(_running)
            {
                bool logging = true;
                {
                    std::unique_lock lk(_mtx);
                    logging = _logging;
                }

                if (logging)
                {
                    _handle_output_messages(msg, _logger_msg_function);   
                }
                _handle_output_messages(msg, _live_msg_output_func);
            }
            
        }

        // will only open a new file for logging if we are not currently logging
        void start_logging_to_new_file()
        {
            std::unique_lock lk(_mtx);
            if (!_logging)
            {
                _logging = true;
                _current_log_name = _generate_log_name(_log_file_extension);
                _open_log_function(_current_log_name);
                if(_init_param_schema_for_mcap())
                {
                    _param_schema_written = true;
                    
                } else {
                    _param_schema_written = false;
                    spdlog::error("logger cannot be initialzied due to not being able to get param schemas written up front. please init all configurable components before construction of MsgLogger"); 
                    
                    // for now i will throw a runtime error, i cant be bothered to fully implement this feature rn
                    throw std::runtime_error("Failed to construct MsgLogger");
                }
            }
        }
        std::tuple<std::string, bool> get_logger_status()
        {
            std::unique_lock lk(_mtx);
            return {_current_log_name, _logging};
        }

        void stop_logging_to_file()
        {
            spdlog::info("attempting stop");
            {
                std::unique_lock lk(_mtx);
                if (_logging)
                {
                    _logging = false;
                    _stop_log_function();
                    _param_schema_written = false;
                }
            }
            spdlog::info("stopped logging");
        }

    private:
        std::string _generate_log_name(const std::string &extension) {
            // Get the current time
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            // Create a stringstream to format the date and time
            std::stringstream ss;
            namespace fs = std::filesystem;
            if(fs::exists("/etc/nixos"))
            {
                ss << "/home/nixos/recordings/";
            }

            ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
               << extension;

            // Return the generated filename
            return ss.str();
        }

        void _handle_output_messages(MsgType msg, std::function<void(MsgType)> output_function)
        {            
            output_function(msg);
        }

        void _handle_param_log()
        {
            // TODO add in interuptible sleep that can be killed from the destructor
            std::chrono::seconds log_time(1); // 1hz param logging
            while(true)
            {
                
                {
                    std::unique_lock lk(_cv_mtx);
                    if(_param_log_condition.wait_for(lk, log_time, [this] {return !_logging; }))
                    {
                        spdlog::info("exiting log thread due to stopping of logging");
                        return;
                    }

                    if(!_running)
                    {
                        spdlog::info("stopping running");
                        return;
                    }
                    
                }
                if(_param_schema_written)
                {
                    spdlog::info("logged params");
                    _log_params_func(); // this function is already thread safe since it also requests lock on the writer queue mutex
                } 

            
            }
        }

        

    private:
        bool _running = true;
        bool _logging = false;
        // mutex protects the underlying logger and member vars of the message logger
        std::mutex _mtx;

        std::mutex _cv_mtx;
        std::condition_variable _param_log_condition;
        
        
        std::thread _param_log_thread;
        std::string _log_file_extension;
        std::string _current_log_name = "NONE";
        std::function<void()> _stop_log_function;
        std::function<void(const std::string &)> _open_log_function;
        std::function<void(MsgType)> _logger_msg_function;
        bool _param_schema_written = false;
        std::function<void()> _log_params_func;
        std::function<bool()> _init_param_schema_for_mcap;
        std::function<void(MsgType)> _live_msg_output_func;
    };
}
#endif // __MSG_LOGGER__
