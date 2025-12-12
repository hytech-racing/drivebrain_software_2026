#ifndef __LOGGABLE_H__
#define __LOGGABLE_H__

#include "MsgLogger.hpp"

// im really tired of writing this code over and over again
namespace core
{
namespace common
{  

template <typename MsgLoggerMsgType>
class Loggable
{
    public: // this gets called on the object externally
        void set_msg_logger(std::shared_ptr<core::MsgLogger<MsgLoggerMsgType>> message_logger)
        {
            _message_logger = message_logger;
        }

    protected: // only Loggable derived classes can log (the loggable component itself is responsible for logging)
        void log(MsgLoggerMsgType msg)
        {
            if(_message_logger)
            {
                _message_logger->log_msg(msg);
            }
        }
    private:
        std::shared_ptr<core::MsgLogger<MsgLoggerMsgType>> _message_logger = nullptr;
};
}
}
#endif // __LOGGABLE_H__