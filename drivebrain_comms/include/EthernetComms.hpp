#ifndef __ETHCOMMS_H__
#define __ETHCOMMS_H__

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/bind.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstdint>
#include <google/protobuf/message.h>

#include "Loggable.hpp"
#include "DriverBus.hpp"
#include "hytech_msgs.pb.h"
#include <memory>


namespace comms
{
    template<typename MsgType = void>
    class ETHDriver : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>
    {
    public:

        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;

        ETHDriver() = delete;
        ~ETHDriver();
        ETHDriver(boost::asio::io_context &io_context, uint16_t port, std::string send_ip = "", bool bind=true);

        void enqueue_msg_send(std::shared_ptr<google::protobuf::Message> send_msg);
    private:
        /* Receive */
        void _handle_receive(const boost::system::error_code &error, std::size_t size);
        void _start_receive();    

        /* Send */
        void _handle_send_msg_from_queue();
        void _send_message(std::shared_ptr<google::protobuf::Message> msg_out);

    private:
        boost::asio::ip::udp::endpoint _send_endp;
        deqtype _send_msg_queue;
        std::array<uint8_t, 4096> _send_buffer;
        std::array<uint8_t, 4096> _recv_buffer;
        std::shared_ptr<MsgType> _received_eth_msg;

        // std::shared_ptr<core::StateEstimator> _state_estimator = nullptr;
        std::atomic<bool> _running = false;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::thread _output_thread;
        
    };

}

#endif // __ETHCOMMS_H__
