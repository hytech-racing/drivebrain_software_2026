
#ifndef __ETHSENDCOMMS_H__
#define __ETHSENDCOMMS_H__

#include <Logger.hpp>
#include <StateEstimator.hpp>
#include <MsgLogger.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <cstdint>
#include <google/protobuf/message.h>
#include "hytech_msgs.pb.h"
#include <memory>

// - [ ] handle sending UDP messages on a specific port and ip
// TODO:
// figure out if we want to keep the queue work flow for sending or if we want to
// instead use just a direct pointer / ref to a generic driver interface that we
// can give to the estimation / control thread to handle the sending of the control msgs

    // will make the queue internal and expose a public function for enqueue of a protobuf message to send


namespace comms
{
    class ETHSendComms : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>
    {
    public:
        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;
        using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;
        
        ETHSendComms() = delete;
        ~ETHSendComms();
        ETHSendComms(boost::asio::io_context &io_context, uint16_t send_port, std::string send_ip, bool bind=true);
    

        // thread safe call to enqueue a message to the internal dequeue for safe multi-thread access from multiple threads at once
        void enqueue_msg_to_send(std::shared_ptr<google::protobuf::Message> send_msg);
        
    private:
        // is this required? i think that it is
        void _send_completition_handler(std::array<uint8_t, 4096> /*message*/,
                                        const boost::system::error_code & /*error*/,
                                        std::size_t /*bytes_transferred*/);
        void _handle_send_msg_from_queue();
        void _send_message(std::shared_ptr<google::protobuf::Message> msg_out);
        
    private:

        boost::asio::ip::udp::endpoint _endp;

        std::array<uint8_t, 4096> _recv_buffer;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        bool _running = false;
        std::thread _output_thread;
        std::array<uint8_t, 4096> _send_buffer;
        deqtype _queue;
    };

}

#endif // __ETHSENDCOMMS_H__
