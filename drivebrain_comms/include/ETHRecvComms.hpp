
#ifndef __ETHRECVCOMMS_H__
#define __ETHRECVCOMMS_H__

#include <memory>
#include <cstdint>

#include <Loggable.hpp>
#include <StateEstimator.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <google/protobuf/message.h>
#include "hytech_msgs.pb.h"

// - [x] boost asio socket for udp port comms
// - [x] handle receiving UDP messages on a specific port
// - [x] handle parsing of UDP message as protobuf message on the port
// TODO:
// figure out if we want to keep the queue work flow for sending or if we want to
// instead use just a direct pointer / ref to a generic driver interface that we
// can give to the estimation / control thread to handle the sending of the control msgs

namespace comms
{
    
    template<typename ETHMsgType>
    class ETHRecvComms : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>
    {
    public:
        
        ETHRecvComms() = delete;
        ~ETHRecvComms();
        ETHRecvComms(boost::asio::io_context &io_context, uint16_t recv_port, std::shared_ptr<core::StateEstimator> state_estim=nullptr);
        
        
    private:
        void _handle_receive(const boost::system::error_code &error, std::size_t size);
        void _start_receive();
        
    private:
        std::shared_ptr<core::StateEstimator> _state_estimator = nullptr;
        std::array<uint8_t, 4096> _recv_buffer;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::shared_ptr<ETHMsgType> _eth_msg;
        bool _running = false;
        std::thread _output_thread;
    };

}

#include "ETHRecvComms.tpp"

#endif // __ETHRECVCOMMS_H__
