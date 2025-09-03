
#include <ETHRecvComms.hpp>
#include "hytech_msgs.pb.h"
#include <spdlog/spdlog.h>


using boost::asio::ip::udp;
namespace comms
{
    template <typename ETHMsgType> 
    ETHRecvComms<ETHMsgType>::ETHRecvComms(boost::asio::io_context &io_context,
                                           uint16_t port,
                                           std::shared_ptr<core::StateEstimator> state_estim) 
                                                : _state_estimator(state_estim),
                                                  _socket(io_context, udp::endpoint(udp::v4(), port))
    {
        _eth_msg = std::make_shared<ETHMsgType>();
        _start_receive();
    }

    template <typename ETHMsgType>
    ETHRecvComms<ETHMsgType>::~ETHRecvComms()
    {
        _running = false;
        spdlog::warn("Destructed ETH COMMS");
    }
    
    template <typename ETHMsgType>
    void ETHRecvComms<ETHMsgType>::_handle_receive(const boost::system::error_code &error, std::size_t size)
    {

        if (!error)
        {
            _eth_msg->ParseFromArray(_recv_buffer.data(), size);
            auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>(_eth_msg);
            
            this->log(out_msg);
            
            if(_state_estimator)
            {
                
                _state_estimator->handle_recv_process(out_msg);
            }

            
            _start_receive();
        }
    }

    template <typename ETHMsgType>
    void ETHRecvComms<ETHMsgType>::_start_receive()
    {
        using namespace boost::placeholders;
        _socket.async_receive_from(
            boost::asio::buffer(_recv_buffer), _remote_endpoint,
            boost::bind(&ETHRecvComms::_handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
}
