#include "EthernetComms.hpp"
#include "hytech_msgs.pb.h"
#include <boost/asio/ip/address.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/detail/error_code.hpp>
#include <google/protobuf/message.h>
#include <spdlog/spdlog.h>
#include <type_traits>

using boost::asio::ip::udp;
namespace comms
{
    template <typename MsgType>
    ETHDriver<MsgType>::ETHDriver(boost::asio::io_context &io_context, uint16_t port, std::string send_ip) 
        : _socket(io_context, udp::endpoint(udp::v4(), port)),
        _send_endp(send_ip.empty()
                    ? udp::endpoint(udp::v4(), port)
                    : udp::endpoint(boost::asio::ip::make_address(send_ip.c_str()), port))
    {
        if(!send_ip.empty()) {
            _socket.bind(_send_endp);
        }
        
        _running = true;
        _output_thread = std::thread(&comms::ETHDriver<MsgType>::_handle_send_msg_from_queue, this);

        if constexpr (!std::is_same_v<MsgType, void>)  {
            _received_eth_msg = std::make_shared<MsgType>();
            _start_receive();
            spdlog::info("ETHDriver full duplex mode initialized for port {}", port);
        } else {
            spdlog::info("ETHDriver send-only mode initialized for port {}", port);
        }

    }

    template <typename MsgType>
    ETHDriver<MsgType>::~ETHDriver() {
        _running = false;
        _send_msg_queue.cv.notify_all();
        if (_output_thread.joinable()) {
             _output_thread.join();
        }
        _socket.close();
        spdlog::warn("Ethernet driver destructed");
    }

    /* Receive methods */
    template <typename MsgType>
    void ETHDriver<MsgType>::_handle_receive(const boost::system::error_code &error, std::size_t size) {
        if (_running && !error) {
            _received_eth_msg ->ParseFromArray(_recv_buffer.data(), size);
            auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>( _received_eth_msg );
            // log to outputs
            // TODO: log to state estimator
            _start_receive();
        }
    }

    template<typename MsgType>
    void ETHDriver<MsgType>::_start_receive() {
        using namespace boost::placeholders;
        _socket.async_receive_from(
            boost::asio::buffer(_recv_buffer), _remote_endpoint,
            boost::bind(&ETHDriver::_handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    /* Send methods */

    template<typename MsgType>
    void ETHDriver<MsgType>::enqueue_msg_send(std::shared_ptr<google::protobuf::Message> send_msg) {
        spdlog::debug("enqueing msg");
        std::unique_lock lk(_send_msg_queue.mtx);
        _send_msg_queue.deque.push_back(send_msg);
        _send_msg_queue.cv.notify_all();
    }

    template<typename MsgType>
    void ETHDriver<MsgType>::_handle_send_msg_from_queue() {
        std::deque<std::shared_ptr<google::protobuf::Message>> temp_msg_queue;
        while(_running) {
            /* transfer messages to temp queue for duration of lock */
            {
                std::unique_lock lk(_send_msg_queue.mtx);
                _send_msg_queue.cv.wait(lk, [this]() { return !_send_msg_queue.deque.empty() || !_running; });

                if(!_running) {
                    break;
                }

                _send_msg_queue.deque.swap(temp_msg_queue);
            }

        }

        for (const auto &msg : temp_msg_queue) {
            _send_message(msg);
            // this->log(msg);
        }

        temp_msg_queue.clear();
    }

    template<typename MsgType>
    void ETHDriver<MsgType>::_send_message(std::shared_ptr<google::protobuf::Message> msg_out) {
        spdlog::info("sending ethernet message");
        msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
        _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                                _send_endp,
                                [](const boost::system::error_code &error, std::size_t bytes_transferred)
        {
            if (error) {
                spdlog::error("async send failed: {}", error.message());
            } else {
                spdlog::debug("async send completed: {} bytes.", bytes_transferred);
            }                        
        });
    }

}
