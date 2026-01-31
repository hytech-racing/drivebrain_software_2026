#include "EthernetComms.hpp"
#include "MCAPLogger.hpp"
#include "hytech_msgs.pb.h"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/system/detail/error_code.hpp>
#include <google/protobuf/message.h>
#include <memory>
#include <spdlog/spdlog.h>
#include <type_traits>

using boost::asio::ip::udp;
namespace comms
{
    template <typename MsgType>
    ETHDriver<MsgType>::ETHDriver(boost::asio::io_context& io_context, uint16_t port, std::string send_ip, std::function<void(std::shared_ptr<MsgType>)> handler)
        : _socket(io_context, udp::v4()),
          _send_endp(std::make_optional(udp::endpoint(boost::asio::ip::make_address(send_ip), port))),
          _handler(std::move(handler))
    {
        static_assert(!std::is_same_v<MsgType, void>, "MsgType cannot be void, receive requires a message type");

        _socket.bind(udp::endpoint(udp::v4(), port));
        _received_msg = std::make_shared<MsgType>();

        _running = true;
        _output_thread = std::thread(&ETHDriver::_handle_send_msg_from_queue, this);
        _start_receive();
        spdlog::info("ETHDriver duplex mode, port {}", port);
    }

    template <typename MsgType>
    ETHDriver<MsgType>::ETHDriver(boost::asio::io_context& io_context, uint16_t port, std::function<void(std::shared_ptr<MsgType>)> handler)
        : _socket(io_context, udp::v4()),
          _send_endp(std::nullopt),
          _handler(std::move(handler))
    {
        static_assert(!std::is_same_v<MsgType, void>, "MsgType cannot be void, receive requires a message type");
        
        _socket.bind(udp::endpoint(udp::v4(), port));
        _received_msg = std::make_shared<MsgType>();
        _start_receive();
        spdlog::info("ETHDriver receive-only mode, port {}", port);
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
            _received_msg ->ParseFromArray(_recv_buffer.data(), size);
            _handler(_received_msg);
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
        if (!_send_endp) {
            spdlog::warn("Cannot send, no send endpoint configured");
            return;
        }
        spdlog::debug("Enqueing msg");
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

            for (const auto &msg : temp_msg_queue) {
                _send_message(msg);
            }

            temp_msg_queue.clear();
        }
    }

    template<typename MsgType>
    void ETHDriver<MsgType>::_send_message(std::shared_ptr<google::protobuf::Message> msg_out) {
        spdlog::info("sending ethernet message");
        msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
        _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                                *_send_endp,
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
