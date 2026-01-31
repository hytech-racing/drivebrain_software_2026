#pragma once

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/bind.hpp>
#include <boost/system/detail/error_code.hpp>

#include <cstdint>
#include <memory>
#include <google/protobuf/message.h>

#include "DriverBus.hpp"
#include "MCAPLogger.hpp"
#include "hytech_msgs.pb.h"

namespace comms
{
    /**
     * @brief Ethernet driver class for sending and receiving Protobuf messages
     * @tparam MsgType The protobuf message type the initialized Ethernet class will receive.
    */
    template<typename MsgType>
    class ETHDriver {
    public:
        using deqtype = core::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;

        ETHDriver() = delete;
        ~ETHDriver();

        /**
         * Constructor for creating a new instance of the Ethernet driver (full-duplex mode)
         *
         * @param io_context The reference to the initialized io_context
         * @param port The port to listen/receive messages from
         * @param handler The handler function to be called on the received message. By default the message is logged to an MCAP.
         * @param send_ip The IP address to send messages to
        */
        ETHDriver(boost::asio::io_context& io_context, uint16_t port, std::string send_ip,
                  std::function<void(std::shared_ptr<MsgType>)> handler = [](std::shared_ptr<MsgType> msg){ core::MCAPLogger::instance().log_msg(msg); });

        /**
         * Constructor for creating a new instance of the Ethernet driver (receive-only mode)
         *
         * @param io_context The reference to the initialized io_context
         * @param port The port to listen/receive messages from
         * @param handler The handler function to be called on the received message. By default the message is logged to an MCAP.
        */
        ETHDriver(boost::asio::io_context& io_context, uint16_t port,
                  std::function<void(std::shared_ptr<MsgType>)> handler = [](std::shared_ptr<MsgType> msg){ core::MCAPLogger::instance().log_msg(msg); });

        /**
         * Method for adding a new message to the send queue
         *
         * @param send_msg The message to send
        */
        void enqueue_msg_send(std::shared_ptr<google::protobuf::Message> send_msg);

    private:
        /* Receive helpers */

        /* Deserializes the received ethernet message and logs to the MessagerLogger and state estimator */
        void _handle_receive(const boost::system::error_code &error, std::size_t size);
    
        /* Begins message receiving loop */
        void _start_receive();    

        /* Send helpers */

        /* Logs and sends all messages in the message queue */
        void _handle_send_msg_from_queue();

        /* Sends one message from the send buffer */
        void _send_message(std::shared_ptr<google::protobuf::Message> msg_out);

    private:
        std::optional<boost::asio::ip::udp::endpoint> _send_endp;
        deqtype _send_msg_queue;
        std::array<uint8_t, 4096> _send_buffer;
        std::array<uint8_t, 4096> _recv_buffer;
        std::shared_ptr<MsgType> _received_msg;

        std::atomic<bool> _running = false;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::thread _output_thread;

        std::function<void(std::shared_ptr<MsgType>)> _handler;
        
    };

}

#include "EthernetComms.tpp"
