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
    /**
     * @brief Ethernet driver class for sending and receiving Protobuf messages
     * @tparam MsgType The protobuf message type the initialized Ethernet class will receive. An empty (void) template type indicates send-only mode
    */
    template<typename MsgType = void>
    class ETHDriver : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>
    {
    public:

        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;

        ETHDriver() = delete;
        ~ETHDriver();

        /**
         * Constructor for creating a new instance of the Ethernet driver
         *
         * @param io_context The reference to the initialized io_context
         * @param port The port to listen/receive messages from
         * @param send_ip The IP address to send messages from (empty by default if in receive-only mode)
         * @param bind Whether or not the socket should bind to local endpoint
        */
        ETHDriver(boost::asio::io_context &io_context, uint16_t port, std::string send_ip = "", bool bind=true);

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
        boost::asio::ip::udp::endpoint _send_endp;
        deqtype _send_msg_queue;
        std::array<uint8_t, 4096> _send_buffer;
        std::array<uint8_t, 4096> _recv_buffer;
        std::shared_ptr<MsgType> _received_eth_msg;

        // std::shared_ptr<core::StateEstimator> _state_estimator = nullptr; //TODO: uncomment when state estimator is added
        std::atomic<bool> _running = false;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::thread _output_thread;
        
    };

}

#endif // __ETHCOMMS_H__
