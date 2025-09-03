#pragma once
// drivebrain includes
#include <Configurable.hpp>
#include <Loggable.hpp>
#include <DriverBus.hpp>
#include <MsgLogger.hpp>
#include <StateEstimator.hpp>
#include <hytech.pb.h> // generated from CAN description

// system includes
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

// boost
#include <boost/asio.hpp>

// dbcppp
#include <Signal.h>
#include <Network.h>

// c++ stl includes
#include <memory>
#include <deque>
#include <variant>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <optional>

#include <unistd.h>
#include <cstring>


// TODO
// - [x] be able to hook into the driver bus tx and rx queues to send and receive multiple messages
// - [x] implement functions to be able to deserialize a CAN message from the bus
// - [x] function to create a protobuf message from a de-serialized CAN message
// - [x] implement functions to be able to create a CAN message from the protobuf message
// - [x] needs to be able to take in an abstract pb message
// - [x] get all fields and the message name
// - [x] for each field get their values
// - [x] align the message name and message fields to a CAN message
// - [x] populate the fields of the CAN message with the data gotten from the pb message
// - [x] implement function to send can message

// https://docs.kernel.org/networking/can.html

namespace comms
{
    class CANDriver : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>,
                      public core::common::Configurable
    {
    public:
        using FieldVariant = std::variant<int32_t, int64_t, uint32_t, uint64_t, float, double, bool, std::string, std::monostate>;
        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;
        /// @brief constructur
        /// @param json_file_handler the file handler 
        /// @param in_deq tx queue
        /// @param out_deq receive queue
        /// @param io_context boost asio required context
        CANDriver(core::JsonFileHandler &json_file_handler, deqtype &in_deq, boost::asio::io_context& io_context, std::optional<std::string> dbc_path, bool &construction_failed, std::shared_ptr<core::StateEstimator> state_estimator) : 
            Configurable(json_file_handler, "CANDriver"),
            _input_deque_ref(in_deq),
            _socket(io_context),
            _dbc_path(dbc_path),
            _state_estimator(state_estimator)
        {
            _running = true;
            _output_thread = std::thread(&comms::CANDriver::_handle_send_msg_from_queue, this);
            construction_failed = !init();
        }

        CANDriver(core::JsonFileHandler &json_file_handler, deqtype &in_deq, boost::asio::io_context& io_context, std::optional<std::string> dbc_path, bool &construction_failed, std::shared_ptr<core::StateEstimator> state_estimator, std::string driver_name) : 
            Configurable(json_file_handler, driver_name),
            _input_deque_ref(in_deq),
            _socket(io_context),
            _dbc_path(dbc_path),
            _state_estimator(state_estimator)
        {
            _running = true;
            _output_thread = std::thread(&comms::CANDriver::_handle_send_msg_from_queue, this);
            construction_failed = !init();
        }
        ~CANDriver();
        bool init();


        void _handle_send_msg_from_queue();
        std::shared_ptr<google::protobuf::Message> pb_msg_recv(const can_frame &in_frame);
        void set_field_values_of_pb_msg(const std::unordered_map<std::string, FieldVariant> &field_values, std::shared_ptr<google::protobuf::Message> message);

        // TODO: move this into a util library?

        /// @brief get the value from a field within a protobuf message based on the name of the field
        /// @param message
        /// @param field_name
        /// @return variant of types
        FieldVariant get_field_value(std::shared_ptr<google::protobuf::Message> message, const std::string &field_name);

        
        // for exposing to the test framework directly
    protected:
        // socket operations
        bool _open_socket(const std::string& interface_name);
        void _do_read();
        void _send_message(const struct can_frame& frame);

        void _handle_recv_CAN_frame(const struct can_frame& frame);

        std::shared_ptr<google::protobuf::Message> _get_pb_msg_by_name(const std::string &name);
        std::optional<can_frame> _get_CAN_msg(std::shared_ptr<google::protobuf::Message> msg);

    private:
        static std::string _to_lowercase(std::string s);

    private:
        deqtype &_input_deque_ref;

        std::condition_variable _cv;
        std::thread _output_thread;

        struct can_frame _frame;

        boost::asio::posix::stream_descriptor _socket;
        std::optional<std::string> _dbc_path;

        std::unordered_map<uint64_t, std::unique_ptr<dbcppp::IMessage>> _messages;
        std::unordered_map<std::string, uint64_t> _messages_names_and_ids;
        int _CAN_socket; // can socket bound to
        bool _running = false;
        std::shared_ptr<core::StateEstimator> _state_estimator;
        size_t _recv_process_count = 0;
        const size_t _recv_process_log_interval = 1000;
        long _recv_process_max_duration_us = 0;           // Microseconds
    };
}