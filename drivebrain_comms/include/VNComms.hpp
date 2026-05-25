#pragma once

#include <StateTracker.hpp>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>

#include "FoxgloveServer.hpp"
#include "hytech_msgs.pb.h"

// boost
#include <boost/array.hpp>
#include <boost/asio.hpp>

// c++ stl includes
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <condition_variable>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>

#include "libvncxx/packet.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/vntime.h"

using namespace vn::xplat;
using namespace vn::protocol::uart;
using SerialPort = boost::asio::serial_port;

namespace htx_ekf
{
class EkfManager;
}

namespace comms
{
struct ParsedGnss
{
    uint64_t tow_ns;
    uint16_t week;
    uint8_t num_sats;
    uint8_t fix;
    vn::math::vec3d poslla;
    vn::math::vec3f velned;
    vn::math::vec3f posu;
    float velu;
    uint8_t time_status;
    int8_t leap_seconds;
};

class VNDriver
{
   public:
    VNDriver(boost::asio::io_context& io_context, bool& init_successful,
             std::shared_ptr<htx_ekf::EkfManager> ekf_manager);
    ~VNDriver() { spdlog::info("destructed %s"); }
    bool init();
    struct config
    {
        int baud_rate;
        int bo1_freq_divisor;
        int bo2_freq_divisor;
        int bo3_freq_divisor;
    };

   private:
    vn::protocol::uart::PacketFinder _processor;
    boost::array<std::uint8_t, 512> _output_buff;
    boost::array<std::uint8_t, 512> _input_buff;
    SerialPort _serial;
    config _config;
    bool _initial_heading_set = false;
    float _local_declination;

    std::optional<uint64_t> _last_bo1_time_startup_ns;

   public:
    // Public methods
    void log_proto_message(std::shared_ptr<google::protobuf::Message> msg);

   private:
    // Private methods
    static void _handle_recieve(void* userData,
                                vn::protocol::uart::Packet& packet,
                                size_t runningIndexOfPacketStart, TimeStamp ts);
    void _configure_INS();
    void _configure_binary_outputs();
    void _start_recieve();
    void _set_initial_heading(float initial_heading);
    void _try_initialize_heading(float mag_heading, uint8_t ins_mode);

    bool _is_bo1_imu_packet(Packet& packet) const;
    bool _is_bo2_gnss_packet(Packet& packet) const;
    bool _is_bo3_ins_packet(Packet& packet) const;

    void _handle_bo1_imu_packet(Packet& packet, TimeStamp ts);
    void _handle_bo2_gnss_packet(Packet& packet, TimeStamp ts);
    void _handle_bo3_ins_packet(Packet& packet, TimeStamp ts);

    ParsedGnss extract_gnss(Packet& packet);
    void fill_gnss_msg(hytech_msgs::VnGnss* msg, const ParsedGnss& data);

    std::shared_ptr<htx_ekf::EkfManager> _ekf_manager;
};
}  // namespace comms
