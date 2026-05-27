
#include <AeroSenseComms.hpp>

#include <algorithm>
#include <array>
#include <cstring>

#include "Telemetry.hpp"
#include "StateTracker.hpp"

namespace {
    constexpr const char *kDeviceName = "/dev/ttyUSB1";
    constexpr int kBaudRate = 500000;
    constexpr std::uint8_t kStartByte = '#';
    constexpr std::size_t kFrameBytesAfterStart = 46;
    constexpr std::size_t kPayloadBytes = 32;
    constexpr std::size_t kFloatBytes = 4;
    constexpr std::size_t kFloatCount = kPayloadBytes / kFloatBytes;
    constexpr std::uint8_t kInitCmd1 = '@';
    constexpr std::uint8_t kInitCmd2 = 'D';

    float decode_float_le(const std::uint8_t *data) {
        std::uint32_t raw = static_cast<std::uint32_t>(data[0]) |
                            (static_cast<std::uint32_t>(data[1]) << 8) |
                            (static_cast<std::uint32_t>(data[2]) << 16) |
                            (static_cast<std::uint32_t>(data[3]) << 24);
        float value = 0.0f;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }
}

namespace comms
{
    AeroSenseDriver::AeroSenseDriver(boost::asio::io_context &io_context) : _serial(io_context) {
        _rx_buffer.reserve(256);
        if (init()) {
            spdlog::info("Starting AeroSense driver receive.");
            _start_recieve();
        }
    }

    bool AeroSenseDriver::init() {
        // Try to establish a connection to the driver
        spdlog::info("Opening AeroSense driver.");
        
        _config.device_name = kDeviceName;
        _config.baud_rate = kBaudRate;


        boost::system::error_code ec;
        auto ec_ret = _serial.open(_config.device_name, ec);

        if (ec) {
            spdlog::warn("Error: {}", ec.message());
            spdlog::info("failed to open AeroSense serial port");
            return false;
        }

        // Set the baud rate of the device along with other configs
        _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
        _serial.set_option(SerialPort::character_size(8));
        _serial.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

        boost::system::error_code write_ec;
        boost::asio::write(_serial, boost::asio::buffer(&kInitCmd1, 1), write_ec);
        if (write_ec) {
            spdlog::warn("Failed to write init '@': {}", write_ec.message());
        }
        boost::asio::write(_serial, boost::asio::buffer(&kInitCmd2, 1), write_ec);
        if (write_ec) {
            spdlog::warn("Failed to write init 'D': {}", write_ec.message());
        }

        return true;


    void AeroSenseDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg) {
        core::log(static_cast<std::shared_ptr<google::protobuf::Message>>(msg));
        core::StateTracker::instance().handle_receive_protobuf_message(msg);
    }

    void AeroSenseDriver::_start_recieve() {
        _serial.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code &ec, std::size_t bytesCount) -> void
            {
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        spdlog::error("ERROR: {}", ec.message());
                    }
                    return;
                }

                _rx_buffer.insert(_rx_buffer.end(), _input_buff.begin(), _input_buff.begin() + bytesCount);

                while (true) {
                    auto start_it = std::find(_rx_buffer.begin(), _rx_buffer.end(), kStartByte);
                    if (start_it == _rx_buffer.end()) {
                        _rx_buffer.clear();
                        break;
                    }

                    if (start_it != _rx_buffer.begin()) {
                        _rx_buffer.erase(_rx_buffer.begin(), start_it);
                    }

                    if (_rx_buffer.size() < 1 + kFrameBytesAfterStart) {
                        break;
                    }

                    const std::uint8_t *payload = _rx_buffer.data() + 1;
                    std::shared_ptr<hytech_msgs::AeroData> msg_out = std::make_shared<hytech_msgs::AeroData>();
                    for (std::size_t i = 0; i < kFloatCount; ++i) {
                        float value = decode_float_le(payload + (i * kFloatBytes));
                        msg_out->add_readings_pa(value);
                    }

                    log_proto_message(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));

                    _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + 1 + kFrameBytesAfterStart);
                }

                _start_recieve();
            });
    }
    }
}