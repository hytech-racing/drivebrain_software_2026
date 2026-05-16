
#include <AeroSenseComms.hpp>

namespace comms
{
    bool AeroSenseDriver::init() {
        // Try to establish a connection to the driver
        spdlog::info("Opening AeroSense driver.");
        
        _config.device_name = "/dev/ttyUSB1";
        _config.baud_rate = 500000;


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

        

        boost::asio::async_write(_serial,
                boost::asio::buffer(_output_buff.data(), num_of_bytes),
                [](const boost::system::error_code &ec, std::size_t bytes_transferred) {
                if (!ec) {
                    spdlog::info("Successfully sent {} bytes.", bytes_transferred);
                } else {
                    spdlog::error("Error sending data: {}", ec.message());
                }
            });




    }
}