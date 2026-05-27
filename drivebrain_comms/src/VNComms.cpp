#include <VNComms.hpp>

// standard includes
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <utility>

#include "EkfTelemHelpers.hpp"
#include "FoxgloveServer.hpp"
#include "Telemetry.hpp"
#include "base_msgs.pb.h"
#include "htx_ekf/ekf.hpp"
#include "htx_ekf/ekf_manager.hpp"
#include "hytech_msgs.pb.h"
#include "libvncxx/packet.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/types.h"
#include "libvncxx/vntime.h"

namespace
{
struct Vec3
{
    float x;
    float y;
    float z;
};

Vec3 sensor_frd_to_vehicle_frd(const Vec3& s)
{
    // sensor +x = car rear
    // sensor +y = car left (driver perspective)
    // sensor +z = car down

    return {.x = -s.x, .y = -s.y, .z = s.z};
}

Vec3 vehicle_frd_to_vehicle_flu(const Vec3& s)
{
    // FRD +x = FLU +x
    // FRD +y = FLU -y
    // FRD +z = FLU -z

    return {.x = s.x, .y = -s.y, .z = -s.z};
}

void set_xyz(hytech_msgs::xyz_vector* msg, const Vec3& v)
{
    msg->set_x(v.x);
    msg->set_y(v.y);
    msg->set_z(v.z);
}

uint64_t now_ns()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

};  // namespace

namespace comms
{

bool VNDriver::init()
{
    // Try to establish a connection to the driver
    spdlog::info("Opening vn driver.");
    auto& foxglove = core::FoxgloveServer::instance();

    auto device_name = foxglove.get_param<std::string>("vn_driver/device_name");
    auto baud_rate = foxglove.get_param<int>("vn_driver/baud_rate");
    auto bo1_freq_divisor =
        foxglove.get_param<int>("vn_driver/bo1_freq_divisor");
    auto bo2_freq_divisor =
        foxglove.get_param<int>("vn_driver/bo2_freq_divisor");
    auto bo3_freq_divisor =
        foxglove.get_param<int>("vn_driver/bo3_freq_divisor");
    auto port = foxglove.get_param<int>("vn_driver/port");

    if (!(device_name && baud_rate && bo1_freq_divisor && bo2_freq_divisor &&
          bo3_freq_divisor && port))
    {
        spdlog::error("Couldn't load all params for VN Driver");
        return false;
    }

    _config.baud_rate = baud_rate.value();

    _config.bo1_freq_divisor = bo1_freq_divisor.value();
    _config.bo2_freq_divisor = bo2_freq_divisor.value();
    _config.bo3_freq_divisor = bo3_freq_divisor.value();

    _processor.registerPossiblePacketFoundHandler(this,
                                                  &VNDriver::_handle_recieve);

    boost::system::error_code ec;
    auto ec_ret = _serial.open(device_name.value(), ec);

    if (ec)
    {
        spdlog::warn("Error: {}", ec.message());
        spdlog::info("failed to open vectornav serial port");
        return false;
    }

    // Set the baud rate of the device along with other configs
    _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
    _serial.set_option(SerialPort::character_size(8));
    _serial.set_option(SerialPort::parity(SerialPort::parity::none));
    _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
    _serial.set_option(
        SerialPort::flow_control(SerialPort::flow_control::none));

    // Configures the binary outputs for the device
    spdlog::info("Configuring binary outputs.");
    _configure_binary_outputs();

    // spdlog::info("INS config");
    // TODO: uncomment
    // _configure_INS();

    return true;
}

VNDriver::VNDriver(boost::asio::io_context& io, bool& init_not_successful,
                   std::shared_ptr<htx_ekf::EkfManager> ekf_manager)
    : _serial(io), _ekf_manager(std::move(ekf_manager))
{
    init_not_successful = !init();

    // Starts read
    if (!init_not_successful)
    {
        spdlog::info("Starting vn driver recieve.");
        _start_recieve();
    }
}

void VNDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg)
{
    core::log(static_cast<std::shared_ptr<google::protobuf::Message>>(msg));
    core::StateTracker::instance().handle_receive_protobuf_message(msg);
}

// TODO: FINISH IMPLEMENTATION
// Sets the heading estimate to the angle provided by the user, immediately
// initializing the INS filter and expediting the startup process
//      Must be before INS mode 3
//      Have to feed initial heading within 5 degrees of True North
//      Calculate declination (w/Reg21 & 83) and offset Magnetic North to get
//      True North Write result to Reg 161 while VN-300 in Mode 0 to set initial
//      heading and expediate startup to Mode 3
// New method to try and get initial heading from the VN-300
void VNDriver::_try_initialize_heading(float mag_heading, uint8_t ins_mode)
{
    if (_initial_heading_set) return;

    if (ins_mode == 0)
    {
        float true_heading = mag_heading + _local_declination;

        _set_initial_heading(true_heading);

        _initial_heading_set = true;
        spdlog::info("Applied declination of {} deg for True Heading of {}",
                     _local_declination, true_heading);
    }
    return;
}

// Sets the heading estimate to the angle provided by the user, immediately
// initializing the INS filter and expediting the startup process
void VNDriver::_set_initial_heading(float initial_heading)
{
// Packet namespace does not have set intial heading command. Generate the
// $VNSIH command. The manual shows the format: $VNSIH,heading
#if VN_HAVE_SECURE_CRT
    size_t length = sprintf_s((char*)_output_buff.data(), _output_buff.size(),
                              "$VNSIH,%.3f", initial_heading);
#else
    size_t length =
        sprintf((char*)_output_buff.data(), "$VNSIH,%.3f", initial_heading);
#endif

    // Create packet Checksum
    auto num_of_bytes =
        Packet::finalizeCommand(ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
                                (char*)_output_buff.data(), length);

    // Send command to VN-300
    boost::asio::async_write(
        _serial, boost::asio::buffer(_output_buff.data(), num_of_bytes),
        [](const boost::system::error_code& ec, std::size_t bytes_transferred)
        {
            if (!ec)
            {
                spdlog::info("Successfully sent {} bytes.", bytes_transferred);
            }
            else
            {
                spdlog::error("Error sending data: {}", ec.message());
            }
        });
}

void VNDriver::_configure_INS()
{
    // TODO: Separate function for this?? Consider if VN-300 is ARHS-enabled by
    // default?? Creating buffer for writing to INS Basic Configuration
    // (Register 67)
    auto num_of_bytes = Packet::genWriteInsBasicConfiguration(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE, (char*)_output_buff.data(),
        _output_buff.size(),
        0,  // Scenario: AHRS (Attitude Only - no GNSS/INS)
        1,  // Set AhrsAiding: provides the ability to switch to using the
            // magnetometer to stabilize heading during times when GNSS-based
            // heading is unavailable.
        0  // Estimate Baseline: if enabled, the sensor will auto-populate the
           // GNSS Compass Estimated Baseline register (Register 97) with the
           // INS-estimated baseline
    );

    // Write the buffer to the VN-300
    boost::asio::async_write(
        _serial, boost::asio::buffer(_output_buff.data(), num_of_bytes),
        [](const boost::system::error_code& ec, std::size_t bytes_transferred)
        {
            if (!ec)
            {
                spdlog::info("Successfully sent {} bytes.", bytes_transferred);
            }
            else
            {
                spdlog::error("Error sending data: {}", ec.message());
            }
        });

    // TODO: SET WNN HERE
}

void VNDriver::_configure_binary_outputs()
{
    auto bo1_num_bytes = Packet::genWriteBinaryOutput1(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE, (char*)_output_buff.data(),
        _output_buff.size(), AsyncMode::ASYNCMODE_PORT1,
        _config.bo1_freq_divisor,
        (CommonGroup::COMMONGROUP_YAWPITCHROLL),  // Note use of binary OR to
                                                  // configure flags.
        TimeGroup::TIMEGROUP_TIMESTARTUP,
        (ImuGroup::IMUGROUP_UNCOMPACCEL | ImuGroup::IMUGROUP_UNCOMPGYRO |
         ImuGroup::IMUGROUP_ACCEL | ImuGroup::IMUGROUP_ANGULARRATE),
        GpsGroup::GPSGROUP_NONE, AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
        (InsGroup::INSGROUP_NONE), GpsGroup::GPSGROUP_NONE);

    boost::asio::write(_serial,
                       boost::asio::buffer(_output_buff.data(), bo1_num_bytes));

    spdlog::info("Configured VN BO1 with {} bytes", bo1_num_bytes);

    auto bo2_num_bytes = Packet::genWriteBinaryOutput2(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE, (char*)_output_buff.data(),
        _output_buff.size(), AsyncMode::ASYNCMODE_PORT1,
        _config.bo2_freq_divisor,
        (CommonGroup::COMMONGROUP_NONE),  // Note use of binary OR to
                                          // configure flags.
        TimeGroup::TIMEGROUP_TIMESTARTUP, ImuGroup::IMUGROUP_NONE,
        (GpsGroup::GPSGROUP_TOW | GpsGroup::GPSGROUP_WEEK |
         GpsGroup::GPSGROUP_NUMSATS | GpsGroup::GPSGROUP_FIX |
         GpsGroup::GPSGROUP_POSLLA | GpsGroup::GPSGROUP_VELNED |
         GpsGroup::GPSGROUP_POSU | GpsGroup::GPSGROUP_VELU |
         GpsGroup::GPSGROUP_TIMEINFO),
        AttitudeGroup::ATTITUDEGROUP_NONE, (InsGroup::INSGROUP_NONE),
        (GpsGroup::GPSGROUP_TOW | GpsGroup::GPSGROUP_WEEK |
         GpsGroup::GPSGROUP_NUMSATS | GpsGroup::GPSGROUP_FIX |
         GpsGroup::GPSGROUP_POSLLA | GpsGroup::GPSGROUP_VELNED |
         GpsGroup::GPSGROUP_POSU | GpsGroup::GPSGROUP_VELU |
         GpsGroup::GPSGROUP_TIMEINFO));

    boost::asio::write(_serial,
                       boost::asio::buffer(_output_buff.data(), bo2_num_bytes));

    spdlog::info("Configured VN BO2 with {} bytes", bo2_num_bytes);

    auto bo3_num_bytes = Packet::genWriteBinaryOutput3(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE, (char*)_output_buff.data(),
        _output_buff.size(), AsyncMode::ASYNCMODE_PORT1,
        _config.bo3_freq_divisor,
        (CommonGroup::COMMONGROUP_NONE),  // Note use of binary OR to
                                          // configure flags.
        TimeGroup::TIMEGROUP_TIMESTARTUP, ImuGroup::IMUGROUP_NONE,
        GpsGroup::GPSGROUP_NONE, AttitudeGroup::ATTITUDEGROUP_NONE,
        (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA |
         InsGroup::INSGROUP_VELBODY | InsGroup::INSGROUP_VELNED |
         InsGroup::INSGROUP_POSU | InsGroup::INSGROUP_VELU),
        GpsGroup::GPSGROUP_NONE);

    boost::asio::write(_serial,
                       boost::asio::buffer(_output_buff.data(), bo3_num_bytes));

    spdlog::info("Configured VN BO3 with {} bytes", bo3_num_bytes);
}

void VNDriver::_handle_recieve(void* userData,
                               vn::protocol::uart::Packet& packet,
                               size_t runningIndexOfPacketStart, TimeStamp ts)
{
    auto self = static_cast<VNDriver*>(userData);

    // spdlog::info("handling VN data receive...");

    if (packet.type() != vn::protocol::uart::Packet::TYPE_BINARY)
    {
        spdlog::warn("Packet type is incorrect");
        return;
    }

    if (self->_is_bo1_imu_packet(packet))
    {
        self->_handle_bo1_imu_packet(packet, ts);
    }
    else if (self->_is_bo2_gnss_packet(packet))
    {
        self->_handle_bo2_gnss_packet(packet, ts);
    }
    else if (self->_is_bo3_ins_packet(packet))
    {
        self->_handle_bo3_ins_packet(packet, ts);
    }
    else
    {
        spdlog::warn("Unknown VN binary packet");
    }
}

void VNDriver::_start_recieve()
{
    _serial.async_read_some(
        boost::asio::buffer(_input_buff),
        [&](const boost::system::error_code& ec, std::size_t bytesCount) -> void
        {
            if (ec)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    spdlog::error("ERROR: {}", ec.message());
                }
                return;
            }

            // spdlog::info("VN serial read {} bytes", bytesCount);

            // std::string ascii;
            // ascii.reserve(bytesCount);

            // std::string hex;
            // hex.reserve(bytesCount * 3);

            // for (std::size_t i = 0; i < bytesCount; ++i)
            // {
            //     uint8_t b = _input_buff[i];

            //     char hex_byte[4];
            //     std::snprintf(hex_byte, sizeof(hex_byte), "%02X ", b);
            //     hex += hex_byte;

            //     if (std::isprint(b))
            //         ascii.push_back(static_cast<char>(b));
            //     else
            //         ascii.push_back('.');
            // }

            // spdlog::info("VN raw hex: {}", hex);
            // spdlog::info("VN raw ascii: {}", ascii);

            _processor.processReceivedData((char*)(_input_buff.data()),
                                           bytesCount);
            // Initiate another asynchronous read
            _start_recieve();
        });
}

bool VNDriver::_is_bo1_imu_packet(Packet& packet) const
{
    const auto common_groups = CommonGroup::COMMONGROUP_YAWPITCHROLL;
    const auto time_groups = TimeGroup::TIMEGROUP_TIMESTARTUP;
    const auto imu_groups =
        ImuGroup::IMUGROUP_UNCOMPACCEL | ImuGroup::IMUGROUP_UNCOMPGYRO |
        ImuGroup::IMUGROUP_ACCEL | ImuGroup::IMUGROUP_ANGULARRATE;
    const auto gps_groups = GpsGroup::GPSGROUP_NONE;
    const auto attitude_groups = AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY;
    const auto ins_groups = InsGroup::INSGROUP_NONE;

    return packet.isCompatible(common_groups, time_groups, imu_groups,
                               gps_groups, attitude_groups, ins_groups,
                               gps_groups);
}

bool VNDriver::_is_bo2_gnss_packet(Packet& packet) const
{
    const auto common_groups = CommonGroup::COMMONGROUP_NONE;
    const auto time_groups = TimeGroup::TIMEGROUP_TIMESTARTUP;
    const auto imu_groups = ImuGroup::IMUGROUP_NONE;

    const auto gps_groups = GpsGroup::GPSGROUP_TOW | GpsGroup::GPSGROUP_WEEK |
                            GpsGroup::GPSGROUP_NUMSATS |
                            GpsGroup::GPSGROUP_FIX | GpsGroup::GPSGROUP_POSLLA |
                            GpsGroup::GPSGROUP_VELNED |
                            GpsGroup::GPSGROUP_POSU | GpsGroup::GPSGROUP_VELU |
                            GpsGroup::GPSGROUP_TIMEINFO;

    const auto attitude_groups = AttitudeGroup::ATTITUDEGROUP_NONE;
    const auto ins_groups = InsGroup::INSGROUP_NONE;

    return packet.isCompatible(common_groups, time_groups, imu_groups,
                               gps_groups, attitude_groups, ins_groups,
                               gps_groups);
}

bool VNDriver::_is_bo3_ins_packet(Packet& packet) const
{
    const auto common_groups = CommonGroup::COMMONGROUP_NONE;
    const auto time_groups = TimeGroup::TIMEGROUP_TIMESTARTUP;
    const auto imu_groups = ImuGroup::IMUGROUP_NONE;
    const auto gps_groups = GpsGroup::GPSGROUP_NONE;
    const auto attitude_groups = AttitudeGroup::ATTITUDEGROUP_NONE;

    const auto ins_groups =
        InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA |
        InsGroup::INSGROUP_VELBODY | InsGroup::INSGROUP_VELNED |
        InsGroup::INSGROUP_POSU | InsGroup::INSGROUP_VELU;

    return packet.isCompatible(common_groups, time_groups, imu_groups,
                               gps_groups, attitude_groups, ins_groups,
                               gps_groups);
}

void VNDriver::_handle_bo1_imu_packet(Packet& packet, TimeStamp ts)
{
    const uint64_t current_time_ns = now_ns();

    auto ypr_deg = packet.extractVec3f();
    auto time_startup_ns = packet.extractUint64();
    auto uncomp_accel = packet.extractVec3f();
    auto uncomp_gyro = packet.extractVec3f();
    auto comp_accel = packet.extractVec3f();
    auto comp_angular_rate = packet.extractVec3f();
    auto comp_no_gravity_linear_accel = packet.extractVec3f();

    double dt_s = 0.0f;

    if (_last_bo1_time_startup_ns)
    {
        dt_s = static_cast<double>(
            static_cast<double>(time_startup_ns -
                                _last_bo1_time_startup_ns.value()) *
            1e-9);
    }

    _last_bo1_time_startup_ns = time_startup_ns;

    const Vec3 uncomp_accel_sensor_frd{
        .x = uncomp_accel.x, .y = uncomp_accel.y, .z = uncomp_accel.z};

    const Vec3 uncomp_gyro_sensor_frd{
        .x = uncomp_gyro.x, .y = uncomp_gyro.y, .z = uncomp_gyro.z};

    const Vec3 uncomp_accel_vehicle_frd =
        sensor_frd_to_vehicle_frd(uncomp_accel_sensor_frd);
    const Vec3 uncomp_gyro_vehicle_frd =
        sensor_frd_to_vehicle_frd(uncomp_gyro_sensor_frd);

    const Vec3 uncomp_accel_vehicle_flu =
        vehicle_frd_to_vehicle_flu(uncomp_accel_vehicle_frd);
    const Vec3 uncomp_gyro_vehicle_flu =
        vehicle_frd_to_vehicle_flu(uncomp_gyro_vehicle_frd);

    const Vec3 comp_accel_sensor_frd{
        .x = comp_accel.x, .y = comp_accel.y, .z = comp_accel.z};
    const Vec3 comp_angular_rate_sensor_frd{.x = comp_angular_rate.x,
                                            .y = comp_angular_rate.y,
                                            .z = comp_angular_rate.z};

    const Vec3 comp_accel_vehicle_frd =
        sensor_frd_to_vehicle_frd(comp_accel_sensor_frd);
    const Vec3 comp_angular_rate_vehicle_frd =
        sensor_frd_to_vehicle_frd(comp_angular_rate_sensor_frd);

    const Vec3 comp_accel_vehicle_flu =
        vehicle_frd_to_vehicle_flu(comp_accel_vehicle_frd);
    const Vec3 comp_angular_rate_vehicle_flu =
        vehicle_frd_to_vehicle_flu(comp_angular_rate_vehicle_frd);

    const Vec3 comp_no_gravity_accel_sensor_frd = {
        .x = comp_no_gravity_linear_accel.x,
        .y = comp_no_gravity_linear_accel.y,
        .z = comp_no_gravity_linear_accel.z};

    const Vec3 comp_no_gravity_accel_vehicle_frd =
        sensor_frd_to_vehicle_frd(comp_no_gravity_accel_sensor_frd);

    const Vec3 comp_no_gravity_accel_vehicle_flu =
        vehicle_frd_to_vehicle_flu(comp_no_gravity_accel_vehicle_frd);

    auto msg = std::make_shared<hytech_msgs::VnImuData>();

    msg->set_host_timestamp_ns(current_time_ns);
    msg->set_vn_time_startup_ns(time_startup_ns);

    msg->set_dt_s(dt_s);

    set_xyz(msg->mutable_uncomp_accel_sensor_frd_m_ss(),
            uncomp_accel_sensor_frd);
    set_xyz(msg->mutable_uncomp_accel_vehicle_frd_m_ss(),
            uncomp_accel_vehicle_frd);
    set_xyz(msg->mutable_uncomp_accel_vehicle_flu_m_ss(),
            uncomp_accel_vehicle_flu);

    set_xyz(msg->mutable_uncomp_gyro_sensor_frd_rad_s(),
            uncomp_gyro_sensor_frd);
    set_xyz(msg->mutable_uncomp_gyro_vehicle_frd_rad_s(),
            uncomp_gyro_vehicle_frd);
    set_xyz(msg->mutable_uncomp_gyro_vehicle_flu_rad_s(),
            uncomp_gyro_vehicle_flu);

    set_xyz(msg->mutable_comp_accel_sensor_frd_m_ss(), comp_accel_sensor_frd);
    set_xyz(msg->mutable_comp_accel_vehicle_frd_m_ss(), comp_accel_vehicle_frd);
    set_xyz(msg->mutable_comp_accel_vehicle_flu_m_ss(), comp_accel_vehicle_flu);

    set_xyz(msg->mutable_comp_gyro_sensor_frd_rad_s(),
            comp_angular_rate_sensor_frd);
    set_xyz(msg->mutable_comp_gyro_vehicle_frd_rad_s(),
            comp_angular_rate_vehicle_frd);
    set_xyz(msg->mutable_comp_gyro_vehicle_flu_rad_s(),
            comp_angular_rate_vehicle_flu);

    set_xyz(msg->mutable_comp_no_gravity_accel_sensor_frd_m_ss(),
            comp_no_gravity_accel_sensor_frd);
    set_xyz(msg->mutable_comp_no_gravity_accel_vehicle_frd_m_ss(),
            comp_no_gravity_accel_vehicle_frd);
    set_xyz(msg->mutable_comp_no_gravity_accel_vehicle_flu_m_ss(),
            comp_no_gravity_accel_vehicle_flu);

    auto ypr_msg = std::make_shared<hytech_msgs::VnYprData>();

    ypr_msg->set_timestamp_ns(current_time_ns);

    hytech_msgs::ypr_vector* ypr_msg_data = ypr_msg->mutable_ypr_deg();
    ypr_msg_data->set_yaw(ypr_deg.x);
    ypr_msg_data->set_pitch(ypr_deg.y);
    ypr_msg_data->set_roll(ypr_deg.z);

    core::log(std::static_pointer_cast<google::protobuf::Message>(msg));
    core::log(std::static_pointer_cast<google::protobuf::Message>(ypr_msg));

    core::StateTracker::instance().handle_receive_protobuf_message(msg);
    core::StateTracker::instance().handle_receive_protobuf_message(ypr_msg);
    
    // call Ekf manager
    htx_ekf::ImuSample raw_vehicle_aligned_frd_imu_sample;

    raw_vehicle_aligned_frd_imu_sample.time_startup_ns = time_startup_ns;
    raw_vehicle_aligned_frd_imu_sample.dt_s = dt_s;

    raw_vehicle_aligned_frd_imu_sample.ax_frd_m_s2 = uncomp_accel_vehicle_frd.x;
    raw_vehicle_aligned_frd_imu_sample.ay_frd_m_s2 = uncomp_accel_vehicle_frd.y;
    raw_vehicle_aligned_frd_imu_sample.gz_frd_rad_s = uncomp_gyro_vehicle_frd.z;

    if (_ekf_manager)
    {
        const htx_ekf::EkfStepResult ekf_result =
            _ekf_manager->handle_imu(raw_vehicle_aligned_frd_imu_sample);

        comms::publish_ekf_step_result(ekf_result);
    }
}

void VNDriver::_handle_bo2_gnss_packet(Packet& packet, TimeStamp ts)
{
    const uint64_t current_time_ns = now_ns();

    auto time_startup_ns = packet.extractUint64();

    ParsedGnss gnss1_data = extract_gnss(packet);
    ParsedGnss gnss2_data = extract_gnss(packet);

    auto dual_gnss_msg = std::make_shared<hytech_msgs::VnDualGnssData>();

    dual_gnss_msg->set_host_timestamp_ns(current_time_ns);
    dual_gnss_msg->set_vn_time_startup_ns(time_startup_ns);
    fill_gnss_msg(dual_gnss_msg->mutable_gnss1(), gnss1_data);
    fill_gnss_msg(dual_gnss_msg->mutable_gnss2(), gnss2_data);

    htx_ekf::GnssSample gnss1_sample;
    gnss1_sample.tow_ns = gnss1_data.tow_ns;
    gnss1_sample.week = gnss1_data.week;
    gnss1_sample.num_sats = gnss1_data.num_sats;
    gnss1_sample.fix = gnss1_data.fix;
    gnss1_sample.lat_deg = gnss1_data.poslla.x;
    gnss1_sample.lon_deg = gnss1_data.poslla.y;
    gnss1_sample.alt_m = gnss1_data.poslla.z;
    gnss1_sample.vn_m_s = gnss1_data.velned.x;
    gnss1_sample.ve_m_s = gnss1_data.velned.y;
    gnss1_sample.vd_m_s = gnss1_data.velned.z;
    gnss1_sample.posu_n_m = gnss1_data.posu.x;
    gnss1_sample.posu_e_m = gnss1_data.posu.y;
    gnss1_sample.posu_d_m = gnss1_data.posu.z;
    gnss1_sample.velu_m_s = gnss1_data.velu;

    htx_ekf::GnssSample gnss2_sample;
    gnss2_sample.tow_ns = gnss2_data.tow_ns;
    gnss2_sample.week = gnss2_data.week;
    gnss2_sample.num_sats = gnss2_data.num_sats;
    gnss2_sample.fix = gnss2_data.fix;
    gnss2_sample.lat_deg = gnss2_data.poslla.x;
    gnss2_sample.lon_deg = gnss2_data.poslla.y;
    gnss2_sample.alt_m = gnss2_data.poslla.z;
    gnss2_sample.vn_m_s = gnss2_data.velned.x;
    gnss2_sample.ve_m_s = gnss2_data.velned.y;
    gnss2_sample.vd_m_s = gnss2_data.velned.z;
    gnss2_sample.posu_n_m = gnss2_data.posu.x;
    gnss2_sample.posu_e_m = gnss2_data.posu.y;
    gnss2_sample.posu_d_m = gnss2_data.posu.z;
    gnss2_sample.velu_m_s = gnss2_data.velu;

    htx_ekf::DualGnssSample dual_gnss_sample;
    dual_gnss_sample.time_startup_ns = time_startup_ns;
    dual_gnss_sample.gnss1 = gnss1_sample;
    dual_gnss_sample.gnss2 = gnss2_sample;

    if (_ekf_manager)
    {
        const htx_ekf::EkfStepResult ekf_result =
            _ekf_manager->handle_dual_gnss(dual_gnss_sample);

        comms::publish_ekf_step_result(ekf_result);
    }

    core::log(
        std::static_pointer_cast<google::protobuf::Message>(dual_gnss_msg));

    core::StateTracker::instance().handle_receive_protobuf_message(
        dual_gnss_msg);
}

void VNDriver::_handle_bo3_ins_packet(Packet& packet, TimeStamp ts)
{
    uint64_t current_time_ns = now_ns();

    auto time_startup_ns = packet.extractUint64();

    uint16_t raw_ins_status = packet.extractUint16();
    auto pos_lla = packet.extractVec3d();
    auto vel_body = packet.extractVec3f();
    auto vel_ned = packet.extractVec3f();
    auto posu = packet.extractFloat();
    auto velu = packet.extractFloat();

    auto ins_msg = std::make_shared<hytech_msgs::VnInsData>();

    ins_msg->set_timestamp_ns(current_time_ns);

    auto ins_status_msg = ins_msg->mutable_ins_status();
    const uint8_t mode = raw_ins_status & 0x3;
    ins_status_msg->set_ins_mode(static_cast<hytech_msgs::INSMode>(mode));
    ins_status_msg->set_ins_mode_int(mode);

    ins_status_msg->set_gnss_fix(((raw_ins_status >> 2) & 0x1) != 0);

    const uint8_t err = (raw_ins_status >> 3) & 0xF;
    ins_status_msg->set_error_imu(((err >> 1) & 0x1) != 0);
    ins_status_msg->set_error_mag_pres(((err >> 2) & 0x1) != 0);
    ins_status_msg->set_error_gnss(((err >> 3) & 0x1) != 0);

    ins_status_msg->set_gnss_heading_ins(((raw_ins_status >> 8) & 0x1) != 0);

    ins_status_msg->set_gnss_compass(((raw_ins_status >> 9) & 0x1) != 0);

    ins_status_msg->set_ins_vel_u(velu);

    // msg->set_ins_status(ins_status);

    ins_msg->set_lat_deg(pos_lla.x);
    ins_msg->set_lon_deg(pos_lla.y);
    ins_msg->set_alt_m(pos_lla.z);

    const Vec3 vel_body_sensor_frd{
        .x = vel_body.x, .y = vel_body.y, .z = vel_body.z};
    const Vec3 vel_body_vehicle_frd =
        sensor_frd_to_vehicle_frd(vel_body_sensor_frd);
    const Vec3 vel_body_vehicle_flu =
        vehicle_frd_to_vehicle_flu(vel_body_vehicle_frd);

    set_xyz(ins_msg->mutable_vel_body_sensor_frd_m_s(), vel_body_sensor_frd);
    set_xyz(ins_msg->mutable_vel_body_vehicle_frd_m_s(), vel_body_vehicle_frd);
    set_xyz(ins_msg->mutable_vel_body_vehicle_flu_m_s(), vel_body_vehicle_flu);

    const Vec3 vel_ned_vec{.x = vel_ned.x, .y = vel_ned.y, .z = vel_ned.z};
    set_xyz(ins_msg->mutable_vel_ned_m_s(), vel_ned_vec);

    ins_msg->set_posu_m(posu);
    ins_msg->set_velu_m_s(velu);

    core::log(std::static_pointer_cast<google::protobuf::Message>(ins_msg));
    core::StateTracker::instance().handle_receive_protobuf_message(ins_msg);
}

ParsedGnss VNDriver::extract_gnss(Packet& packet)
{
    ParsedGnss out;

    out.tow_ns = packet.extractUint64();
    out.week = packet.extractUint16();
    out.num_sats = packet.extractUint8();
    out.fix = packet.extractUint8();
    out.poslla = packet.extractVec3d();
    out.velned = packet.extractVec3f();
    out.posu = packet.extractVec3f();
    out.velu = packet.extractFloat();
    out.time_status = packet.extractUint8();
    out.leap_seconds = packet.extractInt8();

    return out;
}

void VNDriver::fill_gnss_msg(hytech_msgs::VnGnss* msg, const ParsedGnss& data)
{
    msg->set_tow_ns(data.tow_ns);
    msg->set_week(static_cast<uint32_t>(data.week));
    msg->set_num_sats(static_cast<uint32_t>(data.num_sats));
    msg->set_fix(static_cast<uint32_t>(data.fix));

    msg->set_lat_deg(data.poslla.x);
    msg->set_lon_deg(data.poslla.y);
    msg->set_alt_m(data.poslla.z);

    const Vec3 velned_vec{
        .x = data.velned.x, .y = data.velned.y, .z = data.velned.z};
    set_xyz(msg->mutable_vel_ned_ms(), velned_vec);

    const Vec3 posu_vec{.x = data.posu.x, .y = data.posu.y, .z = data.posu.z};
    set_xyz(msg->mutable_posu_ned_m(), posu_vec);

    msg->set_velu_m_s(data.velu);
    msg->set_time_status(static_cast<uint32_t>(data.time_status));
    msg->set_leap_seconds(static_cast<int32_t>(data.leap_seconds));
}

}  // namespace comms
