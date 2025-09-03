#ifndef __VEHICLESTATE_H__
#define __VEHICLESTATE_H__
#include <Utils.hpp>
#include <Literals.hpp>
#include <variant>
#include <cstdint>

// TODO this will need a one-to-one protobuf message
namespace core
{

    // TODO change all driver inputs to use this?
    // this is the struct that will contain the data that is already
    // being logged via the sensor interfaces and doesnt go directly
    // into the vehicle state
    struct RawInputData
    {
        veh_vec<float> raw_load_cell_values;
        veh_vec<float> raw_shock_pot_values;
        float raw_steering_analog;
        float raw_steering_digital;
        veh_vec<float> raw_inverter_torques;
        veh_vec<float> raw_inverter_power;
    };

    struct Position
    {
        double lat;
        double lon;
        bool valid;
    };

    struct DriverInput
    {
        float requested_accel; // float from 0 to 1 representing percent of accel pedal travel
        float requested_brake; // float from 0 to 1 representing pedal travel of brake
    };
    struct ControllerTorqueOut
    {
        veh_vec<float> res_torque_lim_nm;
    };
    struct TireDynamics
    {
        veh_vec<xyz_vec<float>> tire_forces_n;
        veh_vec<xyz_vec<float>> tire_moments_nm;
        veh_vec<float> accel_saturation_nm;
        veh_vec<float> brake_saturation_nm;
        float v_y_lm;
        float psi_dot_lm_rad_s;
        TireDynamics()
        {
            tire_forces_n = {};
            tire_moments_nm = {};
            accel_saturation_nm = {};
            brake_saturation_nm = {};
            v_y_lm = 0;
            psi_dot_lm_rad_s = 0;
        }
    };

    struct TorqueVectoringStatus
    {
        veh_vec<float> torque_additional_nm;
        float additional_mz_moment_nm;
        float des_psi_dot;
        float psi_dot_err;
        float perceived_vx;
        float integral_yaw_rate_err;
        float perceived_psi_dot;
    };

    struct MatlabMathResult
    {
        TireDynamics tire_dynamics;
        TorqueVectoringStatus torque_vectoring_status;
    };

    struct SpeedControlOut
    {
        int64_t mcu_recv_millis;
        veh_vec<float> desired_rpms;
        veh_vec<float> torque_lim_nm;
    };

    struct TorqueControlOut
    {
        veh_vec<torque_nm> desired_torques_nm;
    };


    // we will have both speed and torque control output controllers
    struct ControllerOutput
    {
        std::variant<SpeedControlOut, TorqueControlOut, std::monostate> out;
    };
    struct INSStatus 
    {
        float vel_uncertainty;
        int status_mode;
    };
    struct DrivetrainData
    {
        veh_vec<float> inverter_igbt_temps_c;
        veh_vec<float> inverter_temps_c;
        veh_vec<float> inverter_motor_temps_c;
    };
    struct AccumulatorData
    {
        float min_cell_voltage;
        float pack_voltage;
    };
    struct VehicleState
    {
        bool is_ready_to_drive;
        DriverInput input;
        xyz_vec<float> current_body_vel_ms;
        xyz_vec<float> current_body_accel_mss;
        xyz_vec<float> current_angular_rate_rads;
        ypr_vec<float> current_ypr_rad;
        veh_vec<float> current_rpms;
        bool state_is_valid;
        int prev_MCU_recv_millis;
        float steering_angle_deg;
        ControllerOutput prev_controller_output;
        TireDynamics tire_dynamics;
        veh_vec<float> driver_torque;
        ControllerTorqueOut matlab_math_temp_out;
        veh_vec<float> suspension_potentiometers_mm;
        Position vehicle_position;
        veh_vec<float> loadcells;
        veh_vec<float> current_torques_nm;
        INSStatus ins_status;
        float old_energy_meter_kw;
        DrivetrainData dt_data;
        AccumulatorData acc_data;
    };


    namespace control
    {
        enum class ControllerManagerStatus
        {
            NO_ERROR = 0,
            ERROR_CONTROLLER_INDEX_OUT_OF_RANGE = 1,
            ERROR_SPEED_TOO_HIGH = 2,
            ERROR_TORQUE_TOO_HIGH = 3,
            ERROR_DRIVER_ON_PEDAL = 4,
            ERROR_CONTROLLER_NO_TORQUE_OR_SPEED_OUTPUT = 5,
            ERROR_OUTPUT_EXCEEDS_PHYS_LIMITS = 6,
            ERROR_REQUESTING_SAME_CTR_TYPE = 7,
            NUM_CONTROLLER_MANAGER_STATUSES = 8,
            ERROR_NULLPTR_CONTROLLER = 9
        };

        struct ControllerManagerState
        {
            ControllerManagerStatus current_status;
            ControllerOutput current_controller_output;
        };
    }

}
#endif // __VEHICLESTATE_H__