#ifndef STATE_TRACKER_H
#define STATE_TRACKER_H

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>
#include <vector> 
#include <string> 
#include <fstream> 
#include <thread>

#include "hytech_msgs.pb.h"
#include "hytech.h"

/**
 * The state tracker acts 
 * as a thread-safe translation unit
 * between our comms interfaces and 
 * the rest of the program.
 */
namespace core {

    template <typename T>
    struct veh_vec {
        T FL;
        T FR;
        T RL;
        T RR;

        template<size_t ind>
        void set_from_index(T val) {
            static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
            if constexpr (ind == 0) {
                FL = val;
            } else if(ind == 1) {
                FR = val;
            } else if(ind == 2) {
                RL = val;
            } else {
                RR = val;
            }
        }
        
        template <size_t ind>
        T get_from_index() {
            static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
            if constexpr (ind == 0) {
                return FL;
            } else if(ind == 1) {
                return FR;
            } else if(ind == 2) {
                return RL;
            } else {
                return RR;
            }
        }

    };

    template <typename T>
    struct xyz_vec {
        T x;
        T y;
        T z;
    };

    template <typename T>
    struct ypr_vec  {   
        T yaw;
        T pitch;
        T roll;
    };

    

    struct RawInputData  {
        veh_vec<float> raw_load_cell_values;
        veh_vec<float> raw_shock_pot_values;
        float raw_steering_analog;
        float raw_steering_digital;
        veh_vec<float> raw_inverter_torques;
        veh_vec<float> raw_inverter_power;
    };

    struct Position {
        double lat;
        double lon;
        bool valid;
    };

    struct DriverInput {
        float requested_accel; // float from 0 to 1 representing percent of accel pedal travel
        float requested_brake; // float from 0 to 1 representing pedal travel of brake
    };

    struct ControllerTorqueOut {
        veh_vec<float> res_torque_lim_nm;
    };

    struct TireDynamics {
        veh_vec<xyz_vec<float>> tire_forces_n;
        veh_vec<xyz_vec<float>> tire_moments_nm;
        veh_vec<float> accel_saturation_nm;
        veh_vec<float> brake_saturation_nm;
        float v_y_lm;
        float psi_dot_lm_rad_s;
        TireDynamics() {
            tire_forces_n = {};
            tire_moments_nm = {};
            accel_saturation_nm = {};
            brake_saturation_nm = {};
            v_y_lm = 0;
            psi_dot_lm_rad_s = 0;
        }
    };

    struct TorqueVectoringStatus {
        veh_vec<float> torque_additional_nm;
        float additional_mz_moment_nm;
        float des_psi_dot;
        float psi_dot_err;
        float perceived_vx;
        float integral_yaw_rate_err;
        float perceived_psi_dot;
    };

    struct MatlabMathResult {
        TireDynamics tire_dynamics;
        TorqueVectoringStatus torque_vectoring_status;
    };

    struct SpeedControlOut {
        int64_t mcu_recv_millis;
        veh_vec<float> desired_rpms;
        veh_vec<float> torque_lim_nm;
    };

    struct TorqueControlOut {
        veh_vec<float> desired_torques_nm;
    };


    struct ControllerOutput {
        std::variant<SpeedControlOut, TorqueControlOut, std::monostate> out;
    };

    struct INSStatus {
        float vel_uncertainty;
        int status_mode;
    };

    struct DrivetrainData {
        veh_vec<float> inverter_igbt_temps_c;
        veh_vec<float> inverter_temps_c;
        veh_vec<float> inverter_motor_temps_c;
    };

    struct AccumulatorData {
        float min_cell_voltage;
        float pack_voltage;
    };

    struct VehicleState {
        bool is_ready_to_drive; 
        xyz_vec<float> current_body_vel_ms;
        xyz_vec<float> current_body_accel_mss;
        xyz_vec<float> current_angular_rate_rads;
        ypr_vec<float> current_ypr_rad;
        veh_vec<float> current_rpms;
        veh_vec<float> motor_overload_percentages;
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

    class StateTracker {

        public: 
            StateTracker();

            void handle_receive_protobuf_message(std::shared_ptr<google::protobuf::Message> message);
            
            void set_previous_control_output(ControllerOutput previous_control_output);

            std::pair<core::VehicleState, bool> get_latest_state_and_validity();

            
        private: 

            template <size_t index, typename inverter_dynamics_message>
            void _handle_set_inverter_dynamics(std::shared_ptr<inverter_dynamics_message> msg);

            template <size_t index, typename inverter_temps_message>
            void _handle_set_inverter_temps(std::shared_ptr<inverter_temps_message> msg);

            template<size_t arr_len> 
            bool _validate_timestamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr) {

            void _receive_low_level_state(std::shared_ptr<google::protobuf::Message> message);

            void _receive_inverter_states(std::shared_ptr<google::protobuf::Message> message);

            VehicleState _vehicle_state;
            RawInputData _raw_input_data;
            std::mutex _state_mutex;
            std::array<std::chrono::microseconds, 4> _timestamp_array; 

    };
}

#endif // STATE_TRACKER_H