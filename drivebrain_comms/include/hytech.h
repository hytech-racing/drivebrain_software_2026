#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_HYTECH_MAJ (6U)
#define VER_HYTECH_MIN (0U)

// include current dbc-driver compilation config
#include "hytech-config.h"

#ifdef HYTECH_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
#include "canmonitorutil.h"

#endif // HYTECH_USE_DIAG_MONITORS


// DLC maximum value which is used as the limit for frame's data buffer size.
// Client can set its own value (not sure why) in driver-config
// or can test it on some limit specified by application
// e.g.: static_assert(TESTDB_MAX_DLC_VALUE <= APPLICATION_FRAME_DATA_SIZE, "Max DLC value in the driver is too big")
#ifndef HYTECH_MAX_DLC_VALUE
// The value which was found out by generator (real max value)
#define HYTECH_MAX_DLC_VALUE 8U
#endif

// The limit is used for setting frame's data bytes
#define HYTECH_VALIDATE_DLC(msgDlc) (((msgDlc) <= (HYTECH_MAX_DLC_VALUE)) ? (msgDlc) : (HYTECH_MAX_DLC_VALUE))

// Initial byte value to be filles in data bytes of the frame before pack signals
// User can define its own custom value in driver-config file
#ifndef HYTECH_INITIAL_BYTE_VALUE
#define HYTECH_INITIAL_BYTE_VALUE 0U
#endif


// def @IMU CAN Message (1    0x1)
#define IMU_IDE (0U)
#define IMU_DLC (8U)
#define IMU_CANID (0x1U)
// signal: @Accel_X_ro
#define HYTECH_Accel_X_ro_CovFactor (0.057)
#define HYTECH_Accel_X_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.057)) )
#define HYTECH_Accel_X_ro_fromS(x) ( (((x) * (0.057)) + (0.0)) )
// signal: @Accel_Y_ro
#define HYTECH_Accel_Y_ro_CovFactor (0.057)
#define HYTECH_Accel_Y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.057)) )
#define HYTECH_Accel_Y_ro_fromS(x) ( (((x) * (0.057)) + (0.0)) )
// signal: @Accel_Z_ro
#define HYTECH_Accel_Z_ro_CovFactor (0.057)
#define HYTECH_Accel_Z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.057)) )
#define HYTECH_Accel_Z_ro_fromS(x) ( (((x) * (0.057)) + (0.0)) )
// signal: @Roll_Rate_ro
#define HYTECH_Roll_Rate_ro_CovFactor (0.01)
#define HYTECH_Roll_Rate_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_Roll_Rate_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @Yaw_Rate_ro
#define HYTECH_Yaw_Rate_ro_CovFactor (0.01)
#define HYTECH_Yaw_Rate_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_Yaw_Rate_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @Pitch_Rate_ro
#define HYTECH_Pitch_Rate_ro_CovFactor (0.01)
#define HYTECH_Pitch_Rate_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_Pitch_Rate_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t Accel_X_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_X_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Accel_Y_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_Y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Accel_Z_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_Z_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Roll_Rate_ro;                      //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Roll_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Yaw_Rate_ro;                       //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Yaw_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Pitch_Rate_ro;                     //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Pitch_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t Accel_X_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_X_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Accel_Y_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_Y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Accel_Z_ro;                        //  [-] Bits=10 Factor= 0.057           Unit:'m/s2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Accel_Z_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Roll_Rate_ro;                      //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Roll_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Yaw_Rate_ro;                       //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Yaw_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Pitch_Rate_ro;                     //  [-] Bits=11 Factor= 0.01            Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Pitch_Rate_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} IMU_t;

// def @State CAN Message (2    0x2)
#define State_IDE (0U)
#define State_DLC (5U)
#define State_CANID (0x2U)
// signal: @Temperature_Internal_ro
#define HYTECH_Temperature_Internal_ro_CovFactor (0.5)
#define HYTECH_Temperature_Internal_ro_toS(x) ( (int8_t) (((x) - (20.0)) / (0.5)) )
#define HYTECH_Temperature_Internal_ro_fromS(x) ( (((x) * (0.5)) + (20.0)) )

// Value tables for @sensor_0_Validity signal

#ifndef sensor_0_Validity_State_Sensor_Valid_sense_0
#define sensor_0_Validity_State_Sensor_Valid_sense_0 (0)
#endif

#ifndef sensor_0_Validity_State_Response_Timeout_sense_0
#define sensor_0_Validity_State_Response_Timeout_sense_0 (1)
#endif

#ifndef sensor_0_Validity_State_Bad_Optical_Signal_sense_0
#define sensor_0_Validity_State_Bad_Optical_Signal_sense_0 (2)
#endif

#ifndef sensor_0_Validity_State_Speed_Too_High_sense_0
#define sensor_0_Validity_State_Speed_Too_High_sense_0 (3)
#endif


// Value tables for @sensor_1_Validity signal

#ifndef sensor_1_Validity_State_Sensor_Valid_sense_1
#define sensor_1_Validity_State_Sensor_Valid_sense_1 (0)
#endif

#ifndef sensor_1_Validity_State_Response_Timeout_sense_1
#define sensor_1_Validity_State_Response_Timeout_sense_1 (1)
#endif

#ifndef sensor_1_Validity_State_Bad_Optical_Signal_sense_1
#define sensor_1_Validity_State_Bad_Optical_Signal_sense_1 (2)
#endif

#ifndef sensor_1_Validity_State_Speed_Too_High_sense_1
#define sensor_1_Validity_State_Speed_Too_High_sense_1 (3)
#endif


// Value tables for @sensor_2_Validity signal

#ifndef sensor_2_Validity_State_Sensor_Valid_sense_2
#define sensor_2_Validity_State_Sensor_Valid_sense_2 (0)
#endif

#ifndef sensor_2_Validity_State_Response_Timeout_sense_2
#define sensor_2_Validity_State_Response_Timeout_sense_2 (1)
#endif

#ifndef sensor_2_Validity_State_Bad_Optical_Signal_sense_2
#define sensor_2_Validity_State_Bad_Optical_Signal_sense_2 (2)
#endif

#ifndef sensor_2_Validity_State_Speed_Too_High_sense_2
#define sensor_2_Validity_State_Speed_Too_High_sense_2 (3)
#endif


// Value tables for @sensor_3_Validity signal

#ifndef sensor_3_Validity_State_Sensor_Valid_sense_3
#define sensor_3_Validity_State_Sensor_Valid_sense_3 (0)
#endif

#ifndef sensor_3_Validity_State_Response_Timeout_sense_3
#define sensor_3_Validity_State_Response_Timeout_sense_3 (1)
#endif

#ifndef sensor_3_Validity_State_Bad_Optical_Signal_sense_3
#define sensor_3_Validity_State_Bad_Optical_Signal_sense_3 (2)
#endif

#ifndef sensor_3_Validity_State_Speed_Too_High_sense_3
#define sensor_3_Validity_State_Speed_Too_High_sense_3 (3)
#endif


// Value tables for @SpeedBeam_Health signal

#ifndef SpeedBeam_Health_State_SENSOR_HEALTHY
#define SpeedBeam_Health_State_SENSOR_HEALTHY (0)
#endif

#ifndef SpeedBeam_Health_State_NON_MEASUREMENT_MODE
#define SpeedBeam_Health_State_NON_MEASUREMENT_MODE (1)
#endif

#ifndef SpeedBeam_Health_State_DATA_OPTICAL_QUALITY_POOR
#define SpeedBeam_Health_State_DATA_OPTICAL_QUALITY_POOR (2)
#endif

#ifndef SpeedBeam_Health_State_TEMPERATURE_TOO_HIGH
#define SpeedBeam_Health_State_TEMPERATURE_TOO_HIGH (3)
#endif

#ifndef SpeedBeam_Health_State_SAVING_CALIBRATION
#define SpeedBeam_Health_State_SAVING_CALIBRATION (4)
#endif

#ifndef SpeedBeam_Health_State_SENSOR_COMMUNICATION_ERROR
#define SpeedBeam_Health_State_SENSOR_COMMUNICATION_ERROR (5)
#endif

#ifndef SpeedBeam_Health_State_SPEED_TOO_HIGH
#define SpeedBeam_Health_State_SPEED_TOO_HIGH (6)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int8_t Temperature_Internal_ro;            //  [-] Bits= 8 Offset= 20.0               Factor= 0.5             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Temperature_Internal_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t Firmware_Version;                 //      Bits=16

  //  0 : "Sensor_Valid_sense_0"
  //  1 : "Response_Timeout_sense_0"
  //  2 : "Bad_Optical_Signal_sense_0"
  //  3 : "Speed_Too_High_sense_0"
  uint8_t sensor_0_Validity : 2;             //      Bits= 2

  //  0 : "Sensor_Valid_sense_1"
  //  1 : "Response_Timeout_sense_1"
  //  2 : "Bad_Optical_Signal_sense_1"
  //  3 : "Speed_Too_High_sense_1"
  uint8_t sensor_1_Validity : 2;             //      Bits= 2

  //  0 : "Sensor_Valid_sense_2"
  //  1 : "Response_Timeout_sense_2"
  //  2 : "Bad_Optical_Signal_sense_2"
  //  3 : "Speed_Too_High_sense_2"
  uint8_t sensor_2_Validity : 2;             //      Bits= 2

  //  0 : "Sensor_Valid_sense_3"
  //  1 : "Response_Timeout_sense_3"
  //  2 : "Bad_Optical_Signal_sense_3"
  //  3 : "Speed_Too_High_sense_3"
  uint8_t sensor_3_Validity : 2;             //      Bits= 2

  //  0 : "SENSOR_HEALTHY"
  //  1 : "NON_MEASUREMENT_MODE"
  //  2 : "DATA_OPTICAL_QUALITY_POOR"
  //  3 : "TEMPERATURE_TOO_HIGH"
  //  4 : "SAVING_CALIBRATION"
  //  5 : "SENSOR_COMMUNICATION_ERROR"
  //  6 : "SPEED_TOO_HIGH"
  uint8_t SpeedBeam_Health : 3;              //      Bits= 3

#else

  int8_t Temperature_Internal_ro;            //  [-] Bits= 8 Offset= 20.0               Factor= 0.5             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Temperature_Internal_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t Firmware_Version;                 //      Bits=16

  //  0 : "Sensor_Valid_sense_0"
  //  1 : "Response_Timeout_sense_0"
  //  2 : "Bad_Optical_Signal_sense_0"
  //  3 : "Speed_Too_High_sense_0"
  uint8_t sensor_0_Validity;                 //      Bits= 2

  //  0 : "Sensor_Valid_sense_1"
  //  1 : "Response_Timeout_sense_1"
  //  2 : "Bad_Optical_Signal_sense_1"
  //  3 : "Speed_Too_High_sense_1"
  uint8_t sensor_1_Validity;                 //      Bits= 2

  //  0 : "Sensor_Valid_sense_2"
  //  1 : "Response_Timeout_sense_2"
  //  2 : "Bad_Optical_Signal_sense_2"
  //  3 : "Speed_Too_High_sense_2"
  uint8_t sensor_2_Validity;                 //      Bits= 2

  //  0 : "Sensor_Valid_sense_3"
  //  1 : "Response_Timeout_sense_3"
  //  2 : "Bad_Optical_Signal_sense_3"
  //  3 : "Speed_Too_High_sense_3"
  uint8_t sensor_3_Validity;                 //      Bits= 2

  //  0 : "SENSOR_HEALTHY"
  //  1 : "NON_MEASUREMENT_MODE"
  //  2 : "DATA_OPTICAL_QUALITY_POOR"
  //  3 : "TEMPERATURE_TOO_HIGH"
  //  4 : "SAVING_CALIBRATION"
  //  5 : "SENSOR_COMMUNICATION_ERROR"
  //  6 : "SPEED_TOO_HIGH"
  uint8_t SpeedBeam_Health;                  //      Bits= 3

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} State_t;

// def @Settings CAN Message (3    0x3)
#define Settings_IDE (0U)
#define Settings_DLC (6U)
#define Settings_CANID (0x3U)

// Value tables for @Broadcast_rate signal

#ifndef Broadcast_rate_Settings_hz_1
#define Broadcast_rate_Settings_hz_1 (0)
#endif

#ifndef Broadcast_rate_Settings_hz_5
#define Broadcast_rate_Settings_hz_5 (1)
#endif

#ifndef Broadcast_rate_Settings_hz_10
#define Broadcast_rate_Settings_hz_10 (2)
#endif

#ifndef Broadcast_rate_Settings_hz_50
#define Broadcast_rate_Settings_hz_50 (3)
#endif

#ifndef Broadcast_rate_Settings_hz_100
#define Broadcast_rate_Settings_hz_100 (4)
#endif

#ifndef Broadcast_rate_Settings_hz_250
#define Broadcast_rate_Settings_hz_250 (5)
#endif

#ifndef Broadcast_rate_Settings_hz_500
#define Broadcast_rate_Settings_hz_500 (6)
#endif


// Value tables for @CAN_Termination_Setting_Bus_1 signal

#ifndef CAN_Termination_Setting_Bus_1_Settings_CAN_BUS_UNTERMINATED_CAN1_setting
#define CAN_Termination_Setting_Bus_1_Settings_CAN_BUS_UNTERMINATED_CAN1_setting (0)
#endif

#ifndef CAN_Termination_Setting_Bus_1_Settings_CAN_BUS_TERMINATED_CAN1_setting
#define CAN_Termination_Setting_Bus_1_Settings_CAN_BUS_TERMINATED_CAN1_setting (1)
#endif


// Value tables for @CAN_Termination_Setting_Bus_2 signal

#ifndef CAN_Termination_Setting_Bus_2_Settings_CAN_BUS_UNTERMINATED_CAN2_setting
#define CAN_Termination_Setting_Bus_2_Settings_CAN_BUS_UNTERMINATED_CAN2_setting (0)
#endif

#ifndef CAN_Termination_Setting_Bus_2_Settings_CAN_BUS_TERMINATED_CAN2_setting
#define CAN_Termination_Setting_Bus_2_Settings_CAN_BUS_TERMINATED_CAN2_setting (1)
#endif

// signal: @Height_Offset_ro
#define HYTECH_Height_Offset_ro_CovFactor (0.5)
#define HYTECH_Height_Offset_ro_toS(x) ( (int8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_Height_Offset_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @Sensor_X_Location_ro
#define HYTECH_Sensor_X_Location_ro_CovFactor (0.01)
#define HYTECH_Sensor_X_Location_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_Sensor_X_Location_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @Sensor_Y_Location_ro
#define HYTECH_Sensor_Y_Location_ro_CovFactor (0.01)
#define HYTECH_Sensor_Y_Location_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_Sensor_Y_Location_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

// Value tables for @Attitude_Filtering signal

#ifndef Attitude_Filtering_Settings_NO_FILTERING_ATT
#define Attitude_Filtering_Settings_NO_FILTERING_ATT (0)
#endif

#ifndef Attitude_Filtering_Settings_LIGHT_FILTERING_ATT
#define Attitude_Filtering_Settings_LIGHT_FILTERING_ATT (1)
#endif

#ifndef Attitude_Filtering_Settings_MEDIUM_FILTERING_ATT
#define Attitude_Filtering_Settings_MEDIUM_FILTERING_ATT (2)
#endif

#ifndef Attitude_Filtering_Settings_HEAVY_FILTERING_ATT
#define Attitude_Filtering_Settings_HEAVY_FILTERING_ATT (3)
#endif


// Value tables for @IMU_Filtering signal

#ifndef IMU_Filtering_Settings_NO_FILTERING_IMU
#define IMU_Filtering_Settings_NO_FILTERING_IMU (0)
#endif

#ifndef IMU_Filtering_Settings_LIGHT_FILTERING_IMU
#define IMU_Filtering_Settings_LIGHT_FILTERING_IMU (1)
#endif

#ifndef IMU_Filtering_Settings_MEDIUM_FILTERING_IMU
#define IMU_Filtering_Settings_MEDIUM_FILTERING_IMU (2)
#endif

#ifndef IMU_Filtering_Settings_HEAVY_FILTERING_IMU
#define IMU_Filtering_Settings_HEAVY_FILTERING_IMU (3)
#endif


// Value tables for @Velocity_Filtering signal

#ifndef Velocity_Filtering_Settings_NO_FILTERING_VEL
#define Velocity_Filtering_Settings_NO_FILTERING_VEL (0)
#endif

#ifndef Velocity_Filtering_Settings_LIGHT_FILTERING_VEL
#define Velocity_Filtering_Settings_LIGHT_FILTERING_VEL (1)
#endif

#ifndef Velocity_Filtering_Settings_MEDIUM_FILTERING_VEL
#define Velocity_Filtering_Settings_MEDIUM_FILTERING_VEL (2)
#endif

#ifndef Velocity_Filtering_Settings_HEAVY_FILTERING_VEL
#define Velocity_Filtering_Settings_HEAVY_FILTERING_VEL (3)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  //  0 : "hz_1"
  //  1 : "hz_5"
  //  2 : "hz_10"
  //  3 : "hz_50"
  //  4 : "hz_100"
  //  5 : "hz_250"
  //  6 : "hz_500"
  uint8_t Broadcast_rate : 3;                  //      Bits= 3

  //  0 : "CAN_BUS_UNTERMINATED_CAN1_setting"
  //  1 : "CAN_BUS_TERMINATED_CAN1_setting"
  uint8_t CAN_Termination_Setting_Bus_1 : 1;   //      Bits= 1

  //  0 : "CAN_BUS_UNTERMINATED_CAN2_setting"
  //  1 : "CAN_BUS_TERMINATED_CAN2_setting"
  uint8_t CAN_Termination_Setting_Bus_2 : 1;   //      Bits= 1

  int8_t Height_Offset_ro;                     //  [-] Bits= 8 Factor= 0.5             Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_Offset_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_X_Location_ro;                //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_X_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_Y_Location_ro;                //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_Y_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  //  0 : "NO_FILTERING_ATT"
  //  1 : "LIGHT_FILTERING_ATT"
  //  2 : "MEDIUM_FILTERING_ATT"
  //  3 : "HEAVY_FILTERING_ATT"
  uint8_t Attitude_Filtering : 3;              //      Bits= 3

  //  0 : "NO_FILTERING_IMU"
  //  1 : "LIGHT_FILTERING_IMU"
  //  2 : "MEDIUM_FILTERING_IMU"
  //  3 : "HEAVY_FILTERING_IMU"
  uint8_t IMU_Filtering : 3;                   //      Bits= 3

  //  0 : "NO_FILTERING_VEL"
  //  1 : "LIGHT_FILTERING_VEL"
  //  2 : "MEDIUM_FILTERING_VEL"
  //  3 : "HEAVY_FILTERING_VEL"
  uint8_t Velocity_Filtering : 3;              //      Bits= 3

#else

  //  0 : "hz_1"
  //  1 : "hz_5"
  //  2 : "hz_10"
  //  3 : "hz_50"
  //  4 : "hz_100"
  //  5 : "hz_250"
  //  6 : "hz_500"
  uint8_t Broadcast_rate;                      //      Bits= 3

  //  0 : "CAN_BUS_UNTERMINATED_CAN1_setting"
  //  1 : "CAN_BUS_TERMINATED_CAN1_setting"
  uint8_t CAN_Termination_Setting_Bus_1;       //      Bits= 1

  //  0 : "CAN_BUS_UNTERMINATED_CAN2_setting"
  //  1 : "CAN_BUS_TERMINATED_CAN2_setting"
  uint8_t CAN_Termination_Setting_Bus_2;       //      Bits= 1

  int8_t Height_Offset_ro;                     //  [-] Bits= 8 Factor= 0.5             Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_Offset_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_X_Location_ro;                //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_X_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_Y_Location_ro;                //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_Y_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  //  0 : "NO_FILTERING_ATT"
  //  1 : "LIGHT_FILTERING_ATT"
  //  2 : "MEDIUM_FILTERING_ATT"
  //  3 : "HEAVY_FILTERING_ATT"
  uint8_t Attitude_Filtering;                  //      Bits= 3

  //  0 : "NO_FILTERING_IMU"
  //  1 : "LIGHT_FILTERING_IMU"
  //  2 : "MEDIUM_FILTERING_IMU"
  //  3 : "HEAVY_FILTERING_IMU"
  uint8_t IMU_Filtering;                       //      Bits= 3

  //  0 : "NO_FILTERING_VEL"
  //  1 : "LIGHT_FILTERING_VEL"
  //  2 : "MEDIUM_FILTERING_VEL"
  //  3 : "HEAVY_FILTERING_VEL"
  uint8_t Velocity_Filtering;                  //      Bits= 3

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} Settings_t;

// def @Velocities CAN Message (4    0x4)
#define Velocities_IDE (0U)
#define Velocities_DLC (8U)
#define Velocities_CANID (0x4U)
// signal: @Velocity_x_ro
#define HYTECH_Velocity_x_ro_CovFactor (0.02)
#define HYTECH_Velocity_x_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.02)) )
#define HYTECH_Velocity_x_ro_fromS(x) ( (((x) * (0.02)) + (0.0)) )
// signal: @Velocity_y_ro
#define HYTECH_Velocity_y_ro_CovFactor (0.03)
#define HYTECH_Velocity_y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.03)) )
#define HYTECH_Velocity_y_ro_fromS(x) ( (((x) * (0.03)) + (0.0)) )
// signal: @Velocity_z_ro
#define HYTECH_Velocity_z_ro_CovFactor (0.025)
#define HYTECH_Velocity_z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.025)) )
#define HYTECH_Velocity_z_ro_fromS(x) ( (((x) * (0.025)) + (0.0)) )
// signal: @Velocity_x_CGCorrected_ro
#define HYTECH_Velocity_x_CGCorrected_ro_CovFactor (0.02)
#define HYTECH_Velocity_x_CGCorrected_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.02)) )
#define HYTECH_Velocity_x_CGCorrected_ro_fromS(x) ( (((x) * (0.02)) + (0.0)) )
// signal: @Velocity_y_CGCorrected_ro
#define HYTECH_Velocity_y_CGCorrected_ro_CovFactor (0.03)
#define HYTECH_Velocity_y_CGCorrected_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.03)) )
#define HYTECH_Velocity_y_CGCorrected_ro_fromS(x) ( (((x) * (0.03)) + (0.0)) )
// signal: @Velocity_z_CGCorrected_ro
#define HYTECH_Velocity_z_CGCorrected_ro_CovFactor (0.025)
#define HYTECH_Velocity_z_CGCorrected_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.025)) )
#define HYTECH_Velocity_z_CGCorrected_ro_fromS(x) ( (((x) * (0.025)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t Velocity_x_ro;                     //  [-] Bits=12 Factor= 0.02            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_y_ro;                     //  [-] Bits=10 Factor= 0.03            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_z_ro;                     //  [-] Bits= 9 Factor= 0.025           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_z_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_x_CGCorrected_ro;         //  [-] Bits=12 Factor= 0.02            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_x_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_y_CGCorrected_ro;         //  [-] Bits=10 Factor= 0.03            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_y_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_z_CGCorrected_ro;         //  [-] Bits= 9 Factor= 0.025           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_z_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t Velocity_x_ro;                     //  [-] Bits=12 Factor= 0.02            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_y_ro;                     //  [-] Bits=10 Factor= 0.03            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_z_ro;                     //  [-] Bits= 9 Factor= 0.025           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_z_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_x_CGCorrected_ro;         //  [-] Bits=12 Factor= 0.02            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_x_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_y_CGCorrected_ro;         //  [-] Bits=10 Factor= 0.03            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_y_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Velocity_z_CGCorrected_ro;         //  [-] Bits= 9 Factor= 0.025           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Velocity_z_CGCorrected_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} Velocities_t;

// def @Attitude CAN Message (5    0x5)
#define Attitude_IDE (0U)
#define Attitude_DLC (6U)
#define Attitude_CANID (0x5U)
// signal: @Height_ro
#define HYTECH_Height_ro_CovFactor (0.035)
#define HYTECH_Height_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.035)) )
#define HYTECH_Height_ro_fromS(x) ( (((x) * (0.035)) + (0.0)) )
// signal: @Pitch_angle_ro
#define HYTECH_Pitch_angle_ro_CovFactor (0.25)
#define HYTECH_Pitch_angle_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.25)) )
#define HYTECH_Pitch_angle_ro_fromS(x) ( (((x) * (0.25)) + (0.0)) )
// signal: @Roll_angle_ro
#define HYTECH_Roll_angle_ro_CovFactor (0.25)
#define HYTECH_Roll_angle_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.25)) )
#define HYTECH_Roll_angle_ro_fromS(x) ( (((x) * (0.25)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t Height_ro;                         //  [-] Bits=12 Factor= 0.035           Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Pitch_angle_ro;                    //  [-] Bits= 9 Factor= 0.25            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Pitch_angle_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Roll_angle_ro;                     //  [-] Bits= 9 Factor= 0.25            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Roll_angle_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t Height_ro;                         //  [-] Bits=12 Factor= 0.035           Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Pitch_angle_ro;                    //  [-] Bits= 9 Factor= 0.25            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Pitch_angle_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Roll_angle_ro;                     //  [-] Bits= 9 Factor= 0.25            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Roll_angle_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} Attitude_t;

// def @Configuration CAN Message (6    0x6)
#define Configuration_IDE (0U)
#define Configuration_DLC (6U)
#define Configuration_CANID (0x6U)

// Value tables for @Attitude_Filtering signal

#ifndef Attitude_Filtering_Configuration_NO_FILTERING_ATT
#define Attitude_Filtering_Configuration_NO_FILTERING_ATT (0)
#endif

#ifndef Attitude_Filtering_Configuration_LIGHT_FILTERING_ATT
#define Attitude_Filtering_Configuration_LIGHT_FILTERING_ATT (1)
#endif

#ifndef Attitude_Filtering_Configuration_MEDIUM_FILTERING_ATT
#define Attitude_Filtering_Configuration_MEDIUM_FILTERING_ATT (2)
#endif

#ifndef Attitude_Filtering_Configuration_HEAVY_FILTERING_ATT
#define Attitude_Filtering_Configuration_HEAVY_FILTERING_ATT (3)
#endif


// Value tables for @IMU_Filtering signal

#ifndef IMU_Filtering_Configuration_NO_FILTERING_IMU
#define IMU_Filtering_Configuration_NO_FILTERING_IMU (0)
#endif

#ifndef IMU_Filtering_Configuration_LIGHT_FILTERING_IMU
#define IMU_Filtering_Configuration_LIGHT_FILTERING_IMU (1)
#endif

#ifndef IMU_Filtering_Configuration_MEDIUM_FILTERING_IMU
#define IMU_Filtering_Configuration_MEDIUM_FILTERING_IMU (2)
#endif

#ifndef IMU_Filtering_Configuration_HEAVY_FILTERING_IMU
#define IMU_Filtering_Configuration_HEAVY_FILTERING_IMU (3)
#endif


// Value tables for @CAN_Termination_State_Bus_1 signal

#ifndef CAN_Termination_State_Bus_1_Configuration_CAN_BUS_UNTERMINATED_CAN1_STATUS
#define CAN_Termination_State_Bus_1_Configuration_CAN_BUS_UNTERMINATED_CAN1_STATUS (0)
#endif

#ifndef CAN_Termination_State_Bus_1_Configuration_CAN_BUS_TERMINATED_CAN1_STATUS
#define CAN_Termination_State_Bus_1_Configuration_CAN_BUS_TERMINATED_CAN1_STATUS (1)
#endif


// Value tables for @CAN_Termination_State_Bus_2 signal

#ifndef CAN_Termination_State_Bus_2_Configuration_CAN_BUS_UNTERMINATED_CAN2_STATUS
#define CAN_Termination_State_Bus_2_Configuration_CAN_BUS_UNTERMINATED_CAN2_STATUS (0)
#endif

#ifndef CAN_Termination_State_Bus_2_Configuration_CAN_BUS_TERMINATED_CAN2_STATUS
#define CAN_Termination_State_Bus_2_Configuration_CAN_BUS_TERMINATED_CAN2_STATUS (1)
#endif


// Value tables for @Velocity_Filtering signal

#ifndef Velocity_Filtering_Configuration_NO_FILTERING_VEL
#define Velocity_Filtering_Configuration_NO_FILTERING_VEL (0)
#endif

#ifndef Velocity_Filtering_Configuration_LIGHT_FILTERING_VEL
#define Velocity_Filtering_Configuration_LIGHT_FILTERING_VEL (1)
#endif

#ifndef Velocity_Filtering_Configuration_MEDIUM_FILTERING_VEL
#define Velocity_Filtering_Configuration_MEDIUM_FILTERING_VEL (2)
#endif

#ifndef Velocity_Filtering_Configuration_HEAVY_FILTERING_VEL
#define Velocity_Filtering_Configuration_HEAVY_FILTERING_VEL (3)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int8_t Height_Offset_ro;                   //  [-] Bits= 8 Factor= 0.5             Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_Offset_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_Y_Location_ro;              //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_Y_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_X_Location_ro;              //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_X_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  //  0 : "NO_FILTERING_ATT"
  //  1 : "LIGHT_FILTERING_ATT"
  //  2 : "MEDIUM_FILTERING_ATT"
  //  3 : "HEAVY_FILTERING_ATT"
  uint8_t Attitude_Filtering : 3;            //      Bits= 3

  //  0 : "NO_FILTERING_IMU"
  //  1 : "LIGHT_FILTERING_IMU"
  //  2 : "MEDIUM_FILTERING_IMU"
  //  3 : "HEAVY_FILTERING_IMU"
  uint8_t IMU_Filtering : 3;                 //      Bits= 3

  //  0 : "CAN_BUS_UNTERMINATED_CAN1_STATUS"
  //  1 : "CAN_BUS_TERMINATED_CAN1_STATUS"
  uint8_t CAN_Termination_State_Bus_1 : 1;   //      Bits= 1

  //  0 : "CAN_BUS_UNTERMINATED_CAN2_STATUS"
  //  1 : "CAN_BUS_TERMINATED_CAN2_STATUS"
  uint8_t CAN_Termination_State_Bus_2 : 1;   //      Bits= 1

  //  0 : "NO_FILTERING_VEL"
  //  1 : "LIGHT_FILTERING_VEL"
  //  2 : "MEDIUM_FILTERING_VEL"
  //  3 : "HEAVY_FILTERING_VEL"
  uint8_t Velocity_Filtering : 3;            //      Bits= 3

#else

  int8_t Height_Offset_ro;                   //  [-] Bits= 8 Factor= 0.5             Unit:'cm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Height_Offset_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_Y_Location_ro;              //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_Y_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t Sensor_X_Location_ro;              //  [-] Bits=12 Factor= 0.01            Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t Sensor_X_Location_phys;
#endif // HYTECH_USE_SIGFLOAT

  //  0 : "NO_FILTERING_ATT"
  //  1 : "LIGHT_FILTERING_ATT"
  //  2 : "MEDIUM_FILTERING_ATT"
  //  3 : "HEAVY_FILTERING_ATT"
  uint8_t Attitude_Filtering;                //      Bits= 3

  //  0 : "NO_FILTERING_IMU"
  //  1 : "LIGHT_FILTERING_IMU"
  //  2 : "MEDIUM_FILTERING_IMU"
  //  3 : "HEAVY_FILTERING_IMU"
  uint8_t IMU_Filtering;                     //      Bits= 3

  //  0 : "CAN_BUS_UNTERMINATED_CAN1_STATUS"
  //  1 : "CAN_BUS_TERMINATED_CAN1_STATUS"
  uint8_t CAN_Termination_State_Bus_1;       //      Bits= 1

  //  0 : "CAN_BUS_UNTERMINATED_CAN2_STATUS"
  //  1 : "CAN_BUS_TERMINATED_CAN2_STATUS"
  uint8_t CAN_Termination_State_Bus_2;       //      Bits= 1

  //  0 : "NO_FILTERING_VEL"
  //  1 : "LIGHT_FILTERING_VEL"
  //  2 : "MEDIUM_FILTERING_VEL"
  //  3 : "HEAVY_FILTERING_VEL"
  uint8_t Velocity_Filtering;                //      Bits= 3

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} Configuration_t;

// def @INV1_STATUS CAN Message (112  0x70)
#define INV1_STATUS_IDE (0U)
#define INV1_STATUS_DLC (8U)
#define INV1_STATUS_CANID (0x70U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t system_ready : 1;                  //      Bits= 1

  uint8_t error : 1;                         //      Bits= 1

  uint8_t warning : 1;                       //      Bits= 1

  uint8_t quit_dc_on : 1;                    //      Bits= 1

  uint8_t dc_on : 1;                         //      Bits= 1

  uint8_t quit_inverter_on : 1;              //      Bits= 1

  uint8_t inverter_on : 1;                   //      Bits= 1

  uint8_t derating_on : 1;                   //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#else

  uint8_t system_ready;                      //      Bits= 1

  uint8_t error;                             //      Bits= 1

  uint8_t warning;                           //      Bits= 1

  uint8_t quit_dc_on;                        //      Bits= 1

  uint8_t dc_on;                             //      Bits= 1

  uint8_t quit_inverter_on;                  //      Bits= 1

  uint8_t inverter_on;                       //      Bits= 1

  uint8_t derating_on;                       //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_STATUS_t;

// def @INV1_TEMPS CAN Message (113  0x71)
#define INV1_TEMPS_IDE (0U)
#define INV1_TEMPS_DLC (6U)
#define INV1_TEMPS_CANID (0x71U)
// signal: @motor_temp_ro
#define HYTECH_motor_temp_ro_CovFactor (0.1)
#define HYTECH_motor_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_motor_temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @inverter_temp_ro
#define HYTECH_inverter_temp_ro_CovFactor (0.1)
#define HYTECH_inverter_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_inverter_temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @igbt_temp_ro
#define HYTECH_igbt_temp_ro_CovFactor (0.1)
#define HYTECH_igbt_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_igbt_temp_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_TEMPS_t;

// def @INV1_DYNAMICS CAN Message (114  0x72)
#define INV1_DYNAMICS_IDE (0U)
#define INV1_DYNAMICS_DLC (8U)
#define INV1_DYNAMICS_CANID (0x72U)
// signal: @actual_torque_nm_ro
#define HYTECH_actual_torque_nm_ro_CovFactor (0.0098)
#define HYTECH_actual_torque_nm_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0098)) )
#define HYTECH_actual_torque_nm_ro_fromS(x) ( (((x) * (0.0098)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#else

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_DYNAMICS_t;

// def @INV1_POWER CAN Message (115  0x73)
#define INV1_POWER_IDE (0U)
#define INV1_POWER_DLC (8U)
#define INV1_POWER_CANID (0x73U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#else

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_POWER_t;

// def @INV1_FEEDBACK CAN Message (116  0x74)
#define INV1_FEEDBACK_IDE (0U)
#define INV1_FEEDBACK_DLC (8U)
#define INV1_FEEDBACK_CANID (0x74U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_FEEDBACK_t;

// def @INV2_STATUS CAN Message (117  0x75)
#define INV2_STATUS_IDE (0U)
#define INV2_STATUS_DLC (8U)
#define INV2_STATUS_CANID (0x75U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t system_ready : 1;                  //      Bits= 1

  uint8_t error : 1;                         //      Bits= 1

  uint8_t warning : 1;                       //      Bits= 1

  uint8_t quit_dc_on : 1;                    //      Bits= 1

  uint8_t dc_on : 1;                         //      Bits= 1

  uint8_t quit_inverter_on : 1;              //      Bits= 1

  uint8_t inverter_on : 1;                   //      Bits= 1

  uint8_t derating_on : 1;                   //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#else

  uint8_t system_ready;                      //      Bits= 1

  uint8_t error;                             //      Bits= 1

  uint8_t warning;                           //      Bits= 1

  uint8_t quit_dc_on;                        //      Bits= 1

  uint8_t dc_on;                             //      Bits= 1

  uint8_t quit_inverter_on;                  //      Bits= 1

  uint8_t inverter_on;                       //      Bits= 1

  uint8_t derating_on;                       //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_STATUS_t;

// def @INV2_TEMPS CAN Message (118  0x76)
#define INV2_TEMPS_IDE (0U)
#define INV2_TEMPS_DLC (6U)
#define INV2_TEMPS_CANID (0x76U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_TEMPS_t;

// def @INV2_DYNAMICS CAN Message (119  0x77)
#define INV2_DYNAMICS_IDE (0U)
#define INV2_DYNAMICS_DLC (8U)
#define INV2_DYNAMICS_CANID (0x77U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#else

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_DYNAMICS_t;

// def @INV2_POWER CAN Message (120  0x78)
#define INV2_POWER_IDE (0U)
#define INV2_POWER_DLC (8U)
#define INV2_POWER_CANID (0x78U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#else

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_POWER_t;

// def @INV2_FEEDBACK CAN Message (121  0x79)
#define INV2_FEEDBACK_IDE (0U)
#define INV2_FEEDBACK_DLC (8U)
#define INV2_FEEDBACK_CANID (0x79U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_FEEDBACK_t;

// def @INV3_STATUS CAN Message (128  0x80)
#define INV3_STATUS_IDE (0U)
#define INV3_STATUS_DLC (8U)
#define INV3_STATUS_CANID (0x80U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t system_ready : 1;                  //      Bits= 1

  uint8_t error : 1;                         //      Bits= 1

  uint8_t warning : 1;                       //      Bits= 1

  uint8_t quit_dc_on : 1;                    //      Bits= 1

  uint8_t dc_on : 1;                         //      Bits= 1

  uint8_t quit_inverter_on : 1;              //      Bits= 1

  uint8_t inverter_on : 1;                   //      Bits= 1

  uint8_t derating_on : 1;                   //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#else

  uint8_t system_ready;                      //      Bits= 1

  uint8_t error;                             //      Bits= 1

  uint8_t warning;                           //      Bits= 1

  uint8_t quit_dc_on;                        //      Bits= 1

  uint8_t dc_on;                             //      Bits= 1

  uint8_t quit_inverter_on;                  //      Bits= 1

  uint8_t inverter_on;                       //      Bits= 1

  uint8_t derating_on;                       //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_STATUS_t;

// def @INV3_TEMPS CAN Message (129  0x81)
#define INV3_TEMPS_IDE (0U)
#define INV3_TEMPS_DLC (6U)
#define INV3_TEMPS_CANID (0x81U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_TEMPS_t;

// def @INV3_DYNAMICS CAN Message (130  0x82)
#define INV3_DYNAMICS_IDE (0U)
#define INV3_DYNAMICS_DLC (8U)
#define INV3_DYNAMICS_CANID (0x82U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#else

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_DYNAMICS_t;

// def @INV3_POWER CAN Message (131  0x83)
#define INV3_POWER_IDE (0U)
#define INV3_POWER_DLC (8U)
#define INV3_POWER_CANID (0x83U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#else

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_POWER_t;

// def @INV3_FEEDBACK CAN Message (132  0x84)
#define INV3_FEEDBACK_IDE (0U)
#define INV3_FEEDBACK_DLC (8U)
#define INV3_FEEDBACK_CANID (0x84U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_FEEDBACK_t;

// def @INV4_STATUS CAN Message (133  0x85)
#define INV4_STATUS_IDE (0U)
#define INV4_STATUS_DLC (8U)
#define INV4_STATUS_CANID (0x85U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t system_ready : 1;                  //      Bits= 1

  uint8_t error : 1;                         //      Bits= 1

  uint8_t warning : 1;                       //      Bits= 1

  uint8_t quit_dc_on : 1;                    //      Bits= 1

  uint8_t dc_on : 1;                         //      Bits= 1

  uint8_t quit_inverter_on : 1;              //      Bits= 1

  uint8_t inverter_on : 1;                   //      Bits= 1

  uint8_t derating_on : 1;                   //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#else

  uint8_t system_ready;                      //      Bits= 1

  uint8_t error;                             //      Bits= 1

  uint8_t warning;                           //      Bits= 1

  uint8_t quit_dc_on;                        //      Bits= 1

  uint8_t dc_on;                             //      Bits= 1

  uint8_t quit_inverter_on;                  //      Bits= 1

  uint8_t inverter_on;                       //      Bits= 1

  uint8_t derating_on;                       //      Bits= 1

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  uint16_t diagnostic_number;                //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_STATUS_t;

// def @INV4_TEMPS CAN Message (134  0x86)
#define INV4_TEMPS_IDE (0U)
#define INV4_TEMPS_DLC (6U)
#define INV4_TEMPS_CANID (0x86U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t motor_temp_ro;                     //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t inverter_temp_ro;                  //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t igbt_temp_ro;                      //  [-] Bits=16 Factor= 0.1             Unit:'C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t igbt_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_TEMPS_t;

// def @INV4_DYNAMICS CAN Message (135  0x87)
#define INV4_DYNAMICS_IDE (0U)
#define INV4_DYNAMICS_DLC (8U)
#define INV4_DYNAMICS_CANID (0x87U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#else

  uint32_t actual_power_w;                   //      Bits=32

  int16_t actual_torque_nm_ro;               //  [-] Bits=16 Factor= 0.0098

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t actual_torque_nm_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t actual_speed_rpm;                  //  [-] Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_DYNAMICS_t;

// def @INV4_POWER CAN Message (136  0x88)
#define INV4_POWER_IDE (0U)
#define INV4_POWER_DLC (8U)
#define INV4_POWER_CANID (0x88U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#else

  int32_t active_power_w;                    //  [-] Bits=32

  int32_t reactive_power_var;                //  [-] Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_POWER_t;

// def @INV4_FEEDBACK CAN Message (137  0x89)
#define INV4_FEEDBACK_IDE (0U)
#define INV4_FEEDBACK_DLC (8U)
#define INV4_FEEDBACK_CANID (0x89U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_FEEDBACK_t;

// def @INV1_CONTROL_WORD CAN Message (144  0x90)
#define INV1_CONTROL_WORD_IDE (0U)
#define INV1_CONTROL_WORD_DLC (2U)
#define INV1_CONTROL_WORD_CANID (0x90U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t inverter_enable : 1;               //      Bits= 1

  uint8_t hv_enable : 1;                     //      Bits= 1

  uint8_t driver_enable : 1;                 //      Bits= 1

  uint8_t remove_error : 1;                  //      Bits= 1

#else

  uint8_t inverter_enable;                   //      Bits= 1

  uint8_t hv_enable;                         //      Bits= 1

  uint8_t driver_enable;                     //      Bits= 1

  uint8_t remove_error;                      //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_CONTROL_WORD_t;

// def @INV2_CONTROL_WORD CAN Message (145  0x91)
#define INV2_CONTROL_WORD_IDE (0U)
#define INV2_CONTROL_WORD_DLC (2U)
#define INV2_CONTROL_WORD_CANID (0x91U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t inverter_enable : 1;               //      Bits= 1

  uint8_t hv_enable : 1;                     //      Bits= 1

  uint8_t driver_enable : 1;                 //      Bits= 1

  uint8_t remove_error : 1;                  //      Bits= 1

#else

  uint8_t inverter_enable;                   //      Bits= 1

  uint8_t hv_enable;                         //      Bits= 1

  uint8_t driver_enable;                     //      Bits= 1

  uint8_t remove_error;                      //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_CONTROL_WORD_t;

// def @INV3_CONTROL_WORD CAN Message (146  0x92)
#define INV3_CONTROL_WORD_IDE (0U)
#define INV3_CONTROL_WORD_DLC (2U)
#define INV3_CONTROL_WORD_CANID (0x92U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t inverter_enable : 1;               //      Bits= 1

  uint8_t hv_enable : 1;                     //      Bits= 1

  uint8_t driver_enable : 1;                 //      Bits= 1

  uint8_t remove_error : 1;                  //      Bits= 1

#else

  uint8_t inverter_enable;                   //      Bits= 1

  uint8_t hv_enable;                         //      Bits= 1

  uint8_t driver_enable;                     //      Bits= 1

  uint8_t remove_error;                      //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_CONTROL_WORD_t;

// def @INV4_CONTROL_WORD CAN Message (147  0x93)
#define INV4_CONTROL_WORD_IDE (0U)
#define INV4_CONTROL_WORD_DLC (2U)
#define INV4_CONTROL_WORD_CANID (0x93U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t inverter_enable : 1;               //      Bits= 1

  uint8_t hv_enable : 1;                     //      Bits= 1

  uint8_t driver_enable : 1;                 //      Bits= 1

  uint8_t remove_error : 1;                  //      Bits= 1

#else

  uint8_t inverter_enable;                   //      Bits= 1

  uint8_t hv_enable;                         //      Bits= 1

  uint8_t driver_enable;                     //      Bits= 1

  uint8_t remove_error;                      //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_CONTROL_WORD_t;

// TCU_DRIVER_MSG contains a signal that corresponds to a preset list of messages on the dashboard. It also includes two variables to customize the contents of the message with numerical values. It also includes the target lap time which is determined by the
// def @TCU_DRIVER_MSG CAN Message (148  0x94)
#define TCU_DRIVER_MSG_IDE (0U)
#define TCU_DRIVER_MSG_DLC (8U)
#define TCU_DRIVER_MSG_CANID (0x94U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // corresponds to a pre-set list of driver messages on the dashboard that the TCU can request to be displayed in event of comms failure
  uint8_t driver_message;                    //      Bits= 8

  uint16_t driver_msg_var_1;                 //      Bits=16

  uint16_t driver_msg_var_2;                 //      Bits=16

  // target lap time is the time determined by the pit crew that the driver should aim for. mostly useful for guiding the driver in endurance races.
  uint32_t target_lap_time;                  //      Bits=24 Unit:'ms'

#else

  // corresponds to a pre-set list of driver messages on the dashboard that the TCU can request to be displayed in event of comms failure
  uint8_t driver_message;                    //      Bits= 8

  uint16_t driver_msg_var_1;                 //      Bits=16

  uint16_t driver_msg_var_2;                 //      Bits=16

  // target lap time is the time determined by the pit crew that the driver should aim for. mostly useful for guiding the driver in endurance races.
  uint32_t target_lap_time;                  //      Bits=24 Unit:'ms'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TCU_DRIVER_MSG_t;

// The TCU_LAP_TIMES message contains the best and previous lap times to the dashboard as calculated by the TCU or manually input by the pit crew. It also relays the current state of the TCU's clock to the dashboard so that it can run an approximate stopwatc
// def @TCU_LAP_TIMES CAN Message (149  0x95)
#define TCU_LAP_TIMES_IDE (0U)
#define TCU_LAP_TIMES_DLC (8U)
#define TCU_LAP_TIMES_CANID (0x95U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t best_lap_time;                    //      Bits=24 Unit:'ms'

  uint32_t prev_lap_time;                    //      Bits=24 Unit:'ms'

  uint8_t lap_clock_state;                   //      Bits= 8

#else

  uint32_t best_lap_time;                    //      Bits=24 Unit:'ms'

  uint32_t prev_lap_time;                    //      Bits=24 Unit:'ms'

  uint8_t lap_clock_state;                   //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TCU_LAP_TIMES_t;

// def @INV1_CONTROL_INPUT CAN Message (151  0x97)
#define INV1_CONTROL_INPUT_IDE (0U)
#define INV1_CONTROL_INPUT_DLC (6U)
#define INV1_CONTROL_INPUT_CANID (0x97U)
// signal: @positive_torque_limit_ro
#define HYTECH_positive_torque_limit_ro_CovFactor (0.0098)
#define HYTECH_positive_torque_limit_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0098)) )
#define HYTECH_positive_torque_limit_ro_fromS(x) ( (((x) * (0.0098)) + (0.0)) )
// signal: @negative_torque_limit_ro
#define HYTECH_negative_torque_limit_ro_CovFactor (0.0098)
#define HYTECH_negative_torque_limit_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0098)) )
#define HYTECH_negative_torque_limit_ro_fromS(x) ( (((x) * (0.0098)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_CONTROL_INPUT_t;

// def @INV2_CONTROL_INPUT CAN Message (152  0x98)
#define INV2_CONTROL_INPUT_IDE (0U)
#define INV2_CONTROL_INPUT_DLC (6U)
#define INV2_CONTROL_INPUT_CANID (0x98U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_CONTROL_INPUT_t;

// def @INV3_CONTROL_INPUT CAN Message (153  0x99)
#define INV3_CONTROL_INPUT_IDE (0U)
#define INV3_CONTROL_INPUT_DLC (6U)
#define INV3_CONTROL_INPUT_CANID (0x99U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_CONTROL_INPUT_t;

// def @MC1_ENERGY CAN Message (184  0xb8)
#define MC1_ENERGY_IDE (0U)
#define MC1_ENERGY_DLC (8U)
#define MC1_ENERGY_CANID (0xb8U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  // Made up units from AMK
  uint32_t motor_power;                      //      Bits=32

  // random made up AMK units
  int16_t feedback_torque;                   //  [-] Bits=16 Unit:'Mn'

#else

  uint16_t dc_bus_voltage;                   //      Bits=16 Unit:'V'

  // Made up units from AMK
  uint32_t motor_power;                      //      Bits=32

  // random made up AMK units
  int16_t feedback_torque;                   //  [-] Bits=16 Unit:'Mn'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MC1_ENERGY_t;

// def @PEDALS_SYSTEM_DATA CAN Message (192  0xc0)
#define PEDALS_SYSTEM_DATA_IDE (0U)
#define PEDALS_SYSTEM_DATA_DLC (5U)
#define PEDALS_SYSTEM_DATA_CANID (0xc0U)
// signal: @accel_pedal_ro
#define HYTECH_accel_pedal_ro_CovFactor (0.000015259)
#define HYTECH_accel_pedal_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.000015259)) )
#define HYTECH_accel_pedal_ro_fromS(x) ( (((x) * (0.000015259)) + (0.0)) )
// signal: @brake_pedal_ro
#define HYTECH_brake_pedal_ro_CovFactor (0.000015259)
#define HYTECH_brake_pedal_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.000015259)) )
#define HYTECH_brake_pedal_ro_fromS(x) ( (((x) * (0.000015259)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t accel_implausible : 1;               //      Bits= 1

  uint8_t brake_implausible : 1;               //      Bits= 1

  uint8_t brake_pedal_active : 1;              //      Bits= 1

  uint8_t accel_pedal_active : 1;              //      Bits= 1

  uint8_t mechanical_brake_active : 1;         //      Bits= 1

  // brake and accel are pressed
  uint8_t brake_accel_implausibility : 1;      //      Bits= 1

  // a pedal implausibility has been present for longer than allowed
  uint8_t implaus_exceeded_max_duration : 1;   //      Bits= 1

  uint16_t accel_pedal_ro;                     //      Bits=16 Factor= 0.000015259

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t accel_pedal_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_pedal_ro;                     //      Bits=16 Factor= 0.000015259

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_pedal_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t accel_implausible;                   //      Bits= 1

  uint8_t brake_implausible;                   //      Bits= 1

  uint8_t brake_pedal_active;                  //      Bits= 1

  uint8_t accel_pedal_active;                  //      Bits= 1

  uint8_t mechanical_brake_active;             //      Bits= 1

  // brake and accel are pressed
  uint8_t brake_accel_implausibility;          //      Bits= 1

  // a pedal implausibility has been present for longer than allowed
  uint8_t implaus_exceeded_max_duration;       //      Bits= 1

  uint16_t accel_pedal_ro;                     //      Bits=16 Factor= 0.000015259

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t accel_pedal_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_pedal_ro;                     //      Bits=16 Factor= 0.000015259

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_pedal_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} PEDALS_SYSTEM_DATA_t;

// def @MCU_STATUS CAN Message (195  0xc3)
#define MCU_STATUS_IDE (0U)
#define MCU_STATUS_DLC (8U)
#define MCU_STATUS_CANID (0xc3U)
// signal: @distance_traveled_m_ro
#define HYTECH_distance_traveled_m_ro_CovFactor (0.01)
#define HYTECH_distance_traveled_m_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_distance_traveled_m_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t imd_ok_high : 1;                        //      Bits= 1

  uint8_t shutdown_b_above_threshold : 1;         //      Bits= 1

  uint8_t bms_ok_high : 1;                        //      Bits= 1

  uint8_t shutdown_c_above_threshold : 1;         //      Bits= 1

  uint8_t bspd_ok_high : 1;                       //      Bits= 1

  uint8_t shutdown_d_above_threshold : 1;         //      Bits= 1

  uint8_t software_ok_high : 1;                   //      Bits= 1

  uint8_t shutdown_e_above_threshold : 1;         //      Bits= 1

  uint8_t mechanical_brake_active : 1;            //      Bits= 1

  uint8_t no_accel_implausibility : 1;            //      Bits= 1

  uint8_t no_brake_implausibility : 1;            //      Bits= 1

  uint8_t brake_pedal_active : 1;                 //      Bits= 1

  uint8_t bspd_current_high : 1;                  //      Bits= 1

  uint8_t bspd_brake_high : 1;                    //      Bits= 1

  uint8_t no_accel_or_brake_implausibility : 1;   //      Bits= 1

  uint8_t drive_buzzer : 1;                       //      Bits= 1

  uint8_t ecu_state : 3;                          //      Bits= 3

  uint8_t inverter_error : 1;                     //      Bits= 1

  uint8_t energy_meter_present : 1;               //      Bits= 1

  uint8_t software_ok : 1;                        //      Bits= 1

  uint8_t launch_control_active : 1;              //      Bits= 1

  uint8_t pack_charge_critical : 2;               //      Bits= 2

  // The current drive mode on the ECU irrespective of dial mapping
  uint8_t drive_mode : 4;                         //      Bits= 4

  uint8_t max_torque;                             //      Bits= 8

  // torque mode
  uint8_t torque_mode;                            //      Bits= 8

  uint16_t distance_traveled_m_ro;                //      Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t distance_traveled_m_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t imd_ok_high;                            //      Bits= 1

  uint8_t shutdown_b_above_threshold;             //      Bits= 1

  uint8_t bms_ok_high;                            //      Bits= 1

  uint8_t shutdown_c_above_threshold;             //      Bits= 1

  uint8_t bspd_ok_high;                           //      Bits= 1

  uint8_t shutdown_d_above_threshold;             //      Bits= 1

  uint8_t software_ok_high;                       //      Bits= 1

  uint8_t shutdown_e_above_threshold;             //      Bits= 1

  uint8_t mechanical_brake_active;                //      Bits= 1

  uint8_t no_accel_implausibility;                //      Bits= 1

  uint8_t no_brake_implausibility;                //      Bits= 1

  uint8_t brake_pedal_active;                     //      Bits= 1

  uint8_t bspd_current_high;                      //      Bits= 1

  uint8_t bspd_brake_high;                        //      Bits= 1

  uint8_t no_accel_or_brake_implausibility;       //      Bits= 1

  uint8_t drive_buzzer;                           //      Bits= 1

  uint8_t ecu_state;                              //      Bits= 3

  uint8_t inverter_error;                         //      Bits= 1

  uint8_t energy_meter_present;                   //      Bits= 1

  uint8_t software_ok;                            //      Bits= 1

  uint8_t launch_control_active;                  //      Bits= 1

  uint8_t pack_charge_critical;                   //      Bits= 2

  // The current drive mode on the ECU irrespective of dial mapping
  uint8_t drive_mode;                             //      Bits= 4

  uint8_t max_torque;                             //      Bits= 8

  // torque mode
  uint8_t torque_mode;                            //      Bits= 8

  uint16_t distance_traveled_m_ro;                //      Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t distance_traveled_m_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_STATUS_t;

// def @MCU_LOAD_CELLS CAN Message (197  0xc5)
#define MCU_LOAD_CELLS_IDE (0U)
#define MCU_LOAD_CELLS_DLC (4U)
#define MCU_LOAD_CELLS_CANID (0xc5U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t load_cell_fl;                     //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fr;                     //      Bits=16 Unit:'lbs'

#else

  uint16_t load_cell_fl;                     //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fr;                     //      Bits=16 Unit:'lbs'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_LOAD_CELLS_t;

// def @MCU_FRONT_POTS CAN Message (198  0xc6)
#define MCU_FRONT_POTS_IDE (0U)
#define MCU_FRONT_POTS_DLC (4U)
#define MCU_FRONT_POTS_CANID (0xc6U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t potentiometer_fl;                 //      Bits=16 Unit:'lbs'

  uint16_t potentiometer_fr;                 //      Bits=16 Unit:'lbs'

#else

  uint16_t potentiometer_fl;                 //      Bits=16 Unit:'lbs'

  uint16_t potentiometer_fr;                 //      Bits=16 Unit:'lbs'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_FRONT_POTS_t;

// def @MCU_SUSPENSION CAN Message (200  0xc8)
#define MCU_SUSPENSION_IDE (0U)
#define MCU_SUSPENSION_DLC (8U)
#define MCU_SUSPENSION_CANID (0xc8U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t potentiometer_fl;                 //      Bits=16 Unit:'lbs'

  uint16_t potentiometer_fr;                 //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fl;                     //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fr;                     //      Bits=16 Unit:'lbs'

#else

  uint16_t potentiometer_fl;                 //      Bits=16 Unit:'lbs'

  uint16_t potentiometer_fr;                 //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fl;                     //      Bits=16 Unit:'lbs'

  uint16_t load_cell_fr;                     //      Bits=16 Unit:'lbs'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_SUSPENSION_t;

// MCU's simple torque controller
// def @TC_SIMPLE CAN Message (201  0xc9)
#define TC_SIMPLE_IDE (0U)
#define TC_SIMPLE_DLC (8U)
#define TC_SIMPLE_CANID (0xc9U)
// signal: @torque_request_ro
#define HYTECH_torque_request_ro_CovFactor (0.001)
#define HYTECH_torque_request_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_torque_request_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @front_torque_scale_ro
#define HYTECH_front_torque_scale_ro_CovFactor (0.00787402)
#define HYTECH_front_torque_scale_ro_toS(x) ( (int8_t) (((x) - (0.0)) / (0.00787402)) )
#define HYTECH_front_torque_scale_ro_fromS(x) ( (((x) * (0.00787402)) + (0.0)) )
// signal: @rear_torque_scale_ro
#define HYTECH_rear_torque_scale_ro_CovFactor (0.00787402)
#define HYTECH_rear_torque_scale_ro_toS(x) ( (int8_t) (((x) - (0.0)) / (0.00787402)) )
#define HYTECH_rear_torque_scale_ro_fromS(x) ( (((x) * (0.00787402)) + (0.0)) )
// signal: @front_regen_scale_ro
#define HYTECH_front_regen_scale_ro_CovFactor (0.00787402)
#define HYTECH_front_regen_scale_ro_toS(x) ( (int8_t) (((x) - (0.0)) / (0.00787402)) )
#define HYTECH_front_regen_scale_ro_fromS(x) ( (((x) * (0.00787402)) + (0.0)) )
// signal: @rear_regen_scale_ro
#define HYTECH_rear_regen_scale_ro_CovFactor (0.00787402)
#define HYTECH_rear_regen_scale_ro_toS(x) ( (int8_t) (((x) - (0.0)) / (0.00787402)) )
#define HYTECH_rear_regen_scale_ro_fromS(x) ( (((x) * (0.00787402)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t torque_request_ro;                 //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t torque_request_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t front_torque_scale_ro;              //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t front_torque_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t rear_torque_scale_ro;               //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t rear_torque_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t front_regen_scale_ro;               //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t front_regen_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t rear_regen_scale_ro;                //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t rear_regen_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  // The current state of the acceleration request (0 = decelerating, 1 = accelerating)
  uint8_t accel_request_state : 1;           //      Bits= 1

#else

  int16_t torque_request_ro;                 //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t torque_request_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t front_torque_scale_ro;              //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t front_torque_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t rear_torque_scale_ro;               //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t rear_torque_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t front_regen_scale_ro;               //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t front_regen_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  int8_t rear_regen_scale_ro;                //  [-] Bits= 8 Factor= 0.00787402

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t rear_regen_scale_phys;
#endif // HYTECH_USE_SIGFLOAT

  // The current state of the acceleration request (0 = decelerating, 1 = accelerating)
  uint8_t accel_request_state;               //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TC_SIMPLE_t;

// def @TC_SIMPLE_LAUNCH CAN Message (202  0xca)
#define TC_SIMPLE_LAUNCH_IDE (0U)
#define TC_SIMPLE_LAUNCH_DLC (8U)
#define TC_SIMPLE_LAUNCH_CANID (0xcaU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // The current state of the launch controller (LAUNCH_NOT_READY, LAUNCH_READY, LAUNCHING)
  uint8_t launch_control_state : 2;          //      Bits= 2

  // State of whether the launch algorithm has taken over control from the initial launch target
  uint8_t algo_active : 1;                   //      Bits= 1

  // The initial launch speed target requested by the launch controller
  uint16_t initial_launch_target;            //      Bits=16 Unit:'rpm'

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

#else

  // The current state of the launch controller (LAUNCH_NOT_READY, LAUNCH_READY, LAUNCHING)
  uint8_t launch_control_state;              //      Bits= 2

  // State of whether the launch algorithm has taken over control from the initial launch target
  uint8_t algo_active;                       //      Bits= 1

  // The initial launch speed target requested by the launch controller
  uint16_t initial_launch_target;            //      Bits=16 Unit:'rpm'

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TC_SIMPLE_LAUNCH_t;

// def @BRAKE_PRESSURE_SENSOR CAN Message (203  0xcb)
#define BRAKE_PRESSURE_SENSOR_IDE (0U)
#define BRAKE_PRESSURE_SENSOR_DLC (2U)
#define BRAKE_PRESSURE_SENSOR_CANID (0xcbU)
// signal: @brake_sensor_analog_read_ro
#define HYTECH_brake_sensor_analog_read_ro_CovFactor (3.051757812)
#define HYTECH_brake_sensor_analog_read_ro_toS(x) ( (uint16_t) (((x) - (-312.5)) / (3.051757812)) )
#define HYTECH_brake_sensor_analog_read_ro_fromS(x) ( (((x) * (3.051757812)) + (-312.5)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_sensor_analog_read_ro;      //      Bits=16 Offset= -312.5             Factor= 3.051757812     Unit:'psi'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_sensor_analog_read_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_sensor_analog_read_ro;      //      Bits=16 Offset= -312.5             Factor= 3.051757812     Unit:'psi'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_sensor_analog_read_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BRAKE_PRESSURE_SENSOR_t;

// def @MCU_ANALOG_READINGS CAN Message (204  0xcc)
#define MCU_ANALOG_READINGS_IDE (0U)
#define MCU_ANALOG_READINGS_DLC (8U)
#define MCU_ANALOG_READINGS_CANID (0xccU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t steering_1;                        //  [-] Bits=16

  uint16_t steering_2;                       //      Bits=16

  int16_t hall_effect_current;               //  [-] Bits=16 Unit:'N/A'

  uint16_t glv_battery_voltage;              //      Bits=16

#else

  int16_t steering_1;                        //  [-] Bits=16

  uint16_t steering_2;                       //      Bits=16

  int16_t hall_effect_current;               //  [-] Bits=16 Unit:'N/A'

  uint16_t glv_battery_voltage;              //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_ANALOG_READINGS_t;

// def @MCU_PEDAL_RAW CAN Message (205  0xcd)
#define MCU_PEDAL_RAW_IDE (0U)
#define MCU_PEDAL_RAW_DLC (8U)
#define MCU_PEDAL_RAW_CANID (0xcdU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t accel_1_raw;                      //      Bits=12

  uint16_t accel_2_raw;                      //      Bits=12

  uint16_t brake_1_raw;                      //      Bits=12

  uint16_t brake_2_raw;                      //      Bits=12

#else

  uint16_t accel_1_raw;                      //      Bits=12

  uint16_t accel_2_raw;                      //      Bits=12

  uint16_t brake_1_raw;                      //      Bits=12

  uint16_t brake_2_raw;                      //      Bits=12

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_PEDAL_RAW_t;

// Vehicle body velocity measured by VN-300
// def @VN_VEL CAN Message (208  0xd0)
#define VN_VEL_IDE (0U)
#define VN_VEL_DLC (8U)
#define VN_VEL_CANID (0xd0U)
// signal: @vn_body_vel_x_ro
#define HYTECH_vn_body_vel_x_ro_CovFactor (0.002)
#define HYTECH_vn_body_vel_x_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.002)) )
#define HYTECH_vn_body_vel_x_ro_fromS(x) ( (((x) * (0.002)) + (0.0)) )
// signal: @vn_body_vel_y_ro
#define HYTECH_vn_body_vel_y_ro_CovFactor (0.002)
#define HYTECH_vn_body_vel_y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.002)) )
#define HYTECH_vn_body_vel_y_ro_fromS(x) ( (((x) * (0.002)) + (0.0)) )
// signal: @vn_body_vel_z_ro
#define HYTECH_vn_body_vel_z_ro_CovFactor (0.002)
#define HYTECH_vn_body_vel_z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.002)) )
#define HYTECH_vn_body_vel_z_ro_fromS(x) ( (((x) * (0.002)) + (0.0)) )
// signal: @vn_vel_uncertainty_ro
#define HYTECH_vn_vel_uncertainty_ro_CovFactor (0.0005)
#define HYTECH_vn_vel_uncertainty_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0005)) )
#define HYTECH_vn_vel_uncertainty_ro_fromS(x) ( (((x) * (0.0005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vn_body_vel_x_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_body_vel_y_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_body_vel_z_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t vn_vel_uncertainty_ro;            //      Bits=16 Factor= 0.0005          Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_vel_uncertainty_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vn_body_vel_x_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_body_vel_y_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_body_vel_z_ro;                  //  [-] Bits=16 Factor= 0.002           Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_body_vel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t vn_vel_uncertainty_ro;            //      Bits=16 Factor= 0.0005          Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_vel_uncertainty_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_VEL_t;

// def @VN_LINEAR_ACCEL CAN Message (209  0xd1)
#define VN_LINEAR_ACCEL_IDE (0U)
#define VN_LINEAR_ACCEL_DLC (8U)
#define VN_LINEAR_ACCEL_CANID (0xd1U)
// signal: @vn_lin_ins_accel_x_ro
#define HYTECH_vn_lin_ins_accel_x_ro_CovFactor (0.001)
#define HYTECH_vn_lin_ins_accel_x_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_ins_accel_x_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vn_lin_ins_accel_y_ro
#define HYTECH_vn_lin_ins_accel_y_ro_CovFactor (0.001)
#define HYTECH_vn_lin_ins_accel_y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_ins_accel_y_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vn_lin_ins_accel_z_ro
#define HYTECH_vn_lin_ins_accel_z_ro_CovFactor (0.001)
#define HYTECH_vn_lin_ins_accel_z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_ins_accel_z_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vn_lin_ins_accel_x_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_ins_accel_y_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_ins_accel_z_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vn_lin_ins_accel_x_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_ins_accel_y_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_ins_accel_z_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_ins_accel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_LINEAR_ACCEL_t;

// def @VN_LINEAR_ACCEL_UNCOMP CAN Message (210  0xd2)
#define VN_LINEAR_ACCEL_UNCOMP_IDE (0U)
#define VN_LINEAR_ACCEL_UNCOMP_DLC (8U)
#define VN_LINEAR_ACCEL_UNCOMP_CANID (0xd2U)
// signal: @vn_lin_uncomp_accel_x_ro
#define HYTECH_vn_lin_uncomp_accel_x_ro_CovFactor (0.001)
#define HYTECH_vn_lin_uncomp_accel_x_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_uncomp_accel_x_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vn_lin_uncomp_accel_y_ro
#define HYTECH_vn_lin_uncomp_accel_y_ro_CovFactor (0.001)
#define HYTECH_vn_lin_uncomp_accel_y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_uncomp_accel_y_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vn_lin_uncomp_accel_z_ro
#define HYTECH_vn_lin_uncomp_accel_z_ro_CovFactor (0.001)
#define HYTECH_vn_lin_uncomp_accel_z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vn_lin_uncomp_accel_z_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vn_lin_uncomp_accel_x_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_uncomp_accel_y_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_uncomp_accel_z_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vn_lin_uncomp_accel_x_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_uncomp_accel_y_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_lin_uncomp_accel_z_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'m/s^2'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_lin_uncomp_accel_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_LINEAR_ACCEL_UNCOMP_t;

// def @VN_YPR CAN Message (211  0xd3)
#define VN_YPR_IDE (0U)
#define VN_YPR_DLC (8U)
#define VN_YPR_CANID (0xd3U)
// signal: @vn_yaw_ro
#define HYTECH_vn_yaw_ro_CovFactor (0.01)
#define HYTECH_vn_yaw_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vn_yaw_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vn_pitch_ro
#define HYTECH_vn_pitch_ro_CovFactor (0.01)
#define HYTECH_vn_pitch_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vn_pitch_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vn_roll_ro
#define HYTECH_vn_roll_ro_CovFactor (0.01)
#define HYTECH_vn_roll_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vn_roll_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vn_yaw_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_yaw_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_pitch_ro;                       //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_pitch_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_roll_ro;                        //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_roll_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vn_yaw_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_yaw_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_pitch_ro;                       //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_pitch_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vn_roll_ro;                        //  [-] Bits=16 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_roll_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_YPR_t;

// def @VN_LAT_LON CAN Message (212  0xd4)
#define VN_LAT_LON_IDE (0U)
#define VN_LAT_LON_DLC (8U)
#define VN_LAT_LON_CANID (0xd4U)
// signal: @vn_gps_lat_ro
#define HYTECH_vn_gps_lat_ro_CovFactor (0.000001)
#define HYTECH_vn_gps_lat_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.000001)) )
#define HYTECH_vn_gps_lat_ro_fromS(x) ( (((x) * (0.000001)) + (0.0)) )
// signal: @vn_gps_lon_ro
#define HYTECH_vn_gps_lon_ro_CovFactor (0.000001)
#define HYTECH_vn_gps_lon_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.000001)) )
#define HYTECH_vn_gps_lon_ro_fromS(x) ( (((x) * (0.000001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t vn_gps_lat_ro;                     //  [-] Bits=32 Factor= 0.000001        Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_gps_lat_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t vn_gps_lon_ro;                     //  [-] Bits=32 Factor= 0.000001        Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_gps_lon_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int32_t vn_gps_lat_ro;                     //  [-] Bits=32 Factor= 0.000001        Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_gps_lat_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t vn_gps_lon_ro;                     //  [-] Bits=32 Factor= 0.000001        Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_gps_lon_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_LAT_LON_t;

// def @BMS_ONBOARD_TEMPS CAN Message (213  0xd5)
#define BMS_ONBOARD_TEMPS_IDE (0U)
#define BMS_ONBOARD_TEMPS_DLC (6U)
#define BMS_ONBOARD_TEMPS_CANID (0xd5U)
// signal: @max_board_temp_ro
#define HYTECH_max_board_temp_ro_CovFactor (0.01)
#define HYTECH_max_board_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_max_board_temp_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @low_cell_temp_ro
#define HYTECH_low_cell_temp_ro_CovFactor (0.01)
#define HYTECH_low_cell_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_low_cell_temp_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @high_cell_temp_ro
#define HYTECH_high_cell_temp_ro_CovFactor (0.01)
#define HYTECH_high_cell_temp_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_high_cell_temp_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t max_board_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t max_board_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t low_cell_temp_ro;                  //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t high_cell_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t max_board_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t max_board_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t low_cell_temp_ro;                  //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t high_cell_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_ONBOARD_TEMPS_t;

// def @BMS_ONBOARD_DETAILED_TEMPS CAN Message (214  0xd6)
#define BMS_ONBOARD_DETAILED_TEMPS_IDE (0U)
#define BMS_ONBOARD_DETAILED_TEMPS_DLC (5U)
#define BMS_ONBOARD_DETAILED_TEMPS_CANID (0xd6U)
// signal: @temp_0_ro
#define HYTECH_temp_0_ro_CovFactor (0.01)
#define HYTECH_temp_0_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_temp_0_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @temp_1_ro
#define HYTECH_temp_1_ro_CovFactor (0.01)
#define HYTECH_temp_1_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_temp_1_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t ic_id : 4;                         //      Bits= 4

  int16_t temp_0_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t temp_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t temp_1_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t temp_1_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t ic_id;                             //      Bits= 4

  int16_t temp_0_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t temp_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t temp_1_ro;                         //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t temp_1_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_ONBOARD_DETAILED_TEMPS_t;

// def @BMS_VOLTAGES CAN Message (215  0xd7)
#define BMS_VOLTAGES_IDE (0U)
#define BMS_VOLTAGES_DLC (8U)
#define BMS_VOLTAGES_CANID (0xd7U)
// signal: @average_voltage_ro
#define HYTECH_average_voltage_ro_CovFactor (0.0001)
#define HYTECH_average_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_average_voltage_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @low_voltage_ro
#define HYTECH_low_voltage_ro_CovFactor (0.0001)
#define HYTECH_low_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_low_voltage_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @high_voltage_ro
#define HYTECH_high_voltage_ro_CovFactor (0.0001)
#define HYTECH_high_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_high_voltage_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @total_voltage_ro
#define HYTECH_total_voltage_ro_CovFactor (0.01)
#define HYTECH_total_voltage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_total_voltage_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t average_voltage_ro;               //      Bits=16 Factor= 0.0001          Unit:'V'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t average_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t low_voltage_ro;                   //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t high_voltage_ro;                  //      Bits=16 Factor= 0.0001          Unit:'V'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t total_voltage_ro;                 //      Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t average_voltage_ro;               //      Bits=16 Factor= 0.0001          Unit:'V'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t average_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t low_voltage_ro;                   //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t high_voltage_ro;                  //      Bits=16 Factor= 0.0001          Unit:'V'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t total_voltage_ro;                 //      Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_VOLTAGES_t;

// def @BMS_DETAILED_VOLTAGES CAN Message (216  0xd8)
#define BMS_DETAILED_VOLTAGES_IDE (0U)
#define BMS_DETAILED_VOLTAGES_DLC (7U)
#define BMS_DETAILED_VOLTAGES_CANID (0xd8U)
// signal: @voltage_0_ro
#define HYTECH_voltage_0_ro_CovFactor (0.0001)
#define HYTECH_voltage_0_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_voltage_0_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @voltage_1_ro
#define HYTECH_voltage_1_ro_CovFactor (0.0001)
#define HYTECH_voltage_1_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_voltage_1_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @voltage_2_ro
#define HYTECH_voltage_2_ro_CovFactor (0.0001)
#define HYTECH_voltage_2_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_voltage_2_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t group_id : 4;                      //      Bits= 4

  uint8_t ic_id : 4;                         //      Bits= 4

  uint16_t voltage_0_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t voltage_1_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t voltage_2_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_2_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t group_id;                          //      Bits= 4

  uint8_t ic_id;                             //      Bits= 4

  uint16_t voltage_0_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t voltage_1_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t voltage_2_ro;                     //      Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t voltage_2_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_DETAILED_VOLTAGES_t;

// def @BMS_TEMPS CAN Message (217  0xd9)
#define BMS_TEMPS_IDE (0U)
#define BMS_TEMPS_DLC (6U)
#define BMS_TEMPS_CANID (0xd9U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t max_board_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t max_board_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t low_cell_temp_ro;                  //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t high_cell_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t max_board_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t max_board_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t low_cell_temp_ro;                  //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t low_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t high_cell_temp_ro;                 //  [-] Bits=16 Factor= 0.01            Unit:'Deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t high_cell_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_TEMPS_t;

// def @BMS_DETAILED_TEMPS CAN Message (218  0xda)
#define BMS_DETAILED_TEMPS_IDE (0U)
#define BMS_DETAILED_TEMPS_DLC (7U)
#define BMS_DETAILED_TEMPS_CANID (0xdaU)
// signal: @thermistor_id_0_ro
#define HYTECH_thermistor_id_0_ro_CovFactor (0.01)
#define HYTECH_thermistor_id_0_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_thermistor_id_0_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @thermistor_id_1_ro
#define HYTECH_thermistor_id_1_ro_CovFactor (0.01)
#define HYTECH_thermistor_id_1_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_thermistor_id_1_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @thermistor_id_2_ro
#define HYTECH_thermistor_id_2_ro_CovFactor (0.01)
#define HYTECH_thermistor_id_2_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_thermistor_id_2_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t group_id : 4;                      //      Bits= 4

  uint8_t ic_id : 4;                         //      Bits= 4

  int16_t thermistor_id_0_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t thermistor_id_1_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t thermistor_id_2_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_2_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t group_id;                          //      Bits= 4

  uint8_t ic_id;                             //      Bits= 4

  int16_t thermistor_id_0_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t thermistor_id_1_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t thermistor_id_2_ro;                //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_id_2_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_DETAILED_TEMPS_t;

// def @BMS_STATUS CAN Message (219  0xdb)
#define BMS_STATUS_IDE (0U)
#define BMS_STATUS_DLC (6U)
#define BMS_STATUS_CANID (0xdbU)
// signal: @current_ro
#define HYTECH_current_ro_CovFactor (0.01)
#define HYTECH_current_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_current_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t state;                                  //      Bits= 8

  // Whether or not the energy meter is reading an over-voltage error.
  uint8_t overvoltage_error : 1;                  //      Bits= 1 Unit:'none'

  uint8_t undervoltage_error : 1;                 //      Bits= 1

  uint8_t total_voltage_high_error : 1;           //      Bits= 1

  uint8_t discharge_overcurrent_error : 1;        //      Bits= 1

  uint8_t charge_overcurrent_error : 1;           //      Bits= 1

  uint8_t discharge_overtemp_error : 1;           //      Bits= 1

  uint8_t charge_overtemp_error : 1;              //      Bits= 1

  uint8_t undertemp_error : 1;                    //      Bits= 1

  uint8_t overtemp_error : 1;                     //      Bits= 1

  int16_t current_ro;                             //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t current_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t shutdown_g_above_threshold_error : 1;   //      Bits= 1

  uint8_t shutdown_h_above_threshold_error : 1;   //      Bits= 1

#else

  uint8_t state;                                  //      Bits= 8

  // Whether or not the energy meter is reading an over-voltage error.
  uint8_t overvoltage_error;                      //      Bits= 1 Unit:'none'

  uint8_t undervoltage_error;                     //      Bits= 1

  uint8_t total_voltage_high_error;               //      Bits= 1

  uint8_t discharge_overcurrent_error;            //      Bits= 1

  uint8_t charge_overcurrent_error;               //      Bits= 1

  uint8_t discharge_overtemp_error;               //      Bits= 1

  uint8_t charge_overtemp_error;                  //      Bits= 1

  uint8_t undertemp_error;                        //      Bits= 1

  uint8_t overtemp_error;                         //      Bits= 1

  int16_t current_ro;                             //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t current_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t shutdown_g_above_threshold_error;       //      Bits= 1

  uint8_t shutdown_h_above_threshold_error;       //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_STATUS_t;

// def @CCU_STATUS CAN Message (221  0xdd)
#define CCU_STATUS_IDE (0U)
#define CCU_STATUS_DLC (1U)
#define CCU_STATUS_CANID (0xddU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t charger_enabled : 1;               //      Bits= 1

#else

  uint8_t charger_enabled;                   //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CCU_STATUS_t;

// def @PENTHOUSE_ACCUM_MSG CAN Message (223  0xdf)
#define PENTHOUSE_ACCUM_MSG_IDE (0U)
#define PENTHOUSE_ACCUM_MSG_DLC (8U)
#define PENTHOUSE_ACCUM_MSG_CANID (0xdfU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t hall_curr_ref;                    //      Bits=16 Unit:'mV'

  uint16_t hall_curr_signal;                 //      Bits=16 Unit:'mV'

#else

  uint16_t hall_curr_ref;                    //      Bits=16 Unit:'mV'

  uint16_t hall_curr_signal;                 //      Bits=16 Unit:'mV'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} PENTHOUSE_ACCUM_MSG_t;

// def @VN_GPS_TIME_MSG CAN Message (224  0xe0)
#define VN_GPS_TIME_MSG_IDE (0U)
#define VN_GPS_TIME_MSG_DLC (8U)
#define VN_GPS_TIME_MSG_CANID (0xe0U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // time since start of epoch 1980
  uint64_t vn_gps_time;                      //      Bits=64 Unit:'ns'

#else

  // time since start of epoch 1980
  uint64_t vn_gps_time;                      //      Bits=64 Unit:'ns'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_GPS_TIME_MSG_t;

// def @VN_STATUS CAN Message (225  0xe1)
#define VN_STATUS_IDE (0U)
#define VN_STATUS_DLC (8U)
#define VN_STATUS_CANID (0xe1U)

// Value tables for @vn_gps_status signal

#ifndef vn_gps_status_VN_STATUS_NO_FIX
#define vn_gps_status_VN_STATUS_NO_FIX (0)
#endif

#ifndef vn_gps_status_VN_STATUS_TIME_ONLY
#define vn_gps_status_VN_STATUS_TIME_ONLY (1)
#endif

#ifndef vn_gps_status_VN_STATUS_FIX_2D
#define vn_gps_status_VN_STATUS_FIX_2D (2)
#endif

#ifndef vn_gps_status_VN_STATUS_FIX_3D
#define vn_gps_status_VN_STATUS_FIX_3D (3)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // GPS fix status
  //  0 : "NO_FIX"
  //  1 : "TIME_ONLY"
  //  2 : "FIX_2D"
  //  3 : "FIX_3D"
  uint8_t vn_gps_status;                     //      Bits= 8

#else

  // GPS fix status
  //  0 : "NO_FIX"
  //  1 : "TIME_ONLY"
  //  2 : "FIX_2D"
  //  3 : "FIX_3D"
  uint8_t vn_gps_status;                     //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_STATUS_t;

// UNUSED message to send data on the BMS current draw.
// def @BMS_COULOMB_COUNTS CAN Message (226  0xe2)
#define BMS_COULOMB_COUNTS_IDE (0U)
#define BMS_COULOMB_COUNTS_DLC (8U)
#define BMS_COULOMB_COUNTS_CANID (0xe2U)
// signal: @total_charge_ro
#define HYTECH_total_charge_ro_CovFactor (0.0001)
#define HYTECH_total_charge_ro_toS(x) ( (uint32_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_total_charge_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @total_discharge_ro
#define HYTECH_total_discharge_ro_CovFactor (0.0001)
#define HYTECH_total_discharge_ro_toS(x) ( (uint32_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_total_discharge_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t total_charge_ro;                  //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_charge_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint32_t total_discharge_ro;               //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_discharge_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint32_t total_charge_ro;                  //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_charge_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint32_t total_discharge_ro;               //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t total_discharge_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BMS_COULOMB_COUNTS_t;

// def @VN_ANGULAR_RATE CAN Message (227  0xe3)
#define VN_ANGULAR_RATE_IDE (0U)
#define VN_ANGULAR_RATE_DLC (8U)
#define VN_ANGULAR_RATE_CANID (0xe3U)
// signal: @angular_rate_x_ro
#define HYTECH_angular_rate_x_ro_CovFactor (0.001)
#define HYTECH_angular_rate_x_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_angular_rate_x_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @angular_rate_y_ro
#define HYTECH_angular_rate_y_ro_CovFactor (0.001)
#define HYTECH_angular_rate_y_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_angular_rate_y_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @angular_rate_z_ro
#define HYTECH_angular_rate_z_ro_CovFactor (0.001)
#define HYTECH_angular_rate_z_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_angular_rate_z_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t angular_rate_x_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t angular_rate_y_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t angular_rate_z_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t angular_rate_x_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t angular_rate_y_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_y_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t angular_rate_z_ro;                 //  [-] Bits=16 Factor= 0.001           Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t angular_rate_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_ANGULAR_RATE_t;

// def @REAR_SUSPENSION CAN Message (228  0xe4)
#define REAR_SUSPENSION_IDE (0U)
#define REAR_SUSPENSION_DLC (8U)
#define REAR_SUSPENSION_CANID (0xe4U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t rl_load_cell;                     //      Bits=16

  uint16_t rr_load_cell;                     //      Bits=16

  uint16_t rl_shock_pot;                     //      Bits=16

  uint16_t rr_shock_pot;                     //      Bits=16

#else

  uint16_t rl_load_cell;                     //      Bits=16

  uint16_t rr_load_cell;                     //      Bits=16

  uint16_t rl_shock_pot;                     //      Bits=16

  uint16_t rr_shock_pot;                     //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} REAR_SUSPENSION_t;

// def @SAB_THERMISTORS_1 CAN Message (229  0xe5)
#define SAB_THERMISTORS_1_IDE (0U)
#define SAB_THERMISTORS_1_DLC (8U)
#define SAB_THERMISTORS_1_CANID (0xe5U)
// signal: @thermistor_inv1_ro
#define HYTECH_thermistor_inv1_ro_CovFactor (0.005)
#define HYTECH_thermistor_inv1_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_inv1_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_inv2_ro
#define HYTECH_thermistor_inv2_ro_CovFactor (0.005)
#define HYTECH_thermistor_inv2_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_inv2_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_acc1_ro
#define HYTECH_thermistor_acc1_ro_CovFactor (0.005)
#define HYTECH_thermistor_acc1_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_acc1_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_acc2_ro
#define HYTECH_thermistor_acc2_ro_CovFactor (0.005)
#define HYTECH_thermistor_acc2_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_acc2_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t thermistor_inv1_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_inv1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_inv2_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_inv2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_acc1_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_acc1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_acc2_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_acc2_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t thermistor_inv1_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_inv1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_inv2_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_inv2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_acc1_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_acc1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_acc2_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_acc2_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} SAB_THERMISTORS_1_t;

// def @SAB_THERMISTORS_2 CAN Message (230  0xe6)
#define SAB_THERMISTORS_2_IDE (0U)
#define SAB_THERMISTORS_2_DLC (6U)
#define SAB_THERMISTORS_2_CANID (0xe6U)
// signal: @thermistor_motor_rl_ro
#define HYTECH_thermistor_motor_rl_ro_CovFactor (0.005)
#define HYTECH_thermistor_motor_rl_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_motor_rl_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_motor_rr_ro
#define HYTECH_thermistor_motor_rr_ro_CovFactor (0.005)
#define HYTECH_thermistor_motor_rr_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_motor_rr_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_pump_ro
#define HYTECH_thermistor_pump_ro_CovFactor (0.005)
#define HYTECH_thermistor_pump_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_pump_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t thermistor_motor_rl_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_motor_rr_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_pump_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_pump_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t thermistor_motor_rl_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_motor_rr_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_pump_ro;               //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_pump_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} SAB_THERMISTORS_2_t;

// def @VN_ECEF_POS_XY CAN Message (231  0xe7)
#define VN_ECEF_POS_XY_IDE (0U)
#define VN_ECEF_POS_XY_DLC (8U)
#define VN_ECEF_POS_XY_CANID (0xe7U)
// signal: @vn_ecef_pos_x_ro
#define HYTECH_vn_ecef_pos_x_ro_CovFactor (0.005)
#define HYTECH_vn_ecef_pos_x_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_vn_ecef_pos_x_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @vn_ecef_pos_y_ro
#define HYTECH_vn_ecef_pos_y_ro_CovFactor (0.005)
#define HYTECH_vn_ecef_pos_y_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_vn_ecef_pos_y_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t vn_ecef_pos_x_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t vn_ecef_pos_y_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_y_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int32_t vn_ecef_pos_x_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_x_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t vn_ecef_pos_y_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_y_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_ECEF_POS_XY_t;

// def @TCU_STATUS CAN Message (232  0xe8)
#define TCU_STATUS_IDE (0U)
#define TCU_STATUS_DLC (3U)
#define TCU_STATUS_CANID (0xe8U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t shutdown_status;                  //      Bits=16

  // The current state of TCU data recording (off, requested,on,saving)
  uint8_t tcu_recording_state : 2;           //      Bits= 2

#else

  uint16_t shutdown_status;                  //      Bits=16

  // The current state of TCU data recording (off, requested,on,saving)
  uint8_t tcu_recording_state;               //      Bits= 2

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TCU_STATUS_t;

// def @VN_ECEF_POS_Z_MSG CAN Message (233  0xe9)
#define VN_ECEF_POS_Z_MSG_IDE (0U)
#define VN_ECEF_POS_Z_MSG_DLC (8U)
#define VN_ECEF_POS_Z_MSG_CANID (0xe9U)
// signal: @vn_ecef_pos_z_ro
#define HYTECH_vn_ecef_pos_z_ro_CovFactor (0.005)
#define HYTECH_vn_ecef_pos_z_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_vn_ecef_pos_z_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t vn_ecef_pos_z_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int32_t vn_ecef_pos_z_ro;                  //  [-] Bits=32 Factor= 0.005           Unit:'m'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vn_ecef_pos_z_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_ECEF_POS_Z_MSG_t;

// def @VN_GNSS_COMP_SIG_HEALTH CAN Message (234  0xea)
#define VN_GNSS_COMP_SIG_HEALTH_IDE (0U)
#define VN_GNSS_COMP_SIG_HEALTH_DLC (8U)
#define VN_GNSS_COMP_SIG_HEALTH_CANID (0xeaU)
// signal: @highest_cn0_1_ro
#define HYTECH_highest_cn0_1_ro_CovFactor (0.5)
#define HYTECH_highest_cn0_1_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_highest_cn0_1_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @highest_cn0_2_ro
#define HYTECH_highest_cn0_2_ro_CovFactor (0.5)
#define HYTECH_highest_cn0_2_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_highest_cn0_2_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // num of common satelites that are used in the PVT solution on receiver A
  uint8_t num_sats_pvt_1;                    //      Bits= 8

  // num of common satelites that are used in the RTK solution on receiver A
  uint8_t num_sats_rtk_1;                    //      Bits= 8

  uint8_t highest_cn0_1_ro;                  //      Bits= 8 Factor= 0.5             Unit:'dBHz'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t highest_cn0_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  // num of common satelites that are used in the PVT solution on receiver B
  uint8_t num_sats_pvt_2;                    //      Bits= 8

  // num of common satelites that are used in the RTK solution on receiver B
  uint8_t num_sats_rtk_2;                    //      Bits= 8

  uint8_t highest_cn0_2_ro;                  //      Bits= 8 Factor= 0.5             Unit:'dBHz'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t highest_cn0_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  // num of common satelites that are used in the PVT solution on both receivers
  uint8_t num_com_sats_pvt;                  //      Bits= 8

  // num of common satelites that are used in the RTK solution on both receivers
  uint8_t num_com_sats_rtk;                  //      Bits= 8

#else

  // num of common satelites that are used in the PVT solution on receiver A
  uint8_t num_sats_pvt_1;                    //      Bits= 8

  // num of common satelites that are used in the RTK solution on receiver A
  uint8_t num_sats_rtk_1;                    //      Bits= 8

  uint8_t highest_cn0_1_ro;                  //      Bits= 8 Factor= 0.5             Unit:'dBHz'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t highest_cn0_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  // num of common satelites that are used in the PVT solution on receiver B
  uint8_t num_sats_pvt_2;                    //      Bits= 8

  // num of common satelites that are used in the RTK solution on receiver B
  uint8_t num_sats_rtk_2;                    //      Bits= 8

  uint8_t highest_cn0_2_ro;                  //      Bits= 8 Factor= 0.5             Unit:'dBHz'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t highest_cn0_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  // num of common satelites that are used in the PVT solution on both receivers
  uint8_t num_com_sats_pvt;                  //      Bits= 8

  // num of common satelites that are used in the RTK solution on both receivers
  uint8_t num_com_sats_rtk;                  //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VN_GNSS_COMP_SIG_HEALTH_t;

// def @DASHBOARD_MCU_STATE CAN Message (235  0xeb)
#define DASHBOARD_MCU_STATE_IDE (0U)
#define DASHBOARD_MCU_STATE_DLC (6U)
#define DASHBOARD_MCU_STATE_CANID (0xebU)

// Value tables for @dial_state signal

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_0
#define dial_state_DASHBOARD_MCU_STATE_MODE_0 (0)
#endif

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_1
#define dial_state_DASHBOARD_MCU_STATE_MODE_1 (1)
#endif

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_2
#define dial_state_DASHBOARD_MCU_STATE_MODE_2 (2)
#endif

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_3
#define dial_state_DASHBOARD_MCU_STATE_MODE_3 (3)
#endif

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_4
#define dial_state_DASHBOARD_MCU_STATE_MODE_4 (4)
#endif

#ifndef dial_state_DASHBOARD_MCU_STATE_MODE_5
#define dial_state_DASHBOARD_MCU_STATE_MODE_5 (5)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t bots_led : 2;                      //      Bits= 2

  uint8_t launch_control_led : 2;            //      Bits= 2

  uint8_t mode_led : 2;                      //      Bits= 2

  uint8_t mechanical_brake_led : 2;          //      Bits= 2

  uint8_t cockpit_brb_led : 2;               //      Bits= 2

  uint8_t inertia_status_led : 2;            //      Bits= 2

  uint8_t start_status_led : 2;              //      Bits= 2

  uint8_t motor_controller_error_led : 2;    //      Bits= 2

  uint8_t imd_led : 2;                       //      Bits= 2

  uint8_t ams_led : 2;                       //      Bits= 2

  uint8_t drive_buzzer : 1;                  //      Bits= 1

  // glv led has a value from 0 to 255 representing a percentage that can be displayed on dashboard as a gradient between two colors
  uint8_t glv_led;                           //      Bits= 8

  // pack charge led has a value from 0 to 255 representing a percentage that can be displayed on dashboard as a gradient between two colors
  uint8_t pack_charge_led;                   //      Bits= 8

  //  0 : "MODE_0"
  //  1 : "MODE_1"
  //  2 : "MODE_2"
  //  3 : "MODE_3"
  //  4 : "MODE_4"
  //  5 : "MODE_5"
  uint8_t dial_state;                        //      Bits= 8

#else

  uint8_t bots_led;                          //      Bits= 2

  uint8_t launch_control_led;                //      Bits= 2

  uint8_t mode_led;                          //      Bits= 2

  uint8_t mechanical_brake_led;              //      Bits= 2

  uint8_t cockpit_brb_led;                   //      Bits= 2

  uint8_t inertia_status_led;                //      Bits= 2

  uint8_t start_status_led;                  //      Bits= 2

  uint8_t motor_controller_error_led;        //      Bits= 2

  uint8_t imd_led;                           //      Bits= 2

  uint8_t ams_led;                           //      Bits= 2

  uint8_t drive_buzzer;                      //      Bits= 1

  // glv led has a value from 0 to 255 representing a percentage that can be displayed on dashboard as a gradient between two colors
  uint8_t glv_led;                           //      Bits= 8

  // pack charge led has a value from 0 to 255 representing a percentage that can be displayed on dashboard as a gradient between two colors
  uint8_t pack_charge_led;                   //      Bits= 8

  //  0 : "MODE_0"
  //  1 : "MODE_1"
  //  2 : "MODE_2"
  //  3 : "MODE_3"
  //  4 : "MODE_4"
  //  5 : "MODE_5"
  uint8_t dial_state;                        //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DASHBOARD_MCU_STATE_t;

// def @DASHBOARD_STATE CAN Message (236  0xec)
#define DASHBOARD_STATE_IDE (0U)
#define DASHBOARD_STATE_DLC (3U)
#define DASHBOARD_STATE_CANID (0xecU)

// Value tables for @dial_state signal

#ifndef dial_state_DASHBOARD_STATE_MODE_0
#define dial_state_DASHBOARD_STATE_MODE_0 (0)
#endif

#ifndef dial_state_DASHBOARD_STATE_MODE_1
#define dial_state_DASHBOARD_STATE_MODE_1 (1)
#endif

#ifndef dial_state_DASHBOARD_STATE_MODE_2
#define dial_state_DASHBOARD_STATE_MODE_2 (2)
#endif

#ifndef dial_state_DASHBOARD_STATE_MODE_3
#define dial_state_DASHBOARD_STATE_MODE_3 (3)
#endif

#ifndef dial_state_DASHBOARD_STATE_MODE_4
#define dial_state_DASHBOARD_STATE_MODE_4 (4)
#endif

#ifndef dial_state_DASHBOARD_STATE_MODE_5
#define dial_state_DASHBOARD_STATE_MODE_5 (5)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t start_button : 1;                    //      Bits= 1

  uint8_t mark_button : 1;                     //      Bits= 1

  uint8_t mode_button : 1;                     //      Bits= 1

  uint8_t motor_controller_cycle_button : 1;   //      Bits= 1

  uint8_t launch_ctrl_button : 1;              //      Bits= 1

  uint8_t torque_mode_button : 1;              //      Bits= 1

  uint8_t led_dimmer_button : 1;               //      Bits= 1

  uint8_t left_shifter_button : 1;             //      Bits= 1

  uint8_t right_shifter_button : 1;            //      Bits= 1

  uint8_t shutdown_h_above_threshold : 1;      //      Bits= 1

  uint8_t ssok_above_threshold : 1;            //      Bits= 1

  uint8_t drive_buzzer : 1;                    //      Bits= 1

  // The current state of TCU data recording (off, requested,on,saving)
  uint8_t tcu_recording_state : 2;             //      Bits= 2

  //  0 : "MODE_0"
  //  1 : "MODE_1"
  //  2 : "MODE_2"
  //  3 : "MODE_3"
  //  4 : "MODE_4"
  //  5 : "MODE_5"
  uint8_t dial_state;                          //      Bits= 8

#else

  uint8_t start_button;                        //      Bits= 1

  uint8_t mark_button;                         //      Bits= 1

  uint8_t mode_button;                         //      Bits= 1

  uint8_t motor_controller_cycle_button;       //      Bits= 1

  uint8_t launch_ctrl_button;                  //      Bits= 1

  uint8_t torque_mode_button;                  //      Bits= 1

  uint8_t led_dimmer_button;                   //      Bits= 1

  uint8_t left_shifter_button;                 //      Bits= 1

  uint8_t right_shifter_button;                //      Bits= 1

  uint8_t shutdown_h_above_threshold;          //      Bits= 1

  uint8_t ssok_above_threshold;                //      Bits= 1

  uint8_t drive_buzzer;                        //      Bits= 1

  // The current state of TCU data recording (off, requested,on,saving)
  uint8_t tcu_recording_state;                 //      Bits= 2

  //  0 : "MODE_0"
  //  1 : "MODE_1"
  //  2 : "MODE_2"
  //  3 : "MODE_3"
  //  4 : "MODE_4"
  //  5 : "MODE_5"
  uint8_t dial_state;                          //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DASHBOARD_STATE_t;

// def @FRONT_SUSPENSION CAN Message (237  0xed)
#define FRONT_SUSPENSION_IDE (0U)
#define FRONT_SUSPENSION_DLC (8U)
#define FRONT_SUSPENSION_CANID (0xedU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t fl_load_cell;                     //      Bits=16

  uint16_t fl_shock_pot;                     //      Bits=16

  uint16_t fr_load_cell;                     //      Bits=16

  uint16_t fr_shock_pot;                     //      Bits=16

#else

  uint16_t fl_load_cell;                     //      Bits=16

  uint16_t fl_shock_pot;                     //      Bits=16

  uint16_t fr_load_cell;                     //      Bits=16

  uint16_t fr_shock_pot;                     //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FRONT_SUSPENSION_t;

// def @DRIVEBRAIN_TORQUE_LIM_INPUT CAN Message (241  0xf1)
#define DRIVEBRAIN_TORQUE_LIM_INPUT_IDE (0U)
#define DRIVEBRAIN_TORQUE_LIM_INPUT_DLC (8U)
#define DRIVEBRAIN_TORQUE_LIM_INPUT_CANID (0xf1U)
// signal: @drivebrain_torque_fl_ro
#define HYTECH_drivebrain_torque_fl_ro_CovFactor (0.01)
#define HYTECH_drivebrain_torque_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_drivebrain_torque_fl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @drivebrain_torque_fr_ro
#define HYTECH_drivebrain_torque_fr_ro_CovFactor (0.01)
#define HYTECH_drivebrain_torque_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_drivebrain_torque_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @drivebrain_torque_rl_ro
#define HYTECH_drivebrain_torque_rl_ro_CovFactor (0.01)
#define HYTECH_drivebrain_torque_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_drivebrain_torque_rl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @drivebrain_torque_rr_ro
#define HYTECH_drivebrain_torque_rr_ro_CovFactor (0.01)
#define HYTECH_drivebrain_torque_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_drivebrain_torque_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t drivebrain_torque_fl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_fr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t drivebrain_torque_fl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_fr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVEBRAIN_TORQUE_LIM_INPUT_t;

// def @DRIVEBRAIN_SPEED_SET_INPUT CAN Message (242  0xf2)
#define DRIVEBRAIN_SPEED_SET_INPUT_IDE (0U)
#define DRIVEBRAIN_SPEED_SET_INPUT_DLC (8U)
#define DRIVEBRAIN_SPEED_SET_INPUT_CANID (0xf2U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t drivebrain_set_rpm_fl;            //      Bits=16

  uint16_t drivebrain_set_rpm_fr;            //      Bits=16

  uint16_t drivebrain_set_rpm_rl;            //      Bits=16

  uint16_t drivebrain_set_rpm_rr;            //      Bits=16

#else

  uint16_t drivebrain_set_rpm_fl;            //      Bits=16

  uint16_t drivebrain_set_rpm_fr;            //      Bits=16

  uint16_t drivebrain_set_rpm_rl;            //      Bits=16

  uint16_t drivebrain_set_rpm_rr;            //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVEBRAIN_SPEED_SET_INPUT_t;

// def @DRIVEBRAIN_DESIRED_TORQUE_INPUT CAN Message (243  0xf3)
#define DRIVEBRAIN_DESIRED_TORQUE_INPUT_IDE (0U)
#define DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC (8U)
#define DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID (0xf3U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t drivebrain_torque_fl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_fr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t drivebrain_torque_fl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_fr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rl_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivebrain_torque_rr_ro;           //  [-] Bits=16 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivebrain_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVEBRAIN_DESIRED_TORQUE_INPUT_t;

// def @FRONT_THERMISTORS CAN Message (245  0xf5)
#define FRONT_THERMISTORS_IDE (0U)
#define FRONT_THERMISTORS_DLC (8U)
#define FRONT_THERMISTORS_CANID (0xf5U)
// signal: @thermistor_motor_fl_ro
#define HYTECH_thermistor_motor_fl_ro_CovFactor (0.005)
#define HYTECH_thermistor_motor_fl_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_motor_fl_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )
// signal: @thermistor_motor_fr_ro
#define HYTECH_thermistor_motor_fr_ro_CovFactor (0.005)
#define HYTECH_thermistor_motor_fr_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.005)) )
#define HYTECH_thermistor_motor_fr_ro_fromS(x) ( (((x) * (0.005)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t thermistor_motor_fl_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_motor_fr_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t thermistor_motor_fl_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t thermistor_motor_fr_ro;           //      Bits=16 Factor= 0.005           Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t thermistor_motor_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FRONT_THERMISTORS_t;

// Contains the voltage and current readings from the Energy Meter. Sent by the AMS.
// def @EM_MEASUREMENT CAN Message (256  0x100)
#define EM_MEASUREMENT_IDE (0U)
#define EM_MEASUREMENT_DLC (8U)
#define EM_MEASUREMENT_CANID (0x100U)
// signal: @em_current_ro
#define HYTECH_em_current_ro_CovFactor (0.000015259)
#define HYTECH_em_current_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.000015259)) )
#define HYTECH_em_current_ro_fromS(x) ( (((x) * (0.000015259)) + (0.0)) )
// signal: @em_voltage_ro
#define HYTECH_em_voltage_ro_CovFactor (0.000015259)
#define HYTECH_em_voltage_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.000015259)) )
#define HYTECH_em_voltage_ro_fromS(x) ( (((x) * (0.000015259)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t em_current_ro;                     //  [-] Bits=32 Factor= 0.000015259     Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_current_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t em_voltage_ro;                     //  [-] Bits=32 Factor= 0.000015259     Unit:'Volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int32_t em_current_ro;                     //  [-] Bits=32 Factor= 0.000015259     Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_current_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t em_voltage_ro;                     //  [-] Bits=32 Factor= 0.000015259     Unit:'Volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_voltage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} EM_MEASUREMENT_t;

// this message will have states internal to the MCU code
// def @MCU_ERROR_STATES CAN Message (257  0x101)
#define MCU_ERROR_STATES_IDE (0U)
#define MCU_ERROR_STATES_DLC (8U)
#define MCU_ERROR_STATES_CANID (0x101U)

// Value tables for @torque_controller_mux_status signal

#ifndef torque_controller_mux_status_MCU_ERROR_STATES_NO_ERROR
#define torque_controller_mux_status_MCU_ERROR_STATES_NO_ERROR (0)
#endif

#ifndef torque_controller_mux_status_MCU_ERROR_STATES_ERROR_SPEED_DIFF_TOO_HIGH
#define torque_controller_mux_status_MCU_ERROR_STATES_ERROR_SPEED_DIFF_TOO_HIGH (1)
#endif

#ifndef torque_controller_mux_status_MCU_ERROR_STATES_ERROR_TORQUE_DIFF_TOO_HIGH
#define torque_controller_mux_status_MCU_ERROR_STATES_ERROR_TORQUE_DIFF_TOO_HIGH (2)
#endif

#ifndef torque_controller_mux_status_MCU_ERROR_STATES_ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS
#define torque_controller_mux_status_MCU_ERROR_STATES_ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS (3)
#endif

#ifndef torque_controller_mux_status_MCU_ERROR_STATES_ERROR_CONTROLLER_NULL_POINTER
#define torque_controller_mux_status_MCU_ERROR_STATES_ERROR_CONTROLLER_NULL_POINTER (4)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  //  0 : "NO_ERROR"
  //  1 : "ERROR_SPEED_DIFF_TOO_HIGH"
  //  2 : "ERROR_TORQUE_DIFF_TOO_HIGH"
  //  3 : "ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS"
  //  4 : "ERROR_CONTROLLER_NULL_POINTER"
  uint8_t torque_controller_mux_status : 3;   //      Bits= 3

#else

  //  0 : "NO_ERROR"
  //  1 : "ERROR_SPEED_DIFF_TOO_HIGH"
  //  2 : "ERROR_TORQUE_DIFF_TOO_HIGH"
  //  3 : "ERROR_CONTROLLER_INDEX_OUT_OF_BOUNDS"
  //  4 : "ERROR_CONTROLLER_NULL_POINTER"
  uint8_t torque_controller_mux_status;       //      Bits= 3

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} MCU_ERROR_STATES_t;

// def @INV4_CONTROL_INPUT CAN Message (258  0x102)
#define INV4_CONTROL_INPUT_IDE (0U)
#define INV4_CONTROL_INPUT_DLC (6U)
#define INV4_CONTROL_INPUT_CANID (0x102U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t speed_setpoint_rpm;                //  [-] Bits=16 Unit:'rpm'

  int16_t positive_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t positive_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t negative_torque_limit_ro;          //  [-] Bits=16 Factor= 0.0098          Unit:'Mn'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t negative_torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_CONTROL_INPUT_t;

// def @INV1_CONTROL_PARAMETER CAN Message (259  0x103)
#define INV1_CONTROL_PARAMETER_IDE (0U)
#define INV1_CONTROL_PARAMETER_DLC (8U)
#define INV1_CONTROL_PARAMETER_CANID (0x103U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_CONTROL_PARAMETER_t;

// def @INV2_CONTROL_PARAMETER CAN Message (260  0x104)
#define INV2_CONTROL_PARAMETER_IDE (0U)
#define INV2_CONTROL_PARAMETER_DLC (8U)
#define INV2_CONTROL_PARAMETER_CANID (0x104U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_CONTROL_PARAMETER_t;

// def @INV3_CONTROL_PARAMETER CAN Message (261  0x105)
#define INV3_CONTROL_PARAMETER_IDE (0U)
#define INV3_CONTROL_PARAMETER_DLC (8U)
#define INV3_CONTROL_PARAMETER_CANID (0x105U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_CONTROL_PARAMETER_t;

// def @INV4_CONTROL_PARAMETER CAN Message (262  0x106)
#define INV4_CONTROL_PARAMETER_IDE (0U)
#define INV4_CONTROL_PARAMETER_DLC (8U)
#define INV4_CONTROL_PARAMETER_CANID (0x106U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#else

  uint16_t speed_control_kp;                 //      Bits=16

  uint16_t speed_control_ki;                 //      Bits=16

  uint16_t speed_control_kd;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_CONTROL_PARAMETER_t;

// def @ENERGY_METER_MEAS CAN Message (269  0x10d)
#define ENERGY_METER_MEAS_IDE (0U)
#define ENERGY_METER_MEAS_DLC (8U)
#define ENERGY_METER_MEAS_CANID (0x10dU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint32_t current_A;                        //      Bits=32

  uint32_t voltage_V;                        //      Bits=32

#else

  uint32_t current_A;                        //      Bits=32

  uint32_t voltage_V;                        //      Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} ENERGY_METER_MEAS_t;

// def @DRIVETRAIN_COMMAND CAN Message (303  0x12f)
#define DRIVETRAIN_COMMAND_IDE (0U)
#define DRIVETRAIN_COMMAND_DLC (8U)
#define DRIVETRAIN_COMMAND_CANID (0x12fU)
// signal: @drivetrain_traj_torque_lim_fl_ro
#define HYTECH_drivetrain_traj_torque_lim_fl_ro_CovFactor (0.001)
#define HYTECH_drivetrain_traj_torque_lim_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_drivetrain_traj_torque_lim_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @drivetrain_traj_torque_lim_fr_ro
#define HYTECH_drivetrain_traj_torque_lim_fr_ro_CovFactor (0.001)
#define HYTECH_drivetrain_traj_torque_lim_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_drivetrain_traj_torque_lim_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @drivetrain_traj_torque_lim_rl_ro
#define HYTECH_drivetrain_traj_torque_lim_rl_ro_CovFactor (0.001)
#define HYTECH_drivetrain_traj_torque_lim_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_drivetrain_traj_torque_lim_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @drivetrain_traj_torque_lim_rr_ro
#define HYTECH_drivetrain_traj_torque_lim_rr_ro_CovFactor (0.001)
#define HYTECH_drivetrain_traj_torque_lim_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_drivetrain_traj_torque_lim_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t drivetrain_traj_torque_lim_fl_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_fr_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_rl_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_rr_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t drivetrain_traj_torque_lim_fl_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_fr_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_rl_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t drivetrain_traj_torque_lim_rr_ro;       //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t drivetrain_traj_torque_lim_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_COMMAND_t;

// def @INV1_OVERLOAD CAN Message (310  0x136)
#define INV1_OVERLOAD_IDE (0U)
#define INV1_OVERLOAD_DLC (8U)
#define INV1_OVERLOAD_CANID (0x136U)
// signal: @inverter_overload_percentage_ro
#define HYTECH_inverter_overload_percentage_ro_CovFactor (0.1)
#define HYTECH_inverter_overload_percentage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_inverter_overload_percentage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @motor_overload_percentage_ro
#define HYTECH_motor_overload_percentage_ro_CovFactor (0.1)
#define HYTECH_motor_overload_percentage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_motor_overload_percentage_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV1_OVERLOAD_t;

// def @INV2_OVERLOAD CAN Message (320  0x140)
#define INV2_OVERLOAD_IDE (0U)
#define INV2_OVERLOAD_DLC (4U)
#define INV2_OVERLOAD_CANID (0x140U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV2_OVERLOAD_t;

// def @INV3_OVERLOAD CAN Message (330  0x14a)
#define INV3_OVERLOAD_IDE (0U)
#define INV3_OVERLOAD_DLC (4U)
#define INV3_OVERLOAD_CANID (0x14aU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV3_OVERLOAD_t;

// def @INV4_OVERLOAD CAN Message (340  0x154)
#define INV4_OVERLOAD_IDE (0U)
#define INV4_OVERLOAD_DLC (4U)
#define INV4_OVERLOAD_CANID (0x154U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t inverter_overload_percentage_ro;      //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t inverter_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t motor_overload_percentage_ro;         //      Bits=16 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t motor_overload_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} INV4_OVERLOAD_t;

// def @DRIVETRAIN_RPMS_TELEM CAN Message (512  0x200)
#define DRIVETRAIN_RPMS_TELEM_IDE (0U)
#define DRIVETRAIN_RPMS_TELEM_DLC (8U)
#define DRIVETRAIN_RPMS_TELEM_CANID (0x200U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t fr_motor_rpm;                      //  [-] Bits=16

  int16_t fl_motor_rpm;                      //  [-] Bits=16

  int16_t rr_motor_rpm;                      //  [-] Bits=16

  int16_t rl_motor_rpm;                      //  [-] Bits=16

#else

  int16_t fr_motor_rpm;                      //  [-] Bits=16

  int16_t fl_motor_rpm;                      //  [-] Bits=16

  int16_t rr_motor_rpm;                      //  [-] Bits=16

  int16_t rl_motor_rpm;                      //  [-] Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_RPMS_TELEM_t;

// def @DRIVETRAIN_ERR_STATUS_TELEM CAN Message (513  0x201)
#define DRIVETRAIN_ERR_STATUS_TELEM_IDE (0U)
#define DRIVETRAIN_ERR_STATUS_TELEM_DLC (8U)
#define DRIVETRAIN_ERR_STATUS_TELEM_CANID (0x201U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t mc1_diagnostic_number;            //      Bits=16

  uint16_t mc2_diagnostic_number;            //      Bits=16

  uint16_t mc3_diagnostic_number;            //      Bits=16

  uint16_t mc4_diagnostic_number;            //      Bits=16

#else

  uint16_t mc1_diagnostic_number;            //      Bits=16

  uint16_t mc2_diagnostic_number;            //      Bits=16

  uint16_t mc3_diagnostic_number;            //      Bits=16

  uint16_t mc4_diagnostic_number;            //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_ERR_STATUS_TELEM_t;

// def @DRIVETRAIN_STATUS_TELEM CAN Message (514  0x202)
#define DRIVETRAIN_STATUS_TELEM_IDE (0U)
#define DRIVETRAIN_STATUS_TELEM_DLC (8U)
#define DRIVETRAIN_STATUS_TELEM_CANID (0x202U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t mc1_dc_on : 1;                     //      Bits= 1

  uint8_t mc1_derating_on : 1;               //      Bits= 1

  uint8_t mc1_error : 1;                     //      Bits= 1

  uint8_t mc1_inverter_on : 1;               //      Bits= 1

  uint8_t mc1_quit_dc : 1;                   //      Bits= 1

  uint8_t mc1_quit_inverter_on : 1;          //      Bits= 1

  uint8_t mc1_system_ready : 1;              //      Bits= 1

  uint8_t mc1_warning : 1;                   //      Bits= 1

  uint8_t mc2_dc_on : 1;                     //      Bits= 1

  uint8_t mc2_derating_on : 1;               //      Bits= 1

  uint8_t mc2_error : 1;                     //      Bits= 1

  uint8_t mc2_inverter_on : 1;               //      Bits= 1

  uint8_t mc2_quit_dc : 1;                   //      Bits= 1

  uint8_t mc2_quit_inverter_on : 1;          //      Bits= 1

  uint8_t mc2_system_ready : 1;              //      Bits= 1

  uint8_t mc2_warning : 1;                   //      Bits= 1

  uint8_t mc3_dc_on : 1;                     //      Bits= 1

  uint8_t mc3_derating_on : 1;               //      Bits= 1

  uint8_t mc3_error : 1;                     //      Bits= 1

  uint8_t mc3_inverter_on : 1;               //      Bits= 1

  uint8_t mc3_quit_dc : 1;                   //      Bits= 1

  uint8_t mc3_quit_inverter_on : 1;          //      Bits= 1

  uint8_t mc3_system_ready : 1;              //      Bits= 1

  uint8_t mc3_warning : 1;                   //      Bits= 1

  uint8_t mc4_dc_on : 1;                     //      Bits= 1

  uint8_t mc4_derating_on : 1;               //      Bits= 1

  uint8_t mc4_error : 1;                     //      Bits= 1

  uint8_t mc4_inverter_on : 1;               //      Bits= 1

  uint8_t mc4_quit_dc : 1;                   //      Bits= 1

  uint8_t mc4_quit_inverter_on : 1;          //      Bits= 1

  uint8_t mc4_system_ready : 1;              //      Bits= 1

  uint8_t mc4_warning : 1;                   //      Bits= 1

  uint8_t accel_implausible : 1;             //      Bits= 1

  uint8_t brake_implausible : 1;             //      Bits= 1

  uint8_t brake_percent;                     //      Bits= 8

  uint8_t accel_percent;                     //      Bits= 8

#else

  uint8_t mc1_dc_on;                         //      Bits= 1

  uint8_t mc1_derating_on;                   //      Bits= 1

  uint8_t mc1_error;                         //      Bits= 1

  uint8_t mc1_inverter_on;                   //      Bits= 1

  uint8_t mc1_quit_dc;                       //      Bits= 1

  uint8_t mc1_quit_inverter_on;              //      Bits= 1

  uint8_t mc1_system_ready;                  //      Bits= 1

  uint8_t mc1_warning;                       //      Bits= 1

  uint8_t mc2_dc_on;                         //      Bits= 1

  uint8_t mc2_derating_on;                   //      Bits= 1

  uint8_t mc2_error;                         //      Bits= 1

  uint8_t mc2_inverter_on;                   //      Bits= 1

  uint8_t mc2_quit_dc;                       //      Bits= 1

  uint8_t mc2_quit_inverter_on;              //      Bits= 1

  uint8_t mc2_system_ready;                  //      Bits= 1

  uint8_t mc2_warning;                       //      Bits= 1

  uint8_t mc3_dc_on;                         //      Bits= 1

  uint8_t mc3_derating_on;                   //      Bits= 1

  uint8_t mc3_error;                         //      Bits= 1

  uint8_t mc3_inverter_on;                   //      Bits= 1

  uint8_t mc3_quit_dc;                       //      Bits= 1

  uint8_t mc3_quit_inverter_on;              //      Bits= 1

  uint8_t mc3_system_ready;                  //      Bits= 1

  uint8_t mc3_warning;                       //      Bits= 1

  uint8_t mc4_dc_on;                         //      Bits= 1

  uint8_t mc4_derating_on;                   //      Bits= 1

  uint8_t mc4_error;                         //      Bits= 1

  uint8_t mc4_inverter_on;                   //      Bits= 1

  uint8_t mc4_quit_dc;                       //      Bits= 1

  uint8_t mc4_quit_inverter_on;              //      Bits= 1

  uint8_t mc4_system_ready;                  //      Bits= 1

  uint8_t mc4_warning;                       //      Bits= 1

  uint8_t accel_implausible;                 //      Bits= 1

  uint8_t brake_implausible;                 //      Bits= 1

  uint8_t brake_percent;                     //      Bits= 8

  uint8_t accel_percent;                     //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_STATUS_TELEM_t;

// def @DRIVETRAIN_TORQUE_TELEM CAN Message (515  0x203)
#define DRIVETRAIN_TORQUE_TELEM_IDE (0U)
#define DRIVETRAIN_TORQUE_TELEM_DLC (8U)
#define DRIVETRAIN_TORQUE_TELEM_CANID (0x203U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t fl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t fr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

#else

  int16_t fl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t fr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_TORQUE_TELEM_t;

// def @DRIVETRAIN_FILTER_OUT_TORQUE_TEL CAN Message (516  0x204)
#define DRIVETRAIN_FILTER_OUT_TORQUE_TEL_IDE (0U)
#define DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC (8U)
#define DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID (0x204U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t fl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t fr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

#else

  int16_t fl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t fr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rl_motor_torque;                   //  [-] Bits=16 Unit:'nm'

  int16_t rr_motor_torque;                   //  [-] Bits=16 Unit:'nm'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t;

// def @CAR_STATES CAN Message (519  0x207)
#define CAR_STATES_IDE (0U)
#define CAR_STATES_DLC (3U)
#define CAR_STATES_CANID (0x207U)

// Value tables for @vehicle_state signal

#ifndef vehicle_state_CAR_STATES_TRACTIVE_SYSTEM_NOT_ACTIVE
#define vehicle_state_CAR_STATES_TRACTIVE_SYSTEM_NOT_ACTIVE (0)
#endif

#ifndef vehicle_state_CAR_STATES_TRACTIVE_SYSTEM_ACTIVE
#define vehicle_state_CAR_STATES_TRACTIVE_SYSTEM_ACTIVE (1)
#endif

#ifndef vehicle_state_CAR_STATES_WANTING_READY_TO_DRIVE
#define vehicle_state_CAR_STATES_WANTING_READY_TO_DRIVE (2)
#endif

#ifndef vehicle_state_CAR_STATES_READY_TO_DRIVE
#define vehicle_state_CAR_STATES_READY_TO_DRIVE (3)
#endif

#ifndef vehicle_state_CAR_STATES_WANTING_RECALIBRATE_PEDALS
#define vehicle_state_CAR_STATES_WANTING_RECALIBRATE_PEDALS (4)
#endif

#ifndef vehicle_state_CAR_STATES_RECALIBRATING_PEDALS
#define vehicle_state_CAR_STATES_RECALIBRATING_PEDALS (5)
#endif


// Value tables for @drivetrain_state signal

#ifndef drivetrain_state_CAR_STATES_NOT_CONNECTED
#define drivetrain_state_CAR_STATES_NOT_CONNECTED (0)
#endif

#ifndef drivetrain_state_CAR_STATES_NOT_ENABLED_NO_HV_PRESENT
#define drivetrain_state_CAR_STATES_NOT_ENABLED_NO_HV_PRESENT (1)
#endif

#ifndef drivetrain_state_CAR_STATES_NOT_ENABLED_HV_PRESENT
#define drivetrain_state_CAR_STATES_NOT_ENABLED_HV_PRESENT (2)
#endif

#ifndef drivetrain_state_CAR_STATES_INVERTERS_READY
#define drivetrain_state_CAR_STATES_INVERTERS_READY (3)
#endif

#ifndef drivetrain_state_CAR_STATES_INVERTERS_HV_ENABLED
#define drivetrain_state_CAR_STATES_INVERTERS_HV_ENABLED (4)
#endif

#ifndef drivetrain_state_CAR_STATES_ENABLED_DRIVE_MODE
#define drivetrain_state_CAR_STATES_ENABLED_DRIVE_MODE (5)
#endif

#ifndef drivetrain_state_CAR_STATES_ERROR
#define drivetrain_state_CAR_STATES_ERROR (6)
#endif

#ifndef drivetrain_state_CAR_STATES_CLEARING_ERRORS
#define drivetrain_state_CAR_STATES_CLEARING_ERRORS (7)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  //  0 : "TRACTIVE_SYSTEM_NOT_ACTIVE"
  //  1 : "TRACTIVE_SYSTEM_ACTIVE"
  //  2 : "WANTING_READY_TO_DRIVE"
  //  3 : "READY_TO_DRIVE"
  //  4 : "WANTING_RECALIBRATE_PEDALS"
  //  5 : "RECALIBRATING_PEDALS"
  uint8_t vehicle_state;                     //      Bits= 8

  //  0 : "NOT_CONNECTED"
  //  1 : "NOT_ENABLED_NO_HV_PRESENT"
  //  2 : "NOT_ENABLED_HV_PRESENT"
  //  3 : "INVERTERS_READY"
  //  4 : "INVERTERS_HV_ENABLED"
  //  5 : "ENABLED_DRIVE_MODE"
  //  6 : "ERROR"
  //  7 : "CLEARING_ERRORS"
  uint8_t drivetrain_state;                  //      Bits= 8

  uint8_t drivebrain_in_control : 1;         //      Bits= 1

#else

  //  0 : "TRACTIVE_SYSTEM_NOT_ACTIVE"
  //  1 : "TRACTIVE_SYSTEM_ACTIVE"
  //  2 : "WANTING_READY_TO_DRIVE"
  //  3 : "READY_TO_DRIVE"
  //  4 : "WANTING_RECALIBRATE_PEDALS"
  //  5 : "RECALIBRATING_PEDALS"
  uint8_t vehicle_state;                     //      Bits= 8

  //  0 : "NOT_CONNECTED"
  //  1 : "NOT_ENABLED_NO_HV_PRESENT"
  //  2 : "NOT_ENABLED_HV_PRESENT"
  //  3 : "INVERTERS_READY"
  //  4 : "INVERTERS_HV_ENABLED"
  //  5 : "ENABLED_DRIVE_MODE"
  //  6 : "ERROR"
  //  7 : "CLEARING_ERRORS"
  uint8_t drivetrain_state;                  //      Bits= 8

  uint8_t drivebrain_in_control;             //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CAR_STATES_t;

// def @DRIVEBRAIN_STATE_DATA CAN Message (553  0x229)
#define DRIVEBRAIN_STATE_DATA_IDE (0U)
#define DRIVEBRAIN_STATE_DATA_DLC (1U)
#define DRIVEBRAIN_STATE_DATA_CANID (0x229U)

// Value tables for @vn_gps_status signal

#ifndef vn_gps_status_DRIVEBRAIN_STATE_DATA_NO_FIX
#define vn_gps_status_DRIVEBRAIN_STATE_DATA_NO_FIX (0)
#endif

#ifndef vn_gps_status_DRIVEBRAIN_STATE_DATA_TIME_ONLY
#define vn_gps_status_DRIVEBRAIN_STATE_DATA_TIME_ONLY (1)
#endif

#ifndef vn_gps_status_DRIVEBRAIN_STATE_DATA_FIX_2D
#define vn_gps_status_DRIVEBRAIN_STATE_DATA_FIX_2D (2)
#endif

#ifndef vn_gps_status_DRIVEBRAIN_STATE_DATA_FIX_3D
#define vn_gps_status_DRIVEBRAIN_STATE_DATA_FIX_3D (3)
#endif


typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // GPS fix status
  //  0 : "NO_FIX"
  //  1 : "TIME_ONLY"
  //  2 : "FIX_2D"
  //  3 : "FIX_3D"
  uint8_t vn_gps_status;                     //      Bits= 8

#else

  // GPS fix status
  //  0 : "NO_FIX"
  //  1 : "TIME_ONLY"
  //  2 : "FIX_2D"
  //  3 : "FIX_3D"
  uint8_t vn_gps_status;                     //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DRIVEBRAIN_STATE_DATA_t;

// def @DASH_INPUT CAN Message (768  0x300)
#define DASH_INPUT_IDE (0U)
#define DASH_INPUT_DLC (2U)
#define DASH_INPUT_CANID (0x300U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t led_dimmer_button : 1;               //      Bits= 1

  uint8_t preset_button : 1;                   //      Bits= 1

  uint8_t motor_controller_cycle_button : 1;   //      Bits= 1

  uint8_t mode_button : 1;                     //      Bits= 1

  uint8_t start_button : 1;                    //      Bits= 1

  uint8_t data_button_is_pressed : 1;          //      Bits= 1

  uint8_t left_shifter_button : 1;             //      Bits= 1

  uint8_t right_shifter_button : 1;            //      Bits= 1

  // Dashboard dial position
  uint8_t dash_dial_mode;                      //      Bits= 8

#else

  uint8_t led_dimmer_button;                   //      Bits= 1

  uint8_t preset_button;                       //      Bits= 1

  uint8_t motor_controller_cycle_button;       //      Bits= 1

  uint8_t mode_button;                         //      Bits= 1

  uint8_t start_button;                        //      Bits= 1

  uint8_t data_button_is_pressed;              //      Bits= 1

  uint8_t left_shifter_button;                 //      Bits= 1

  uint8_t right_shifter_button;                //      Bits= 1

  // Dashboard dial position
  uint8_t dash_dial_mode;                      //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DASH_INPUT_t;

// def @AERO_PRESSURE_SENSOR_22 CAN Message (800  0x320)
#define AERO_PRESSURE_SENSOR_22_IDE (0U)
#define AERO_PRESSURE_SENSOR_22_DLC (8U)
#define AERO_PRESSURE_SENSOR_22_CANID (0x320U)
// signal: @aero_channel_4_ro
#define HYTECH_aero_channel_4_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_4_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_4_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_5_ro
#define HYTECH_aero_channel_5_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_5_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_5_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_6_ro
#define HYTECH_aero_channel_6_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_6_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_6_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_7_ro
#define HYTECH_aero_channel_7_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_7_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_7_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_22_t;

// def @AERO_PRESSURE_SENSOR_32 CAN Message (816  0x330)
#define AERO_PRESSURE_SENSOR_32_IDE (0U)
#define AERO_PRESSURE_SENSOR_32_DLC (8U)
#define AERO_PRESSURE_SENSOR_32_CANID (0x330U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_32_t;

// def @AERO_PRESSURE_SENSOR_42 CAN Message (832  0x340)
#define AERO_PRESSURE_SENSOR_42_IDE (0U)
#define AERO_PRESSURE_SENSOR_42_DLC (8U)
#define AERO_PRESSURE_SENSOR_42_CANID (0x340U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_42_t;

// def @AERO_PRESSURE_SENSOR_11 CAN Message (1008 0x3f0)
#define AERO_PRESSURE_SENSOR_11_IDE (0U)
#define AERO_PRESSURE_SENSOR_11_DLC (8U)
#define AERO_PRESSURE_SENSOR_11_CANID (0x3f0U)
// signal: @aero_channel_0_ro
#define HYTECH_aero_channel_0_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_0_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_0_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_1_ro
#define HYTECH_aero_channel_1_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_1_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_1_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_2_ro
#define HYTECH_aero_channel_2_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_2_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_2_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )
// signal: @aero_channel_3_ro
#define HYTECH_aero_channel_3_ro_CovFactor (0.142857142)
#define HYTECH_aero_channel_3_ro_toS(x) ( (uint16_t) (((x) - (-53000.0)) / (0.142857142)) )
#define HYTECH_aero_channel_3_ro_fromS(x) ( (((x) * (0.142857142)) + (-53000.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_11_t;

// def @AERO_PRESSURE_SENSOR_12 CAN Message (1012 0x3f4)
#define AERO_PRESSURE_SENSOR_12_IDE (0U)
#define AERO_PRESSURE_SENSOR_12_DLC (8U)
#define AERO_PRESSURE_SENSOR_12_CANID (0x3f4U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_4_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_5_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_6_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_7_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_12_t;

// Forwarded directly from the Energy Meter with no modifications. See datasheet for more specifications.
// def @EM_STATUS CAN Message (1024 0x400)
#define EM_STATUS_IDE (0U)
#define EM_STATUS_DLC (2U)
#define EM_STATUS_CANID (0x400U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t voltage_gain : 4;                  //      Bits= 4 Unit:'none'

  uint8_t current_gain : 4;                  //      Bits= 4 Unit:'none'

  // Whether or not the energy meter is reading an over-voltage error.
  uint8_t overvoltage_error : 1;             //      Bits= 1 Unit:'none'

  // Whether or not the Energy Meter is reading an over-power error.
  uint8_t overpower_error : 1;               //      Bits= 1 Unit:'none'

  // Whether or not the energy meter is currently logging data.
  uint8_t logging : 1;                       //      Bits= 1 Unit:'none'

#else

  uint8_t voltage_gain;                      //      Bits= 4 Unit:'none'

  uint8_t current_gain;                      //      Bits= 4 Unit:'none'

  // Whether or not the energy meter is reading an over-voltage error.
  uint8_t overvoltage_error;                 //      Bits= 1 Unit:'none'

  // Whether or not the Energy Meter is reading an over-power error.
  uint8_t overpower_error;                   //      Bits= 1 Unit:'none'

  // Whether or not the energy meter is currently logging data.
  uint8_t logging;                           //      Bits= 1 Unit:'none'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} EM_STATUS_t;

// def @ACU_SHUNT_MEASUREMENTS CAN Message (1025 0x401)
#define ACU_SHUNT_MEASUREMENTS_IDE (0U)
#define ACU_SHUNT_MEASUREMENTS_DLC (6U)
#define ACU_SHUNT_MEASUREMENTS_CANID (0x401U)
// signal: @current_shunt_read_ro
#define HYTECH_current_shunt_read_ro_CovFactor (0.291373985)
#define HYTECH_current_shunt_read_ro_toS(x) ( (uint16_t) (((x) - (-666.0)) / (0.291373985)) )
#define HYTECH_current_shunt_read_ro_fromS(x) ( (((x) * (0.291373985)) + (-666.0)) )
// signal: @pack_filtered_read_ro
#define HYTECH_pack_filtered_read_ro_CovFactor (0.168693511)
#define HYTECH_pack_filtered_read_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.168693511)) )
#define HYTECH_pack_filtered_read_ro_fromS(x) ( (((x) * (0.168693511)) + (0.0)) )
// signal: @ts_out_filtered_read_ro
#define HYTECH_ts_out_filtered_read_ro_CovFactor (0.169047463)
#define HYTECH_ts_out_filtered_read_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.169047463)) )
#define HYTECH_ts_out_filtered_read_ro_fromS(x) ( (((x) * (0.169047463)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t current_shunt_read_ro;            //      Bits=16 Offset= -666.0             Factor= 0.291373985     Unit:'amps'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t current_shunt_read_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t pack_filtered_read_ro;            //      Bits=16 Factor= 0.168693511     Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pack_filtered_read_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t ts_out_filtered_read_ro;          //      Bits=16 Factor= 0.169047463     Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t ts_out_filtered_read_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t current_shunt_read_ro;            //      Bits=16 Offset= -666.0             Factor= 0.291373985     Unit:'amps'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t current_shunt_read_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t pack_filtered_read_ro;            //      Bits=16 Factor= 0.168693511     Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pack_filtered_read_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t ts_out_filtered_read_ro;          //      Bits=16 Factor= 0.169047463     Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t ts_out_filtered_read_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} ACU_SHUNT_MEASUREMENTS_t;

// def @ENERGY_METER_STATUS CAN Message (1037 0x40d)
#define ENERGY_METER_STATUS_IDE (0U)
#define ENERGY_METER_STATUS_DLC (8U)
#define ENERGY_METER_STATUS_CANID (0x40dU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t em_violation : 1;                  //      Bits= 1

  uint8_t em_logging : 1;                    //      Bits= 1

  uint32_t em_energy_w_hr;                   //      Bits=32

#else

  uint8_t em_violation;                      //      Bits= 1

  uint8_t em_logging;                        //      Bits= 1

  uint32_t em_energy_w_hr;                   //      Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} ENERGY_METER_STATUS_t;

// def @STATE_OF_CHARGE CAN Message (1054 0x41e)
#define STATE_OF_CHARGE_IDE (0U)
#define STATE_OF_CHARGE_DLC (8U)
#define STATE_OF_CHARGE_CANID (0x41eU)
// signal: @charge_percentage_ro
#define HYTECH_charge_percentage_ro_CovFactor (0.01)
#define HYTECH_charge_percentage_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_charge_percentage_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @min_cell_voltage_est_ro
#define HYTECH_min_cell_voltage_est_ro_CovFactor (0.0001)
#define HYTECH_min_cell_voltage_est_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_min_cell_voltage_est_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @charge_coulombs_ro
#define HYTECH_charge_coulombs_ro_CovFactor (0.0001)
#define HYTECH_charge_coulombs_ro_toS(x) ( (uint32_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_charge_coulombs_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t charge_percentage_ro;             //      Bits=16 Factor= 0.01            Unit:'percent'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t charge_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t min_cell_voltage_est_ro;          //      Bits=16 Factor= 0.0001          Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t min_cell_voltage_est_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint32_t charge_coulombs_ro;               //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t charge_coulombs_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t charge_percentage_ro;             //      Bits=16 Factor= 0.01            Unit:'percent'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t charge_percentage_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t min_cell_voltage_est_ro;          //      Bits=16 Factor= 0.0001          Unit:'volts'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t min_cell_voltage_est_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint32_t charge_coulombs_ro;               //      Bits=32 Factor= 0.0001          Unit:'Coulombs'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t charge_coulombs_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} STATE_OF_CHARGE_t;

// Steering reading; system and sensor status
// def @STEERING_DATA CAN Message (1055 0x41f)
#define STEERING_DATA_IDE (0U)
#define STEERING_DATA_DLC (6U)
#define STEERING_DATA_CANID (0x41fU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // raw measurement as measured by bottom steering analog sensor
  uint16_t steering_analog_raw;              //      Bits=16

  // raw measurement by digital steering encoder
  uint32_t steering_digital_raw;             //      Bits=32

#else

  // raw measurement as measured by bottom steering analog sensor
  uint16_t steering_analog_raw;              //      Bits=16

  // raw measurement by digital steering encoder
  uint32_t steering_digital_raw;             //      Bits=32

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} STEERING_DATA_t;

// def @LF_TTPMS_1 CAN Message (1060 0x424)
#define LF_TTPMS_1_IDE (0U)
#define LF_TTPMS_1_DLC (8U)
#define LF_TTPMS_1_CANID (0x424U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_SN;                      //      Bits=16

  uint16_t LF_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t LF_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t LF_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#else

  uint16_t LF_TTPMS_SN;                      //      Bits=16

  uint16_t LF_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t LF_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t LF_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_1_t;

// def @LF_TTPMS_2 CAN Message (1061 0x425)
#define LF_TTPMS_2_IDE (0U)
#define LF_TTPMS_2_DLC (8U)
#define LF_TTPMS_2_CANID (0x425U)
// signal: @LF_TTPMS_T1_ro
#define HYTECH_LF_TTPMS_T1_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T1_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T1_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T2_ro
#define HYTECH_LF_TTPMS_T2_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T2_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T2_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T3_ro
#define HYTECH_LF_TTPMS_T3_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T3_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T3_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T4_ro
#define HYTECH_LF_TTPMS_T4_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T4_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T4_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LF_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_2_t;

// def @LF_TTPMS_3 CAN Message (1062 0x426)
#define LF_TTPMS_3_IDE (0U)
#define LF_TTPMS_3_DLC (8U)
#define LF_TTPMS_3_CANID (0x426U)
// signal: @LF_TTPMS_T5_ro
#define HYTECH_LF_TTPMS_T5_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T5_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T5_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T6_ro
#define HYTECH_LF_TTPMS_T6_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T6_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T6_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T7_ro
#define HYTECH_LF_TTPMS_T7_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T7_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T7_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T8_ro
#define HYTECH_LF_TTPMS_T8_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T8_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T8_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LF_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_3_t;

// def @LF_TTPMS_4 CAN Message (1063 0x427)
#define LF_TTPMS_4_IDE (0U)
#define LF_TTPMS_4_DLC (8U)
#define LF_TTPMS_4_CANID (0x427U)
// signal: @LF_TTPMS_T9_ro
#define HYTECH_LF_TTPMS_T9_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T9_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T9_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T10_ro
#define HYTECH_LF_TTPMS_T10_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T10_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T10_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T11_ro
#define HYTECH_LF_TTPMS_T11_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T11_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T11_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T12_ro
#define HYTECH_LF_TTPMS_T12_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T12_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T12_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LF_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_4_t;

// def @LF_TTPMS_5 CAN Message (1064 0x428)
#define LF_TTPMS_5_IDE (0U)
#define LF_TTPMS_5_DLC (8U)
#define LF_TTPMS_5_CANID (0x428U)
// signal: @LF_TTPMS_T13_ro
#define HYTECH_LF_TTPMS_T13_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T13_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T13_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T14_ro
#define HYTECH_LF_TTPMS_T14_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T14_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T14_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T15_ro
#define HYTECH_LF_TTPMS_T15_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T15_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T15_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LF_TTPMS_T16_ro
#define HYTECH_LF_TTPMS_T16_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T16_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T16_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LF_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_5_t;

// def @LF_TTPMS_6 CAN Message (1065 0x429)
#define LF_TTPMS_6_IDE (0U)
#define LF_TTPMS_6_DLC (8U)
#define LF_TTPMS_6_CANID (0x429U)
// signal: @LF_TTPMS_T_ro
#define HYTECH_LF_TTPMS_T_ro_CovFactor (0.1)
#define HYTECH_LF_TTPMS_T_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LF_TTPMS_T_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LF_TTPMS_TC;                      //      Bits=16

  int16_t LF_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t LF_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_NODE_ID;                 //      Bits=16

#else

  uint16_t LF_TTPMS_TC;                      //      Bits=16

  int16_t LF_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t LF_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LF_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LF_TTPMS_NODE_ID;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LF_TTPMS_6_t;

// def @LR_TTPMS_1 CAN Message (1066 0x42a)
#define LR_TTPMS_1_IDE (0U)
#define LR_TTPMS_1_DLC (8U)
#define LR_TTPMS_1_CANID (0x42aU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // Serial Number
  uint16_t LR_TTPMS_SN;                      //      Bits=16

  uint16_t LR_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t LR_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t LR_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#else

  // Serial Number
  uint16_t LR_TTPMS_SN;                      //      Bits=16

  uint16_t LR_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t LR_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t LR_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_1_t;

// def @LR_TTPMS_2 CAN Message (1067 0x42b)
#define LR_TTPMS_2_IDE (0U)
#define LR_TTPMS_2_DLC (8U)
#define LR_TTPMS_2_CANID (0x42bU)
// signal: @LR_TTPMS_T1_ro
#define HYTECH_LR_TTPMS_T1_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T1_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T1_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T2_ro
#define HYTECH_LR_TTPMS_T2_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T2_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T2_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T3_ro
#define HYTECH_LR_TTPMS_T3_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T3_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T3_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T4_ro
#define HYTECH_LR_TTPMS_T4_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T4_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T4_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LR_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LR_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_2_t;

// def @LR_TTPMS_3 CAN Message (1068 0x42c)
#define LR_TTPMS_3_IDE (0U)
#define LR_TTPMS_3_DLC (8U)
#define LR_TTPMS_3_CANID (0x42cU)
// signal: @LR_TTPMS_T5_ro
#define HYTECH_LR_TTPMS_T5_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T5_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T5_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T6_ro
#define HYTECH_LR_TTPMS_T6_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T6_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T6_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T7_ro
#define HYTECH_LR_TTPMS_T7_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T7_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T7_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T8_ro
#define HYTECH_LR_TTPMS_T8_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T8_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T8_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LR_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LR_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_3_t;

// def @LR_TTPMS_4 CAN Message (1069 0x42d)
#define LR_TTPMS_4_IDE (0U)
#define LR_TTPMS_4_DLC (8U)
#define LR_TTPMS_4_CANID (0x42dU)
// signal: @LR_TTPMS_T9_ro
#define HYTECH_LR_TTPMS_T9_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T9_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T9_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T10_ro
#define HYTECH_LR_TTPMS_T10_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T10_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T10_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T11_ro
#define HYTECH_LR_TTPMS_T11_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T11_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T11_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T12_ro
#define HYTECH_LR_TTPMS_T12_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T12_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T12_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LR_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LR_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_4_t;

// def @LR_TTPMS_5 CAN Message (1070 0x42e)
#define LR_TTPMS_5_IDE (0U)
#define LR_TTPMS_5_DLC (8U)
#define LR_TTPMS_5_CANID (0x42eU)
// signal: @LR_TTPMS_T13_ro
#define HYTECH_LR_TTPMS_T13_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T13_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T13_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T14_ro
#define HYTECH_LR_TTPMS_T14_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T14_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T14_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T15_ro
#define HYTECH_LR_TTPMS_T15_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T15_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T15_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @LR_TTPMS_T16_ro
#define HYTECH_LR_TTPMS_T16_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T16_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T16_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LR_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t LR_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_5_t;

// def @LR_TTPMS_6 CAN Message (1071 0x42f)
#define LR_TTPMS_6_IDE (0U)
#define LR_TTPMS_6_DLC (8U)
#define LR_TTPMS_6_CANID (0x42fU)
// signal: @LR_TTPMS_T_ro
#define HYTECH_LR_TTPMS_T_ro_CovFactor (0.1)
#define HYTECH_LR_TTPMS_T_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_LR_TTPMS_T_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t LR_TTPMS_TC;                      //      Bits=16

  int16_t LR_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t LR_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_NODE_ID;                 //      Bits=16

#else

  uint16_t LR_TTPMS_TC;                      //      Bits=16

  int16_t LR_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t LR_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t LR_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t LR_TTPMS_NODE_ID;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} LR_TTPMS_6_t;

// def @RF_TTPMS_1 CAN Message (1072 0x430)
#define RF_TTPMS_1_IDE (0U)
#define RF_TTPMS_1_DLC (8U)
#define RF_TTPMS_1_CANID (0x430U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_SN;                      //      Bits=16

  uint16_t RF_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t RF_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t RF_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#else

  uint16_t RF_TTPMS_SN;                      //      Bits=16

  uint16_t RF_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t RF_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t RF_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_1_t;

// def @RF_TTPMS_2 CAN Message (1073 0x431)
#define RF_TTPMS_2_IDE (0U)
#define RF_TTPMS_2_DLC (8U)
#define RF_TTPMS_2_CANID (0x431U)
// signal: @RF_TTPMS_T1_ro
#define HYTECH_RF_TTPMS_T1_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T1_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T1_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T2_ro
#define HYTECH_RF_TTPMS_T2_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T2_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T2_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T3_ro
#define HYTECH_RF_TTPMS_T3_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T3_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T3_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T4_ro
#define HYTECH_RF_TTPMS_T4_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T4_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T4_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RF_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_2_t;

// def @RF_TTPMS_3 CAN Message (1074 0x432)
#define RF_TTPMS_3_IDE (0U)
#define RF_TTPMS_3_DLC (8U)
#define RF_TTPMS_3_CANID (0x432U)
// signal: @RF_TTPMS_T5_ro
#define HYTECH_RF_TTPMS_T5_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T5_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T5_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T6_ro
#define HYTECH_RF_TTPMS_T6_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T6_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T6_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T7_ro
#define HYTECH_RF_TTPMS_T7_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T7_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T7_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T8_ro
#define HYTECH_RF_TTPMS_T8_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T8_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T8_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RF_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_3_t;

// def @RF_TTPMS_4 CAN Message (1075 0x433)
#define RF_TTPMS_4_IDE (0U)
#define RF_TTPMS_4_DLC (8U)
#define RF_TTPMS_4_CANID (0x433U)
// signal: @RF_TTPMS_T9_ro
#define HYTECH_RF_TTPMS_T9_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T9_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T9_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T10_ro
#define HYTECH_RF_TTPMS_T10_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T10_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T10_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T11_ro
#define HYTECH_RF_TTPMS_T11_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T11_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T11_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T12_ro
#define HYTECH_RF_TTPMS_T12_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T12_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T12_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RF_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_4_t;

// def @RF_TTPMS_5 CAN Message (1076 0x434)
#define RF_TTPMS_5_IDE (0U)
#define RF_TTPMS_5_DLC (8U)
#define RF_TTPMS_5_CANID (0x434U)
// signal: @RF_TTPMS_T13_ro
#define HYTECH_RF_TTPMS_T13_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T13_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T13_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T14_ro
#define HYTECH_RF_TTPMS_T14_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T14_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T14_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T15_ro
#define HYTECH_RF_TTPMS_T15_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T15_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T15_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RF_TTPMS_T16_ro
#define HYTECH_RF_TTPMS_T16_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T16_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T16_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RF_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_5_t;

// def @RF_TTPMS_6 CAN Message (1077 0x435)
#define RF_TTPMS_6_IDE (0U)
#define RF_TTPMS_6_DLC (8U)
#define RF_TTPMS_6_CANID (0x435U)
// signal: @RF_TTPMS_T_ro
#define HYTECH_RF_TTPMS_T_ro_CovFactor (0.1)
#define HYTECH_RF_TTPMS_T_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RF_TTPMS_T_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RF_TTPMS_TC;                      //      Bits=16

  int16_t RF_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t RF_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_NODE_ID;                 //      Bits=16

#else

  uint16_t RF_TTPMS_TC;                      //      Bits=16

  int16_t RF_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t RF_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RF_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RF_TTPMS_NODE_ID;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RF_TTPMS_6_t;

// def @RR_TTPMS_1 CAN Message (1078 0x436)
#define RR_TTPMS_1_IDE (0U)
#define RR_TTPMS_1_DLC (8U)
#define RR_TTPMS_1_CANID (0x436U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_SN;                      //      Bits=16

  uint16_t RR_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t RR_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t RR_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#else

  uint16_t RR_TTPMS_SN;                      //      Bits=16

  uint16_t RR_TTPMS_BAT_V;                   //      Bits=16 Unit:'mV'

  uint16_t RR_TTPMS_P;                       //      Bits=16 Unit:'mbar'

  uint16_t RR_TTPMS_P_GAUGE;                 //      Bits=16 Unit:'mbar'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_1_t;

// def @RR_TTPMS_2 CAN Message (1079 0x437)
#define RR_TTPMS_2_IDE (0U)
#define RR_TTPMS_2_DLC (8U)
#define RR_TTPMS_2_CANID (0x437U)
// signal: @RR_TTPMS_T1_ro
#define HYTECH_RR_TTPMS_T1_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T1_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T1_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T2_ro
#define HYTECH_RR_TTPMS_T2_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T2_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T2_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T3_ro
#define HYTECH_RR_TTPMS_T3_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T3_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T3_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T4_ro
#define HYTECH_RR_TTPMS_T4_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T4_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T4_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RR_TTPMS_T1_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T2_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T3_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T4_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_2_t;

// def @RR_TTPMS_3 CAN Message (1080 0x438)
#define RR_TTPMS_3_IDE (0U)
#define RR_TTPMS_3_DLC (8U)
#define RR_TTPMS_3_CANID (0x438U)
// signal: @RR_TTPMS_T5_ro
#define HYTECH_RR_TTPMS_T5_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T5_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T5_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T6_ro
#define HYTECH_RR_TTPMS_T6_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T6_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T6_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T7_ro
#define HYTECH_RR_TTPMS_T7_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T7_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T7_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T8_ro
#define HYTECH_RR_TTPMS_T8_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T8_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T8_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RR_TTPMS_T5_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T6_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T7_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T8_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_3_t;

// def @RR_TTPMS_4 CAN Message (1081 0x439)
#define RR_TTPMS_4_IDE (0U)
#define RR_TTPMS_4_DLC (8U)
#define RR_TTPMS_4_CANID (0x439U)
// signal: @RR_TTPMS_T9_ro
#define HYTECH_RR_TTPMS_T9_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T9_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T9_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T10_ro
#define HYTECH_RR_TTPMS_T10_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T10_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T10_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T11_ro
#define HYTECH_RR_TTPMS_T11_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T11_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T11_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T12_ro
#define HYTECH_RR_TTPMS_T12_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T12_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T12_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RR_TTPMS_T9_ro;                   //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T10_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T11_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T12_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_4_t;

// def @RR_TTPMS_5 CAN Message (1082 0x43a)
#define RR_TTPMS_5_IDE (0U)
#define RR_TTPMS_5_DLC (8U)
#define RR_TTPMS_5_CANID (0x43aU)
// signal: @RR_TTPMS_T13_ro
#define HYTECH_RR_TTPMS_T13_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T13_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T13_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T14_ro
#define HYTECH_RR_TTPMS_T14_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T14_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T14_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T15_ro
#define HYTECH_RR_TTPMS_T15_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T15_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T15_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @RR_TTPMS_T16_ro
#define HYTECH_RR_TTPMS_T16_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T16_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T16_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t RR_TTPMS_T13_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T14_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T15_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_T16_ro;                  //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_5_t;

// def @RR_TTPMS_6 CAN Message (1083 0x43b)
#define RR_TTPMS_6_IDE (0U)
#define RR_TTPMS_6_DLC (8U)
#define RR_TTPMS_6_CANID (0x43bU)
// signal: @RR_TTPMS_T_ro
#define HYTECH_RR_TTPMS_T_ro_CovFactor (0.1)
#define HYTECH_RR_TTPMS_T_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_RR_TTPMS_T_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t RR_TTPMS_TC;                      //      Bits=16

  int16_t RR_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t RR_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_NODE_ID;                 //      Bits=16

#else

  uint16_t RR_TTPMS_TC;                      //      Bits=16

  int16_t RR_TTPMS_RSSI;                     //  [-] Bits=16 Unit:'dBm'

  uint16_t RR_TTPMS_T_ro;                    //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'°C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t RR_TTPMS_T_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t RR_TTPMS_NODE_ID;                 //      Bits=16

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} RR_TTPMS_6_t;

// def @BRAKE_ROTOR_SENSOR_COMMAND CAN Message (1219 0x4c3)
#define BRAKE_ROTOR_SENSOR_COMMAND_IDE (0U)
#define BRAKE_ROTOR_SENSOR_COMMAND_DLC (8U)
#define BRAKE_ROTOR_SENSOR_COMMAND_CANID (0x4c3U)
// signal: @brake_temp_sensor_emissivity_ro
#define HYTECH_brake_temp_sensor_emissivity_ro_CovFactor (0.01)
#define HYTECH_brake_temp_sensor_emissivity_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_brake_temp_sensor_emissivity_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // programming constant, should be set to 30000
  uint16_t brake_temp_sensor_prog_const;         //      Bits=16

  uint16_t brake_temp_sensor_base_can_id;        //      Bits=16

  uint8_t brake_temp_sensor_emissivity_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_sensor_emissivity_phys;
#endif // HYTECH_USE_SIGFLOAT

  // 1 = 1 hz, 2 = 2hz, 3 = 4hz, 4 = 8 hz, 5=16 hz, 6=32 hz, 7 = 64hz, 8 = 100hz
  uint8_t brake_temp_sensor_sampling_freq;       //      Bits= 8

  // 40 = 4 Channels, 80 = 8 channels, 160 = 16 channels
  uint8_t brake_temp_sensor_ch_setting;          //      Bits= 8

  // bitrate setting for CAN on the brake temp sensor. 1 = 1M, 2 = 500k, 3 = 250k, 4 = 100k
  uint8_t brake_temp_sensor_can_bitrate;         //      Bits= 8

#else

  // programming constant, should be set to 30000
  uint16_t brake_temp_sensor_prog_const;         //      Bits=16

  uint16_t brake_temp_sensor_base_can_id;        //      Bits=16

  uint8_t brake_temp_sensor_emissivity_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_sensor_emissivity_phys;
#endif // HYTECH_USE_SIGFLOAT

  // 1 = 1 hz, 2 = 2hz, 3 = 4hz, 4 = 8 hz, 5=16 hz, 6=32 hz, 7 = 64hz, 8 = 100hz
  uint8_t brake_temp_sensor_sampling_freq;       //      Bits= 8

  // 40 = 4 Channels, 80 = 8 channels, 160 = 16 channels
  uint8_t brake_temp_sensor_ch_setting;          //      Bits= 8

  // bitrate setting for CAN on the brake temp sensor. 1 = 1M, 2 = 500k, 3 = 250k, 4 = 100k
  uint8_t brake_temp_sensor_can_bitrate;         //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} BRAKE_ROTOR_SENSOR_COMMAND_t;

// def @FL_BRAKE_ROTOR_TEMP_CH1_CH4 CAN Message (1220 0x4c4)
#define FL_BRAKE_ROTOR_TEMP_CH1_CH4_IDE (0U)
#define FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC (8U)
#define FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID (0x4c4U)
// signal: @brake_temp_channel_1_ro
#define HYTECH_brake_temp_channel_1_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_1_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_1_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_2_ro
#define HYTECH_brake_temp_channel_2_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_2_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_2_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_3_ro
#define HYTECH_brake_temp_channel_3_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_3_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_3_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_4_ro
#define HYTECH_brake_temp_channel_4_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_4_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_4_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_1_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_2_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_3_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_4_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_1_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_2_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_3_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_4_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FL_BRAKE_ROTOR_TEMP_CH1_CH4_t;

// def @FL_BRAKE_ROTOR_TEMP_CH5_CH8 CAN Message (1221 0x4c5)
#define FL_BRAKE_ROTOR_TEMP_CH5_CH8_IDE (0U)
#define FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC (8U)
#define FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID (0x4c5U)
// signal: @brake_temp_channel_5_ro
#define HYTECH_brake_temp_channel_5_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_5_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_5_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_6_ro
#define HYTECH_brake_temp_channel_6_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_6_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_6_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_7_ro
#define HYTECH_brake_temp_channel_7_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_7_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_7_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_8_ro
#define HYTECH_brake_temp_channel_8_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_8_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_8_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_5_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_6_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_7_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_8_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_5_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_6_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_7_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_8_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FL_BRAKE_ROTOR_TEMP_CH5_CH8_t;

// def @FL_BRAKE_ROTOR_TEMP_CH9_CH12 CAN Message (1222 0x4c6)
#define FL_BRAKE_ROTOR_TEMP_CH9_CH12_IDE (0U)
#define FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC (8U)
#define FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID (0x4c6U)
// signal: @brake_temp_channel_9_ro
#define HYTECH_brake_temp_channel_9_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_9_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_9_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_10_ro
#define HYTECH_brake_temp_channel_10_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_10_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_10_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_11_ro
#define HYTECH_brake_temp_channel_11_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_11_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_11_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_12_ro
#define HYTECH_brake_temp_channel_12_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_12_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_12_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_9_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_10_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_11_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_12_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_9_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_10_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_11_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_12_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FL_BRAKE_ROTOR_TEMP_CH9_CH12_t;

// def @FL_BRAKE_ROTOR_TEMP_CH13_CH16 CAN Message (1223 0x4c7)
#define FL_BRAKE_ROTOR_TEMP_CH13_CH16_IDE (0U)
#define FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC (8U)
#define FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID (0x4c7U)
// signal: @brake_temp_channel_13_ro
#define HYTECH_brake_temp_channel_13_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_13_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_13_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_14_ro
#define HYTECH_brake_temp_channel_14_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_14_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_14_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_15_ro
#define HYTECH_brake_temp_channel_15_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_15_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_15_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )
// signal: @brake_temp_channel_16_ro
#define HYTECH_brake_temp_channel_16_ro_CovFactor (0.1)
#define HYTECH_brake_temp_channel_16_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_temp_channel_16_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_13_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_14_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_15_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_16_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_13_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_14_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_15_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_16_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FL_BRAKE_ROTOR_TEMP_CH13_CH16_t;

// def @FL_BRAKE_ROTOR_SENSOR_TEMP CAN Message (1224 0x4c8)
#define FL_BRAKE_ROTOR_SENSOR_TEMP_IDE (0U)
#define FL_BRAKE_ROTOR_SENSOR_TEMP_DLC (8U)
#define FL_BRAKE_ROTOR_SENSOR_TEMP_CANID (0x4c8U)
// signal: @brake_rotor_sensor_temp_ro
#define HYTECH_brake_rotor_sensor_temp_ro_CovFactor (0.1)
#define HYTECH_brake_rotor_sensor_temp_ro_toS(x) ( (uint16_t) (((x) - (-100.0)) / (0.1)) )
#define HYTECH_brake_rotor_sensor_temp_ro_fromS(x) ( (((x) * (0.1)) + (-100.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_rotor_sensor_temp_ro;       //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_rotor_sensor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_rotor_sensor_temp_ro;       //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_rotor_sensor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FL_BRAKE_ROTOR_SENSOR_TEMP_t;

// def @FR_BRAKE_ROTOR_TEMP_CH1_CH4 CAN Message (1225 0x4c9)
#define FR_BRAKE_ROTOR_TEMP_CH1_CH4_IDE (0U)
#define FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC (8U)
#define FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID (0x4c9U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_1_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_2_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_3_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_4_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_1_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_2_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_3_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_4_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FR_BRAKE_ROTOR_TEMP_CH1_CH4_t;

// def @FR_BRAKE_ROTOR_TEMP_CH5_CH8 CAN Message (1226 0x4ca)
#define FR_BRAKE_ROTOR_TEMP_CH5_CH8_IDE (0U)
#define FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC (8U)
#define FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID (0x4caU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_5_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_6_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_7_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_8_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_8_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_5_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_5_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_6_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_6_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_7_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_7_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_8_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_8_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FR_BRAKE_ROTOR_TEMP_CH5_CH8_t;

// def @FR_BRAKE_ROTOR_TEMP_CH9_CH12 CAN Message (1227 0x4cb)
#define FR_BRAKE_ROTOR_TEMP_CH9_CH12_IDE (0U)
#define FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC (8U)
#define FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID (0x4cbU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_9_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_10_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_11_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_12_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_12_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_9_ro;          //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_9_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_10_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_10_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_11_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_11_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_12_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_12_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FR_BRAKE_ROTOR_TEMP_CH9_CH12_t;

// def @FR_BRAKE_ROTOR_TEMP_CH13_CH16 CAN Message (1228 0x4cc)
#define FR_BRAKE_ROTOR_TEMP_CH13_CH16_IDE (0U)
#define FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC (8U)
#define FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID (0x4ccU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_temp_channel_13_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_14_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_15_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_16_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_16_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_temp_channel_13_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_13_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_14_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_14_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_15_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_15_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t brake_temp_channel_16_ro;         //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_temp_channel_16_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FR_BRAKE_ROTOR_TEMP_CH13_CH16_t;

// def @FR_BRAKE_ROTOR_SENSOR_TEMP CAN Message (1229 0x4cd)
#define FR_BRAKE_ROTOR_SENSOR_TEMP_IDE (0U)
#define FR_BRAKE_ROTOR_SENSOR_TEMP_DLC (8U)
#define FR_BRAKE_ROTOR_SENSOR_TEMP_CANID (0x4cdU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t brake_rotor_sensor_temp_ro;       //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_rotor_sensor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t brake_rotor_sensor_temp_ro;       //      Bits=16 Offset= -100.0             Factor= 0.1             Unit:'deg C'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t brake_rotor_sensor_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} FR_BRAKE_ROTOR_SENSOR_TEMP_t;

// def @CONTROLLER_PID_TV_DATA CAN Message (1263 0x4ef)
#define CONTROLLER_PID_TV_DATA_IDE (0U)
#define CONTROLLER_PID_TV_DATA_DLC (8U)
#define CONTROLLER_PID_TV_DATA_CANID (0x4efU)
// signal: @controller_input_ro
#define HYTECH_controller_input_ro_CovFactor (0.01)
#define HYTECH_controller_input_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_input_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_output_ro
#define HYTECH_controller_output_ro_CovFactor (0.01)
#define HYTECH_controller_output_ro_toS(x) ( (int32_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_output_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int32_t controller_input_ro;               //  [-] Bits=20 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_input_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t controller_output_ro;              //  [-] Bits=20 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_output_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int32_t controller_input_ro;               //  [-] Bits=20 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_input_phys;
#endif // HYTECH_USE_SIGFLOAT

  int32_t controller_output_ro;              //  [-] Bits=20 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_output_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_PID_TV_DATA_t;

// def @CONTROLLER_PID_TV_DELTA_DATA CAN Message (1264 0x4f0)
#define CONTROLLER_PID_TV_DELTA_DATA_IDE (0U)
#define CONTROLLER_PID_TV_DELTA_DATA_DLC (8U)
#define CONTROLLER_PID_TV_DELTA_DATA_CANID (0x4f0U)
// signal: @pid_tv_fl_delta_ro
#define HYTECH_pid_tv_fl_delta_ro_CovFactor (0.1)
#define HYTECH_pid_tv_fl_delta_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_pid_tv_fl_delta_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @pid_tv_fr_delta_ro
#define HYTECH_pid_tv_fr_delta_ro_CovFactor (0.1)
#define HYTECH_pid_tv_fr_delta_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_pid_tv_fr_delta_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @pid_tv_rl_delta_ro
#define HYTECH_pid_tv_rl_delta_ro_CovFactor (0.1)
#define HYTECH_pid_tv_rl_delta_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_pid_tv_rl_delta_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @pid_tv_rr_delta_ro
#define HYTECH_pid_tv_rr_delta_ro_CovFactor (0.1)
#define HYTECH_pid_tv_rr_delta_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_pid_tv_rr_delta_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t pid_tv_fl_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_fl_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_fr_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_fr_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_rl_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_rl_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_rr_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_rr_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t pid_tv_fl_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_fl_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_fr_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_fr_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_rl_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_rl_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t pid_tv_rr_delta_ro;                //  [-] Bits=10 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t pid_tv_rr_delta_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_PID_TV_DELTA_DATA_t;

// def @TCMUX_STATUS_REPORT CAN Message (1265 0x4f1)
#define TCMUX_STATUS_REPORT_IDE (0U)
#define TCMUX_STATUS_REPORT_DLC (8U)
#define TCMUX_STATUS_REPORT_CANID (0x4f1U)
// signal: @torque_limit_ro
#define HYTECH_torque_limit_ro_CovFactor (0.001)
#define HYTECH_torque_limit_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_torque_limit_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // Vehicle speed is above 5 m/s, TCMux does not allow driver to switch mode
  uint8_t speed_above_thresh : 1;            //      Bits= 1

  // Torque delta between old and new controllers is < 0.5Nm on every wheel
  uint8_t torque_delta_above_thresh : 1;     //      Bits= 1

  // Selected TC not in ready state
  uint8_t tc_not_ready : 1;                  //      Bits= 1

  // Steering system data in ERROR state
  uint8_t steering_system_has_err : 1;       //      Bits= 1

  // TC mode selected by driver
  uint8_t mode_intended;                     //      Bits= 8

  // actual mode in tcmux
  uint8_t mode_actual;                       //      Bits= 8

  // Dashboard dial position
  uint8_t dash_dial_mode;                    //      Bits= 8

  // torque mode
  uint8_t torque_mode;                       //      Bits= 8

  uint16_t torque_limit_ro;                  //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  // Vehicle speed is above 5 m/s, TCMux does not allow driver to switch mode
  uint8_t speed_above_thresh;                //      Bits= 1

  // Torque delta between old and new controllers is < 0.5Nm on every wheel
  uint8_t torque_delta_above_thresh;         //      Bits= 1

  // Selected TC not in ready state
  uint8_t tc_not_ready;                      //      Bits= 1

  // Steering system data in ERROR state
  uint8_t steering_system_has_err;           //      Bits= 1

  // TC mode selected by driver
  uint8_t mode_intended;                     //      Bits= 8

  // actual mode in tcmux
  uint8_t mode_actual;                       //      Bits= 8

  // Dashboard dial position
  uint8_t dash_dial_mode;                    //      Bits= 8

  // torque mode
  uint8_t torque_mode;                       //      Bits= 8

  uint16_t torque_limit_ro;                  //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t torque_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} TCMUX_STATUS_REPORT_t;

// def @ACU_OK CAN Message (1280 0x500)
#define ACU_OK_IDE (0U)
#define ACU_OK_DLC (1U)
#define ACU_OK_CANID (0x500U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t bms_ok : 1;                        //      Bits= 1

  uint8_t imd_ok : 1;                        //      Bits= 1

  uint8_t latch_ok : 1;                      //      Bits= 1

#else

  uint8_t bms_ok;                            //      Bits= 1

  uint8_t imd_ok;                            //      Bits= 1

  uint8_t latch_ok;                          //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} ACU_OK_t;

// def @ENERGY_METER_TEMP CAN Message (1549 0x60d)
#define ENERGY_METER_TEMP_IDE (0U)
#define ENERGY_METER_TEMP_DLC (8U)
#define ENERGY_METER_TEMP_CANID (0x60dU)
// signal: @em_min_temp_ro
#define HYTECH_em_min_temp_ro_CovFactor (0.5)
#define HYTECH_em_min_temp_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_min_temp_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_max_temp_ro
#define HYTECH_em_max_temp_ro_CovFactor (0.5)
#define HYTECH_em_max_temp_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_max_temp_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_temp_0_ro
#define HYTECH_em_temp_0_ro_CovFactor (0.5)
#define HYTECH_em_temp_0_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_temp_0_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_temp_1_ro
#define HYTECH_em_temp_1_ro_CovFactor (0.5)
#define HYTECH_em_temp_1_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_temp_1_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_temp_2_ro
#define HYTECH_em_temp_2_ro_CovFactor (0.5)
#define HYTECH_em_temp_2_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_temp_2_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_temp_3_ro
#define HYTECH_em_temp_3_ro_CovFactor (0.5)
#define HYTECH_em_temp_3_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_temp_3_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )
// signal: @em_temp_4_ro
#define HYTECH_em_temp_4_ro_CovFactor (0.5)
#define HYTECH_em_temp_4_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.5)) )
#define HYTECH_em_temp_4_ro_fromS(x) ( (((x) * (0.5)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t em_temp_sig_index : 3;             //      Bits= 3

  uint8_t em_num_sensors : 5;                //      Bits= 5

  uint8_t em_min_temp_ro;                    //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_min_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_max_temp_ro;                    //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_max_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_0_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_1_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_2_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_3_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_4_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t em_temp_sig_index;                 //      Bits= 3

  uint8_t em_num_sensors;                    //      Bits= 5

  uint8_t em_min_temp_ro;                    //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_min_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_max_temp_ro;                    //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_max_temp_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_0_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_1_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_2_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_3_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_3_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t em_temp_4_ro;                      //      Bits= 8 Factor= 0.5

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t em_temp_4_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} ENERGY_METER_TEMP_t;

// def @AERO_PRESSURE_SENSOR_21 CAN Message (1568 0x620)
#define AERO_PRESSURE_SENSOR_21_IDE (0U)
#define AERO_PRESSURE_SENSOR_21_DLC (8U)
#define AERO_PRESSURE_SENSOR_21_CANID (0x620U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_21_t;

// def @AERO_PRESSURE_SENSOR_31 CAN Message (1584 0x630)
#define AERO_PRESSURE_SENSOR_31_IDE (0U)
#define AERO_PRESSURE_SENSOR_31_DLC (8U)
#define AERO_PRESSURE_SENSOR_31_CANID (0x630U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_31_t;

// def @AERO_PRESSURE_SENSOR_41 CAN Message (1600 0x640)
#define AERO_PRESSURE_SENSOR_41_IDE (0U)
#define AERO_PRESSURE_SENSOR_41_DLC (8U)
#define AERO_PRESSURE_SENSOR_41_CANID (0x640U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t aero_channel_0_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_0_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_1_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_1_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_2_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_2_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t aero_channel_3_ro;                //      Bits=16 Offset= -53000.0           Factor= 0.142857142     Unit:'uBar'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t aero_channel_3_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} AERO_PRESSURE_SENSOR_41_t;

// def @CONTROLLER_TCS_PID_OUTPUT CAN Message (1962 0x7aa)
#define CONTROLLER_TCS_PID_OUTPUT_IDE (0U)
#define CONTROLLER_TCS_PID_OUTPUT_DLC (8U)
#define CONTROLLER_TCS_PID_OUTPUT_CANID (0x7aaU)
// signal: @controller_tcs_pid_output_fl_ro
#define HYTECH_controller_tcs_pid_output_fl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_output_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_output_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_output_fr_ro
#define HYTECH_controller_tcs_pid_output_fr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_output_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_output_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_output_rl_ro
#define HYTECH_controller_tcs_pid_output_rl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_output_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_output_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_output_rr_ro
#define HYTECH_controller_tcs_pid_output_rr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_output_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_output_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_tcs_pid_output_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_tcs_pid_output_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_output_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_output_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_PID_OUTPUT_t;

// def @VEHM_WHEEL_LIN_VEL CAN Message (1963 0x7ab)
#define VEHM_WHEEL_LIN_VEL_IDE (0U)
#define VEHM_WHEEL_LIN_VEL_DLC (8U)
#define VEHM_WHEEL_LIN_VEL_CANID (0x7abU)
// signal: @vehm_wheel_lin_vel_fl_ro
#define HYTECH_vehm_wheel_lin_vel_fl_ro_CovFactor (0.01)
#define HYTECH_vehm_wheel_lin_vel_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_wheel_lin_vel_fl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_wheel_lin_vel_fr_ro
#define HYTECH_vehm_wheel_lin_vel_fr_ro_CovFactor (0.01)
#define HYTECH_vehm_wheel_lin_vel_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_wheel_lin_vel_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_wheel_lin_vel_rl_ro
#define HYTECH_vehm_wheel_lin_vel_rl_ro_CovFactor (0.01)
#define HYTECH_vehm_wheel_lin_vel_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_wheel_lin_vel_rl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_wheel_lin_vel_rr_ro
#define HYTECH_vehm_wheel_lin_vel_rr_ro_CovFactor (0.01)
#define HYTECH_vehm_wheel_lin_vel_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_wheel_lin_vel_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_wheel_lin_vel_fl_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_fr_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_rl_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_rr_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_wheel_lin_vel_fl_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_fr_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_rl_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_wheel_lin_vel_rr_ro;          //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_lin_vel_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_WHEEL_LIN_VEL_t;

// def @CONTROLLER_TORQUE_BIAS CAN Message (1964 0x7ac)
#define CONTROLLER_TORQUE_BIAS_IDE (0U)
#define CONTROLLER_TORQUE_BIAS_DLC (8U)
#define CONTROLLER_TORQUE_BIAS_CANID (0x7acU)
// signal: @controller_bias_torq_avg_front_ro
#define HYTECH_controller_bias_torq_avg_front_ro_CovFactor (0.001)
#define HYTECH_controller_bias_torq_avg_front_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_bias_torq_avg_front_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_bias_torq_avg_rear_ro
#define HYTECH_controller_bias_torq_avg_rear_ro_CovFactor (0.001)
#define HYTECH_controller_bias_torq_avg_rear_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_bias_torq_avg_rear_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_bias_torq_avg_front_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_bias_torq_avg_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_bias_torq_avg_rear_ro;        //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_bias_torq_avg_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_bias_torq_avg_front_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_bias_torq_avg_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_bias_torq_avg_rear_ro;        //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_bias_torq_avg_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TORQUE_BIAS_t;

// def @CONTROLLER_REGEN_5KPH_TORQUE CAN Message (1965 0x7ad)
#define CONTROLLER_REGEN_5KPH_TORQUE_IDE (0U)
#define CONTROLLER_REGEN_5KPH_TORQUE_DLC (8U)
#define CONTROLLER_REGEN_5KPH_TORQUE_CANID (0x7adU)
// signal: @controller_regen_5kph_torq_fl_ro
#define HYTECH_controller_regen_5kph_torq_fl_ro_CovFactor (0.001)
#define HYTECH_controller_regen_5kph_torq_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_regen_5kph_torq_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_regen_5kph_torq_fr_ro
#define HYTECH_controller_regen_5kph_torq_fr_ro_CovFactor (0.001)
#define HYTECH_controller_regen_5kph_torq_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_regen_5kph_torq_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_regen_5kph_torq_rl_ro
#define HYTECH_controller_regen_5kph_torq_rl_ro_CovFactor (0.001)
#define HYTECH_controller_regen_5kph_torq_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_regen_5kph_torq_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_regen_5kph_torq_rr_ro
#define HYTECH_controller_regen_5kph_torq_rr_ro_CovFactor (0.001)
#define HYTECH_controller_regen_5kph_torq_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_regen_5kph_torq_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_regen_5kph_torq_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_regen_5kph_torq_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_5kph_torq_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_5kph_torq_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_REGEN_5KPH_TORQUE_t;

// def @CONTROLLER_REGEN_5KPH_STATUS CAN Message (1966 0x7ae)
#define CONTROLLER_REGEN_5KPH_STATUS_IDE (0U)
#define CONTROLLER_REGEN_5KPH_STATUS_DLC (8U)
#define CONTROLLER_REGEN_5KPH_STATUS_CANID (0x7aeU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_regen_5kph_status_fl : 1;   //      Bits= 1

  uint8_t controller_regen_5kph_status_fr : 1;   //      Bits= 1

  uint8_t controller_regen_5kph_status_rl : 1;   //      Bits= 1

  uint8_t controller_regen_5kph_status_rr : 1;   //      Bits= 1

#else

  uint8_t controller_regen_5kph_status_fl;       //      Bits= 1

  uint8_t controller_regen_5kph_status_fr;       //      Bits= 1

  uint8_t controller_regen_5kph_status_rl;       //      Bits= 1

  uint8_t controller_regen_5kph_status_rr;       //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_REGEN_5KPH_STATUS_t;

// def @CONTROLLER_TCS_PID_CONFIG CAN Message (1967 0x7af)
#define CONTROLLER_TCS_PID_CONFIG_IDE (0U)
#define CONTROLLER_TCS_PID_CONFIG_DLC (8U)
#define CONTROLLER_TCS_PID_CONFIG_CANID (0x7afU)
// signal: @controller_tcs_pid_p_fl_ro
#define HYTECH_controller_tcs_pid_p_fl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_p_fl_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_p_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_p_fr_ro
#define HYTECH_controller_tcs_pid_p_fr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_p_fr_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_p_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_p_rl_ro
#define HYTECH_controller_tcs_pid_p_rl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_p_rl_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_p_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_pid_p_rr_ro
#define HYTECH_controller_tcs_pid_p_rr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_pid_p_rr_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_pid_p_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_tcs_pid_p_fl_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_fr_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_rl_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_rr_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t controller_tcs_pid_p_fl_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_fr_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_rl_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_pid_p_rr_ro;       //      Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_p_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_PID_CONFIG_t;

// def @CONTROLLER_YAW_PID_CONFIG CAN Message (1978 0x7ba)
#define CONTROLLER_YAW_PID_CONFIG_IDE (0U)
#define CONTROLLER_YAW_PID_CONFIG_DLC (8U)
#define CONTROLLER_YAW_PID_CONFIG_CANID (0x7baU)
// signal: @controller_yaw_pid_p_ro
#define HYTECH_controller_yaw_pid_p_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_p_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_p_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_pid_i_ro
#define HYTECH_controller_yaw_pid_i_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_i_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_i_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_pid_d_ro
#define HYTECH_controller_yaw_pid_d_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_d_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_d_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_pid_brakes_p_ro
#define HYTECH_controller_yaw_pid_brakes_p_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_brakes_p_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_brakes_p_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_pid_brakes_i_ro
#define HYTECH_controller_yaw_pid_brakes_i_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_brakes_i_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_brakes_i_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_pid_brakes_d_ro
#define HYTECH_controller_yaw_pid_brakes_d_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_brakes_d_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_brakes_d_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_yaw_pid_p_ro;             //      Bits=13 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_p_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_i_ro;             //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_i_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_d_ro;             //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_d_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_p_ro;      //      Bits=11 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_p_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_i_ro;      //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_i_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_d_ro;      //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_d_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t controller_yaw_pid_p_ro;             //      Bits=13 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_p_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_i_ro;             //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_i_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_d_ro;             //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_d_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_p_ro;      //      Bits=11 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_p_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_i_ro;      //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_i_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_yaw_pid_brakes_d_ro;      //      Bits=10 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_brakes_d_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_YAW_PID_CONFIG_t;

// def @CONTROLLER_TCS_SATURATION_CONFIG CAN Message (1979 0x7bb)
#define CONTROLLER_TCS_SATURATION_CONFIG_IDE (0U)
#define CONTROLLER_TCS_SATURATION_CONFIG_DLC (8U)
#define CONTROLLER_TCS_SATURATION_CONFIG_CANID (0x7bbU)
// signal: @controller_tcs_saturation_front_ro
#define HYTECH_controller_tcs_saturation_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_saturation_front_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_saturation_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_saturation_rear_ro
#define HYTECH_controller_tcs_saturation_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_saturation_rear_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_saturation_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_tcs_saturation_front_ro;      //      Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_saturation_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_saturation_rear_ro;       //      Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_saturation_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t controller_tcs_saturation_front_ro;      //      Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_saturation_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_saturation_rear_ro;       //      Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_saturation_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_SATURATION_CONFIG_t;

// def @CONTROLLER_TCS_CONFIG_CONT CAN Message (1980 0x7bc)
#define CONTROLLER_TCS_CONFIG_CONT_IDE (0U)
#define CONTROLLER_TCS_CONFIG_CONT_DLC (8U)
#define CONTROLLER_TCS_CONFIG_CONT_CANID (0x7bcU)
// signal: @controller_tcs_lauSL_start_front_ro
#define HYTECH_controller_tcs_lauSL_start_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_lauSL_start_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_lauSL_start_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_lauSL_end_front_ro
#define HYTECH_controller_tcs_lauSL_end_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_lauSL_end_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_lauSL_end_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_lauSL_start_rear_ro
#define HYTECH_controller_tcs_lauSL_start_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_lauSL_start_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_lauSL_start_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_lauSL_end_rear_ro
#define HYTECH_controller_tcs_lauSL_end_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_lauSL_end_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_lauSL_end_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_SL_start_front_ro
#define HYTECH_controller_tcs_SL_start_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_SL_start_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_SL_start_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_SL_end_front_ro
#define HYTECH_controller_tcs_SL_end_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_SL_end_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_SL_end_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_SL_start_rear_ro
#define HYTECH_controller_tcs_SL_start_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_SL_start_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_SL_start_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_SL_end_rear_ro
#define HYTECH_controller_tcs_SL_end_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_SL_end_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_SL_end_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_tcs_lauSL_start_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_start_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_end_front_ro;         //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_end_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_start_rear_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_start_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_end_rear_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_end_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_start_front_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_start_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_end_front_ro;            //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_end_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_start_rear_ro;           //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_start_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_end_rear_ro;             //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_end_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t controller_tcs_lauSL_start_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_start_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_end_front_ro;         //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_end_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_start_rear_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_start_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_lauSL_end_rear_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_lauSL_end_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_start_front_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_start_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_end_front_ro;            //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_end_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_start_rear_ro;           //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_start_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_SL_end_rear_ro;             //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_SL_end_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_CONFIG_CONT_t;

// def @CONTROLLER_TCS_DIFF_CONFIG CAN Message (1981 0x7bd)
#define CONTROLLER_TCS_DIFF_CONFIG_IDE (0U)
#define CONTROLLER_TCS_DIFF_CONFIG_DLC (8U)
#define CONTROLLER_TCS_DIFF_CONFIG_CANID (0x7bdU)
// signal: @controller_tcs_gen_LRdiff_lowerB_ro
#define HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_gen_LRdiff_upperB_ro
#define HYTECH_controller_tcs_gen_LRdiff_upperB_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_gen_LRdiff_upperB_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_gen_LRdiff_upperB_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_w_steer_lower_b_ro
#define HYTECH_controller_tcs_w_steer_lower_b_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_w_steer_lower_b_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_w_steer_lower_b_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_w_steer_upper_b_ro
#define HYTECH_controller_tcs_w_steer_upper_b_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_w_steer_upper_b_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_w_steer_upper_b_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_launch_LRdiff_ro
#define HYTECH_controller_tcs_launch_LRdiff_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_launch_LRdiff_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_launch_LRdiff_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_tcs_gen_LRdiff_lowerB_ro;      //      Bits=13 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_gen_LRdiff_lowerB_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_gen_LRdiff_upperB_ro;      //      Bits=14 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_gen_LRdiff_upperB_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_w_steer_lower_b_ro;        //      Bits=12 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_w_steer_lower_b_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_w_steer_upper_b_ro;        //      Bits=12 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_w_steer_upper_b_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_launch_LRdiff_ro;          //      Bits=13 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_LRdiff_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t controller_tcs_gen_LRdiff_lowerB_ro;      //      Bits=13 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_gen_LRdiff_lowerB_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_gen_LRdiff_upperB_ro;      //      Bits=14 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_gen_LRdiff_upperB_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_w_steer_lower_b_ro;        //      Bits=12 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_w_steer_lower_b_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_w_steer_upper_b_ro;        //      Bits=12 Factor= 0.01            Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_w_steer_upper_b_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_launch_LRdiff_ro;          //      Bits=13 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_LRdiff_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_DIFF_CONFIG_t;

// def @CONTROLLER_TCS_RPM_SCHE_CONFIG CAN Message (1982 0x7be)
#define CONTROLLER_TCS_RPM_SCHE_CONFIG_IDE (0U)
#define CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC (8U)
#define CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID (0x7beU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_tcs_lower_rpm_front;      //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_lower_rpm_rear;       //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_upper_rpm_front;      //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_upper_rpm_rear;       //      Bits=15 Unit:'RPM'

#else

  uint16_t controller_tcs_lower_rpm_front;      //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_lower_rpm_rear;       //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_upper_rpm_front;      //      Bits=15 Unit:'RPM'

  uint16_t controller_tcs_upper_rpm_rear;       //      Bits=15 Unit:'RPM'

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_RPM_SCHE_CONFIG_t;

// def @CONTROLLER_TCS_SLIP_TARGETS CAN Message (1983 0x7bf)
#define CONTROLLER_TCS_SLIP_TARGETS_IDE (0U)
#define CONTROLLER_TCS_SLIP_TARGETS_DLC (8U)
#define CONTROLLER_TCS_SLIP_TARGETS_CANID (0x7bfU)
// signal: @controller_tcs_slip_target_front_ro
#define HYTECH_controller_tcs_slip_target_front_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_slip_target_front_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_slip_target_front_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_slip_target_rear_ro
#define HYTECH_controller_tcs_slip_target_rear_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_slip_target_rear_ro_toS(x) ( (uint16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_slip_target_rear_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint16_t controller_tcs_slip_target_front_ro;      //      Bits=12 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_slip_target_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_slip_target_rear_ro;       //      Bits=12 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_slip_target_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint16_t controller_tcs_slip_target_front_ro;      //      Bits=12 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_slip_target_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint16_t controller_tcs_slip_target_rear_ro;       //      Bits=12 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_slip_target_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_SLIP_TARGETS_t;

// def @VEHM_WHEEL_STEER_AVG_DEG_MSG CAN Message (1994 0x7ca)
#define VEHM_WHEEL_STEER_AVG_DEG_MSG_IDE (0U)
#define VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC (8U)
#define VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID (0x7caU)
// signal: @vehm_wheel_steer_avg_deg_ro
#define HYTECH_vehm_wheel_steer_avg_deg_ro_CovFactor (0.001)
#define HYTECH_vehm_wheel_steer_avg_deg_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_wheel_steer_avg_deg_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_wheel_steer_avg_deg_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_steer_avg_deg_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_wheel_steer_avg_deg_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_wheel_steer_avg_deg_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_WHEEL_STEER_AVG_DEG_MSG_t;

// def @VEHM_KIN_DES_YAW_RATE_RAD_S_MSG CAN Message (1995 0x7cb)
#define VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_IDE (0U)
#define VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC (8U)
#define VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID (0x7cbU)
// signal: @vehm_kin_desired_yaw_rate_rad_s_ro
#define HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_CovFactor (0.0001)
#define HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_kin_desired_yaw_rate_rad_s_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_kin_desired_yaw_rate_rad_s_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_kin_desired_yaw_rate_rad_s_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_kin_desired_yaw_rate_rad_s_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t;

// def @CONTROLLER_TCS_NL_SCHE_CONFIG CAN Message (1996 0x7cc)
#define CONTROLLER_TCS_NL_SCHE_CONFIG_IDE (0U)
#define CONTROLLER_TCS_NL_SCHE_CONFIG_DLC (8U)
#define CONTROLLER_TCS_NL_SCHE_CONFIG_CANID (0x7ccU)
// signal: @controller_tcs_endper_nl_front_ro
#define HYTECH_controller_tcs_endper_nl_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_endper_nl_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_endper_nl_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_endper_nl_rear_ro
#define HYTECH_controller_tcs_endper_nl_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_endper_nl_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_endper_nl_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_startper_nl_front_ro
#define HYTECH_controller_tcs_startper_nl_front_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_startper_nl_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_startper_nl_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_startper_nl_rear_ro
#define HYTECH_controller_tcs_startper_nl_rear_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_startper_nl_rear_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_startper_nl_rear_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_tcs_endper_nl_front_ro;         //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_endper_nl_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_endper_nl_rear_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_endper_nl_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_startper_nl_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_startper_nl_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_startper_nl_rear_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_startper_nl_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t controller_tcs_endper_nl_front_ro;         //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_endper_nl_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_endper_nl_rear_ro;          //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_endper_nl_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_startper_nl_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_startper_nl_front_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_startper_nl_rear_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_startper_nl_rear_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_NL_SCHE_CONFIG_t;

// def @CONTROLLER_PID_YAW CAN Message (1997 0x7cd)
#define CONTROLLER_PID_YAW_IDE (0U)
#define CONTROLLER_PID_YAW_DLC (8U)
#define CONTROLLER_PID_YAW_CANID (0x7cdU)
// signal: @controller_yaw_rate_error_ro
#define HYTECH_controller_yaw_rate_error_ro_CovFactor (0.0001)
#define HYTECH_controller_yaw_rate_error_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_yaw_rate_error_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_yaw_pid_output_ro
#define HYTECH_controller_yaw_pid_output_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_pid_output_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_pid_output_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_kin_desired_yaw_rate_rad_s_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_kin_desired_yaw_rate_rad_s_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_rate_error_ro;             //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_rate_error_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_pid_output_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_output_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_kin_desired_yaw_rate_rad_s_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_kin_desired_yaw_rate_rad_s_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_rate_error_ro;             //  [-] Bits=16 Factor= 0.0001          Unit:'rad/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_rate_error_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_pid_output_ro;             //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_pid_output_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_PID_YAW_t;

// def @VEHM_LONG_CORNER_VEL CAN Message (1998 0x7ce)
#define VEHM_LONG_CORNER_VEL_IDE (0U)
#define VEHM_LONG_CORNER_VEL_DLC (8U)
#define VEHM_LONG_CORNER_VEL_CANID (0x7ceU)
// signal: @vehm_long_corner_vel_fl_ro
#define HYTECH_vehm_long_corner_vel_fl_ro_CovFactor (0.01)
#define HYTECH_vehm_long_corner_vel_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_long_corner_vel_fl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_long_corner_vel_fr_ro
#define HYTECH_vehm_long_corner_vel_fr_ro_CovFactor (0.01)
#define HYTECH_vehm_long_corner_vel_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_long_corner_vel_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_long_corner_vel_rl_ro
#define HYTECH_vehm_long_corner_vel_rl_ro_CovFactor (0.01)
#define HYTECH_vehm_long_corner_vel_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_long_corner_vel_rl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @vehm_long_corner_vel_rr_ro
#define HYTECH_vehm_long_corner_vel_rr_ro_CovFactor (0.01)
#define HYTECH_vehm_long_corner_vel_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_vehm_long_corner_vel_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_long_corner_vel_fl_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_fr_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_rl_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_rr_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_long_corner_vel_fl_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_fr_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_rl_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_long_corner_vel_rr_ro;        //  [-] Bits=16 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_long_corner_vel_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_LONG_CORNER_VEL_t;

// def @VEHM_SL CAN Message (1999 0x7cf)
#define VEHM_SL_IDE (0U)
#define VEHM_SL_DLC (8U)
#define VEHM_SL_CANID (0x7cfU)
// signal: @vehm_sl_fl_ro
#define HYTECH_vehm_sl_fl_ro_CovFactor (0.0001)
#define HYTECH_vehm_sl_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_vehm_sl_fl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @vehm_sl_fr_ro
#define HYTECH_vehm_sl_fr_ro_CovFactor (0.0001)
#define HYTECH_vehm_sl_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_vehm_sl_fr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @vehm_sl_rl_ro
#define HYTECH_vehm_sl_rl_ro_CovFactor (0.0001)
#define HYTECH_vehm_sl_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_vehm_sl_rl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @vehm_sl_rr_ro
#define HYTECH_vehm_sl_rr_ro_CovFactor (0.0001)
#define HYTECH_vehm_sl_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_vehm_sl_rr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_sl_fl_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_fr_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_rl_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_rr_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_sl_fl_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_fr_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_rl_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_sl_rr_ro;                     //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_sl_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_SL_t;

// def @CONTROLLER_PID_YAW_TORQUE CAN Message (2010 0x7da)
#define CONTROLLER_PID_YAW_TORQUE_IDE (0U)
#define CONTROLLER_PID_YAW_TORQUE_DLC (8U)
#define CONTROLLER_PID_YAW_TORQUE_CANID (0x7daU)
// signal: @controller_yaw_torque_fl_ro
#define HYTECH_controller_yaw_torque_fl_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_torque_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_torque_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_torque_fr_ro
#define HYTECH_controller_yaw_torque_fr_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_torque_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_torque_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_torque_rl_ro
#define HYTECH_controller_yaw_torque_rl_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_torque_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_torque_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_yaw_torque_rr_ro
#define HYTECH_controller_yaw_torque_rr_ro_CovFactor (0.001)
#define HYTECH_controller_yaw_torque_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_yaw_torque_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_yaw_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_yaw_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_yaw_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_yaw_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_PID_YAW_TORQUE_t;

// def @CONTROLLER_NORMAL_DIST CAN Message (2011 0x7db)
#define CONTROLLER_NORMAL_DIST_IDE (0U)
#define CONTROLLER_NORMAL_DIST_DLC (8U)
#define CONTROLLER_NORMAL_DIST_CANID (0x7dbU)
// signal: @controller_normal_percent_fl_ro
#define HYTECH_controller_normal_percent_fl_ro_CovFactor (0.0001)
#define HYTECH_controller_normal_percent_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_normal_percent_fl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_normal_percent_fr_ro
#define HYTECH_controller_normal_percent_fr_ro_CovFactor (0.0001)
#define HYTECH_controller_normal_percent_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_normal_percent_fr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_normal_percent_rl_ro
#define HYTECH_controller_normal_percent_rl_ro_CovFactor (0.0001)
#define HYTECH_controller_normal_percent_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_normal_percent_rl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_normal_percent_rr_ro
#define HYTECH_controller_normal_percent_rr_ro_CovFactor (0.0001)
#define HYTECH_controller_normal_percent_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_normal_percent_rr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_normal_percent_fl_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_fr_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_rl_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_rr_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_normal_percent_fl_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_fr_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_rl_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_percent_rr_ro;       //  [-] Bits=16 Factor= 0.0001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_percent_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_NORMAL_DIST_t;

// def @CONTROLLER_NORMAL_TORQUE CAN Message (2012 0x7dc)
#define CONTROLLER_NORMAL_TORQUE_IDE (0U)
#define CONTROLLER_NORMAL_TORQUE_DLC (8U)
#define CONTROLLER_NORMAL_TORQUE_CANID (0x7dcU)
// signal: @controller_normal_torque_fl_ro
#define HYTECH_controller_normal_torque_fl_ro_CovFactor (0.001)
#define HYTECH_controller_normal_torque_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_normal_torque_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_normal_torque_fr_ro
#define HYTECH_controller_normal_torque_fr_ro_CovFactor (0.001)
#define HYTECH_controller_normal_torque_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_normal_torque_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_normal_torque_rl_ro
#define HYTECH_controller_normal_torque_rl_ro_CovFactor (0.001)
#define HYTECH_controller_normal_torque_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_normal_torque_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_normal_torque_rr_ro
#define HYTECH_controller_normal_torque_rr_ro_CovFactor (0.001)
#define HYTECH_controller_normal_torque_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_normal_torque_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_normal_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_normal_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_normal_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_normal_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_NORMAL_TORQUE_t;

// def @CONTROLLER_POWER_LIM CAN Message (2014 0x7de)
#define CONTROLLER_POWER_LIM_IDE (0U)
#define CONTROLLER_POWER_LIM_DLC (8U)
#define CONTROLLER_POWER_LIM_CANID (0x7deU)
// signal: @controller_power_lim_error_ro
#define HYTECH_controller_power_lim_error_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_error_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_error_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_power_lim_torque_adj_ro
#define HYTECH_controller_power_lim_torque_adj_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_torque_adj_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_torque_adj_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_power_lim_status : 4;          //      Bits= 4

  int16_t controller_power_lim_error_ro;            //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_error_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_adj_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_adj_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t controller_power_lim_status;              //      Bits= 4

  int16_t controller_power_lim_error_ro;            //  [-] Bits=16 Factor= 0.001

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_error_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_adj_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_adj_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_POWER_LIM_t;

// def @CONTROLLER_POWER_LIM_CORNER_POW CAN Message (2026 0x7ea)
#define CONTROLLER_POWER_LIM_CORNER_POW_IDE (0U)
#define CONTROLLER_POWER_LIM_CORNER_POW_DLC (8U)
#define CONTROLLER_POWER_LIM_CORNER_POW_CANID (0x7eaU)
// signal: @controller_power_lim_cornerp_fl_ro
#define HYTECH_controller_power_lim_cornerp_fl_ro_CovFactor (0.01)
#define HYTECH_controller_power_lim_cornerp_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_power_lim_cornerp_fl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_power_lim_cornerp_fr_ro
#define HYTECH_controller_power_lim_cornerp_fr_ro_CovFactor (0.01)
#define HYTECH_controller_power_lim_cornerp_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_power_lim_cornerp_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_power_lim_cornerp_rl_ro
#define HYTECH_controller_power_lim_cornerp_rl_ro_CovFactor (0.01)
#define HYTECH_controller_power_lim_cornerp_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_power_lim_cornerp_rl_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_power_lim_cornerp_rr_ro
#define HYTECH_controller_power_lim_cornerp_rr_ro_CovFactor (0.01)
#define HYTECH_controller_power_lim_cornerp_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_power_lim_cornerp_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_power_lim_cornerp_fl_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_fr_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_rl_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_rr_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_power_lim_cornerp_fl_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_fr_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_rl_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_cornerp_rr_ro;       //  [-] Bits=16 Factor= 0.01            Unit:'kW'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_cornerp_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_POWER_LIM_CORNER_POW_t;

// def @CONTROLLER_POWER_LIM_TORQUE CAN Message (2027 0x7eb)
#define CONTROLLER_POWER_LIM_TORQUE_IDE (0U)
#define CONTROLLER_POWER_LIM_TORQUE_DLC (8U)
#define CONTROLLER_POWER_LIM_TORQUE_CANID (0x7ebU)
// signal: @controller_power_lim_torque_fl_ro
#define HYTECH_controller_power_lim_torque_fl_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_torque_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_torque_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_power_lim_torque_fr_ro
#define HYTECH_controller_power_lim_torque_fr_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_torque_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_torque_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_power_lim_torque_rl_ro
#define HYTECH_controller_power_lim_torque_rl_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_torque_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_torque_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_power_lim_torque_rr_ro
#define HYTECH_controller_power_lim_torque_rr_ro_CovFactor (0.001)
#define HYTECH_controller_power_lim_torque_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_power_lim_torque_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_power_lim_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_power_lim_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_power_lim_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_power_lim_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_POWER_LIM_TORQUE_t;

// def @CONTROLLER_BOOLEAN CAN Message (2028 0x7ec)
#define CONTROLLER_BOOLEAN_IDE (0U)
#define CONTROLLER_BOOLEAN_DLC (8U)
#define CONTROLLER_BOOLEAN_CANID (0x7ecU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_use_launch : 1;              //      Bits= 1

  uint8_t controller_use_pid_tv : 1;              //      Bits= 1

  uint8_t controller_use_normal_force : 1;        //      Bits= 1

  uint8_t controller_use_pid_power_limit : 1;     //      Bits= 1

  uint8_t controller_use_power_limit : 1;         //      Bits= 1

  uint8_t controller_use_tcs : 1;                 //      Bits= 1

  uint8_t controller_use_tcs_lim_yaw_pid : 1;     //      Bits= 1

  uint8_t controller_use_dec_yaw_pid_brake : 1;   //      Bits= 1

  uint8_t controller_use_discontin_brakes : 1;    //      Bits= 1

  uint8_t controller_use_no_regen_5kph : 1;       //      Bits= 1

  uint8_t controller_use_torque_bias : 1;         //      Bits= 1

  uint8_t controller_use_nl_tcs_gain_sche : 1;    //      Bits= 1

  uint8_t controller_use_rpm_tcs_gain_sche : 1;   //      Bits= 1

  uint8_t controller_use_nl_tcs_slipschedu : 1;   //      Bits= 1

#else

  uint8_t controller_use_launch;                  //      Bits= 1

  uint8_t controller_use_pid_tv;                  //      Bits= 1

  uint8_t controller_use_normal_force;            //      Bits= 1

  uint8_t controller_use_pid_power_limit;         //      Bits= 1

  uint8_t controller_use_power_limit;             //      Bits= 1

  uint8_t controller_use_tcs;                     //      Bits= 1

  uint8_t controller_use_tcs_lim_yaw_pid;         //      Bits= 1

  uint8_t controller_use_dec_yaw_pid_brake;       //      Bits= 1

  uint8_t controller_use_discontin_brakes;        //      Bits= 1

  uint8_t controller_use_no_regen_5kph;           //      Bits= 1

  uint8_t controller_use_torque_bias;             //      Bits= 1

  uint8_t controller_use_nl_tcs_gain_sche;        //      Bits= 1

  uint8_t controller_use_rpm_tcs_gain_sche;       //      Bits= 1

  uint8_t controller_use_nl_tcs_slipschedu;       //      Bits= 1

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_BOOLEAN_t;

// def @VEHM_BETA CAN Message (2031 0x7ef)
#define VEHM_BETA_IDE (0U)
#define VEHM_BETA_DLC (8U)
#define VEHM_BETA_CANID (0x7efU)
// signal: @vehm_beta_deg_ro
#define HYTECH_vehm_beta_deg_ro_CovFactor (0.001)
#define HYTECH_vehm_beta_deg_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_beta_deg_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_beta_deg_ro;                  //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_beta_deg_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_beta_deg_ro;                  //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_beta_deg_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_BETA_t;

// def @DASHBOARD_BUZZER_CONTROL CAN Message (2033 0x7f1)
#define DASHBOARD_BUZZER_CONTROL_IDE (0U)
#define DASHBOARD_BUZZER_CONTROL_DLC (2U)
#define DASHBOARD_BUZZER_CONTROL_CANID (0x7f1U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t dash_buzzer_flag : 1;              //      Bits= 1

  uint8_t in_pedal_calibration_state : 1;    //      Bits= 1

  uint8_t torque_limit_enum_value;           //      Bits= 8

#else

  uint8_t dash_buzzer_flag;                  //      Bits= 1

  uint8_t in_pedal_calibration_state;        //      Bits= 1

  uint8_t torque_limit_enum_value;           //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} DASHBOARD_BUZZER_CONTROL_t;

// def @CONTROLLER_TORQUE_SETUP CAN Message (2042 0x7fa)
#define CONTROLLER_TORQUE_SETUP_IDE (0U)
#define CONTROLLER_TORQUE_SETUP_DLC (8U)
#define CONTROLLER_TORQUE_SETUP_CANID (0x7faU)
// signal: @controller_initial_torque_req_ro
#define HYTECH_controller_initial_torque_req_ro_CovFactor (0.001)
#define HYTECH_controller_initial_torque_req_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_initial_torque_req_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_constrained_torq_req_ro
#define HYTECH_controller_constrained_torq_req_ro_CovFactor (0.001)
#define HYTECH_controller_constrained_torq_req_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_constrained_torq_req_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_regen_limit_ro
#define HYTECH_controller_regen_limit_ro_CovFactor (0.01)
#define HYTECH_controller_regen_limit_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_regen_limit_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_torque_mode_ro
#define HYTECH_controller_torque_mode_ro_CovFactor (0.1)
#define HYTECH_controller_torque_mode_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_controller_torque_mode_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @controller_max_yaw_nl_accel_perc_ro
#define HYTECH_controller_max_yaw_nl_accel_perc_ro_CovFactor (0.1)
#define HYTECH_controller_max_yaw_nl_accel_perc_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_controller_max_yaw_nl_accel_perc_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @controller_max_nl_brake_sc_front_ro
#define HYTECH_controller_max_nl_brake_sc_front_ro_CovFactor (0.01)
#define HYTECH_controller_max_nl_brake_sc_front_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_max_nl_brake_sc_front_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_initial_torque_req_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_initial_torque_req_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_constrained_torq_req_ro;        //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_constrained_torq_req_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_limit_ro;                 //  [-] Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_torque_mode_ro;                 //      Bits= 8 Factor= 0.1             Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_torque_mode_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_max_yaw_nl_accel_perc_ro : 4;   //      Bits= 4 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_max_yaw_nl_accel_perc_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_max_nl_brake_sc_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_max_nl_brake_sc_front_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_initial_torque_req_ro;          //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_initial_torque_req_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_constrained_torq_req_ro;        //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_constrained_torq_req_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_regen_limit_ro;                 //  [-] Bits=12 Factor= 0.01            Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_regen_limit_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_torque_mode_ro;                 //      Bits= 8 Factor= 0.1             Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_torque_mode_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_max_yaw_nl_accel_perc_ro;       //      Bits= 4 Factor= 0.1

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_max_yaw_nl_accel_perc_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_max_nl_brake_sc_front_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_max_nl_brake_sc_front_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TORQUE_SETUP_t;

// def @CONTROLLER_TCS_CONFIG CAN Message (2043 0x7fb)
#define CONTROLLER_TCS_CONFIG_IDE (0U)
#define CONTROLLER_TCS_CONFIG_DLC (8U)
#define CONTROLLER_TCS_CONFIG_CANID (0x7fbU)
// signal: @controller_tcs_launch_dead_zone_ro
#define HYTECH_controller_tcs_launch_dead_zone_ro_CovFactor (0.1)
#define HYTECH_controller_tcs_launch_dead_zone_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.1)) )
#define HYTECH_controller_tcs_launch_dead_zone_ro_fromS(x) ( (((x) * (0.1)) + (0.0)) )
// signal: @controller_tcs_launch_vel_thresh_ro
#define HYTECH_controller_tcs_launch_vel_thresh_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_launch_vel_thresh_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_launch_vel_thresh_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_vel_thresh_ro
#define HYTECH_controller_tcs_vel_thresh_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_vel_thresh_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_vel_thresh_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_sl_nlperc_star_fr_ro
#define HYTECH_controller_tcs_sl_nlperc_star_fr_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_sl_nlperc_star_fr_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_sl_nlperc_star_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_sl_nlperc_end_fr_ro
#define HYTECH_controller_tcs_sl_nlperc_end_fr_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_sl_nlperc_end_fr_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_sl_nlperc_end_fr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_sl_nlperc_end_rr_ro
#define HYTECH_controller_tcs_sl_nlperc_end_rr_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_sl_nlperc_end_rr_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_sl_nlperc_end_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )
// signal: @controller_tcs_sl_nlperc_star_rr_ro
#define HYTECH_controller_tcs_sl_nlperc_star_rr_ro_CovFactor (0.01)
#define HYTECH_controller_tcs_sl_nlperc_star_rr_ro_toS(x) ( (uint8_t) (((x) - (0.0)) / (0.01)) )
#define HYTECH_controller_tcs_sl_nlperc_star_rr_ro_fromS(x) ( (((x) * (0.01)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_tcs_launch_dead_zone_ro;        //      Bits= 8 Factor= 0.1             Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_dead_zone_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_launch_vel_thresh_ro : 6;   //      Bits= 6 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_vel_thresh_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_vel_thresh_ro;              //      Bits= 8 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_vel_thresh_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_star_fr_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_star_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_end_fr_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_end_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_end_rr_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_end_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_star_rr_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_star_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  uint8_t controller_tcs_launch_dead_zone_ro;        //      Bits= 8 Factor= 0.1             Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_dead_zone_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_launch_vel_thresh_ro;       //      Bits= 6 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_launch_vel_thresh_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_vel_thresh_ro;              //      Bits= 8 Factor= 0.01            Unit:'m/s'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_vel_thresh_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_star_fr_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_star_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_end_fr_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_end_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_end_rr_ro;        //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_end_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

  uint8_t controller_tcs_sl_nlperc_star_rr_ro;       //      Bits= 8 Factor= 0.01

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_sl_nlperc_star_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_CONFIG_t;

// def @CONTROLLER_TCS_TORQUE CAN Message (2044 0x7fc)
#define CONTROLLER_TCS_TORQUE_IDE (0U)
#define CONTROLLER_TCS_TORQUE_DLC (8U)
#define CONTROLLER_TCS_TORQUE_CANID (0x7fcU)
// signal: @controller_tcs_torque_fl_ro
#define HYTECH_controller_tcs_torque_fl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_torque_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_torque_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_torque_fr_ro
#define HYTECH_controller_tcs_torque_fr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_torque_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_torque_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_torque_rl_ro
#define HYTECH_controller_tcs_torque_rl_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_torque_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_torque_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @controller_tcs_torque_rr_ro
#define HYTECH_controller_tcs_torque_rr_ro_CovFactor (0.001)
#define HYTECH_controller_tcs_torque_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_controller_tcs_torque_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_tcs_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_tcs_torque_fl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_fr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_rl_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_torque_rr_ro;       //  [-] Bits=16 Factor= 0.001           Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_torque_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_TORQUE_t;

// def @CONTROLLER_TCS_STATUS CAN Message (2045 0x7fd)
#define CONTROLLER_TCS_STATUS_IDE (0U)
#define CONTROLLER_TCS_STATUS_DLC (8U)
#define CONTROLLER_TCS_STATUS_CANID (0x7fdU)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t controller_tcs_status_fl : 2;      //      Bits= 2

  uint8_t controller_tcs_status_fr : 2;      //      Bits= 2

  uint8_t controller_tcs_status_rl : 2;      //      Bits= 2

  uint8_t controller_tcs_status_rr : 2;      //      Bits= 2

#else

  uint8_t controller_tcs_status_fl;          //      Bits= 2

  uint8_t controller_tcs_status_fr;          //      Bits= 2

  uint8_t controller_tcs_status_rl;          //      Bits= 2

  uint8_t controller_tcs_status_rr;          //      Bits= 2

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_STATUS_t;

// def @CONTROLLER_TCS_PID_INPUT CAN Message (2046 0x7fe)
#define CONTROLLER_TCS_PID_INPUT_IDE (0U)
#define CONTROLLER_TCS_PID_INPUT_DLC (8U)
#define CONTROLLER_TCS_PID_INPUT_CANID (0x7feU)
// signal: @controller_tcs_pid_input_fl_ro
#define HYTECH_controller_tcs_pid_input_fl_ro_CovFactor (0.0001)
#define HYTECH_controller_tcs_pid_input_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_tcs_pid_input_fl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_tcs_pid_input_fr_ro
#define HYTECH_controller_tcs_pid_input_fr_ro_CovFactor (0.0001)
#define HYTECH_controller_tcs_pid_input_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_tcs_pid_input_fr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_tcs_pid_input_rl_ro
#define HYTECH_controller_tcs_pid_input_rl_ro_CovFactor (0.0001)
#define HYTECH_controller_tcs_pid_input_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_tcs_pid_input_rl_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )
// signal: @controller_tcs_pid_input_rr_ro
#define HYTECH_controller_tcs_pid_input_rr_ro_CovFactor (0.0001)
#define HYTECH_controller_tcs_pid_input_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.0001)) )
#define HYTECH_controller_tcs_pid_input_rr_ro_fromS(x) ( (((x) * (0.0001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t controller_tcs_pid_input_fl_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_fr_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_rl_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_rr_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t controller_tcs_pid_input_fl_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_fr_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_rl_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t controller_tcs_pid_input_rr_ro;       //  [-] Bits=16 Factor= 0.0001          Unit:'Nm'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t controller_tcs_pid_input_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CONTROLLER_TCS_PID_INPUT_t;

// def @VEHM_ALPHA CAN Message (2047 0x7ff)
#define VEHM_ALPHA_IDE (0U)
#define VEHM_ALPHA_DLC (8U)
#define VEHM_ALPHA_CANID (0x7ffU)
// signal: @vehm_alpha_deg_fl_ro
#define HYTECH_vehm_alpha_deg_fl_ro_CovFactor (0.001)
#define HYTECH_vehm_alpha_deg_fl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_alpha_deg_fl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vehm_alpha_deg_fr_ro
#define HYTECH_vehm_alpha_deg_fr_ro_CovFactor (0.001)
#define HYTECH_vehm_alpha_deg_fr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_alpha_deg_fr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vehm_alpha_deg_rl_ro
#define HYTECH_vehm_alpha_deg_rl_ro_CovFactor (0.001)
#define HYTECH_vehm_alpha_deg_rl_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_alpha_deg_rl_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )
// signal: @vehm_alpha_deg_rr_ro
#define HYTECH_vehm_alpha_deg_rr_ro_CovFactor (0.001)
#define HYTECH_vehm_alpha_deg_rr_ro_toS(x) ( (int16_t) (((x) - (0.0)) / (0.001)) )
#define HYTECH_vehm_alpha_deg_rr_ro_fromS(x) ( (((x) * (0.001)) + (0.0)) )

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  int16_t vehm_alpha_deg_fl_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_fr_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_rl_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_rr_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#else

  int16_t vehm_alpha_deg_fl_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_fl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_fr_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_fr_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_rl_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_rl_phys;
#endif // HYTECH_USE_SIGFLOAT

  int16_t vehm_alpha_deg_rr_ro;              //  [-] Bits=16 Factor= 0.001           Unit:'deg'

#ifdef HYTECH_USE_SIGFLOAT
  sigfloat_t vehm_alpha_deg_rr_phys;
#endif // HYTECH_USE_SIGFLOAT

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} VEHM_ALPHA_t;

// def @CHARGER_CONTROL CAN Message (403105268 0x1806e5f4)
#define CHARGER_CONTROL_IDE (1U)
#define CHARGER_CONTROL_DLC (5U)
#define CHARGER_CONTROL_CANID (0x1806e5f4U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_voltage_high;         //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_voltage_low;          //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_current_high;         //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_current_low;          //      Bits= 8

  uint8_t control;                           //      Bits= 8

#else

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_voltage_high;         //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_voltage_low;          //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_current_high;         //      Bits= 8

  // Unused for any PCAN stuff only for Elcon Charger
  uint8_t max_charging_current_low;          //      Bits= 8

  uint8_t control;                           //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CHARGER_CONTROL_t;

// def @CHARGER_DATA CAN Message (419385573 0x18ff50e5)
#define CHARGER_DATA_IDE (1U)
#define CHARGER_DATA_DLC (7U)
#define CHARGER_DATA_CANID (0x18ff50e5U)

typedef struct
{
#ifdef HYTECH_USE_BITS_SIGNAL

  uint8_t output_dc_voltage_high;            //      Bits= 8

  uint8_t output_dc_voltage_low;             //      Bits= 8

  uint8_t output_current_high;               //      Bits= 8

  uint8_t output_current_low;                //      Bits= 8

  uint8_t flags;                             //      Bits= 8

  uint8_t input_ac_voltage_high;             //      Bits= 8

  uint8_t input_ac_voltage_low;              //      Bits= 8

#else

  uint8_t output_dc_voltage_high;            //      Bits= 8

  uint8_t output_dc_voltage_low;             //      Bits= 8

  uint8_t output_current_high;               //      Bits= 8

  uint8_t output_current_low;                //      Bits= 8

  uint8_t flags;                             //      Bits= 8

  uint8_t input_ac_voltage_high;             //      Bits= 8

  uint8_t input_ac_voltage_low;              //      Bits= 8

#endif // HYTECH_USE_BITS_SIGNAL

#ifdef HYTECH_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // HYTECH_USE_DIAG_MONITORS

} CHARGER_DATA_t;

// Function signatures

uint32_t Unpack_IMU_hytech(IMU_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_IMU_hytech(IMU_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_IMU_hytech(IMU_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_State_hytech(State_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_State_hytech(State_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_State_hytech(State_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Settings_hytech(Settings_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_Settings_hytech(Settings_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_Settings_hytech(Settings_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Velocities_hytech(Velocities_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_Velocities_hytech(Velocities_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_Velocities_hytech(Velocities_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Attitude_hytech(Attitude_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_Attitude_hytech(Attitude_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_Attitude_hytech(Attitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Configuration_hytech(Configuration_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_Configuration_hytech(Configuration_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_Configuration_hytech(Configuration_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_STATUS_hytech(INV1_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_STATUS_hytech(INV1_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_STATUS_hytech(INV1_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_POWER_hytech(INV1_POWER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_POWER_hytech(INV1_POWER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_POWER_hytech(INV1_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_STATUS_hytech(INV2_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_STATUS_hytech(INV2_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_STATUS_hytech(INV2_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_POWER_hytech(INV2_POWER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_POWER_hytech(INV2_POWER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_POWER_hytech(INV2_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_STATUS_hytech(INV3_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_STATUS_hytech(INV3_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_STATUS_hytech(INV3_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_POWER_hytech(INV3_POWER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_POWER_hytech(INV3_POWER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_POWER_hytech(INV3_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_STATUS_hytech(INV4_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_STATUS_hytech(INV4_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_STATUS_hytech(INV4_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_POWER_hytech(INV4_POWER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_POWER_hytech(INV4_POWER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_POWER_hytech(INV4_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_STATUS_hytech(MCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_STATUS_hytech(MCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_STATUS_hytech(MCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_VEL_hytech(VN_VEL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_VEL_hytech(VN_VEL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_VEL_hytech(VN_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_YPR_hytech(VN_YPR_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_YPR_hytech(VN_YPR_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_YPR_hytech(VN_YPR_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_STATUS_hytech(BMS_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_STATUS_hytech(BMS_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_STATUS_hytech(BMS_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CCU_STATUS_hytech(CCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CCU_STATUS_hytech(CCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CCU_STATUS_hytech(CCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_STATUS_hytech(VN_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_STATUS_hytech(VN_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_STATUS_hytech(VN_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_STATUS_hytech(TCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TCU_STATUS_hytech(TCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TCU_STATUS_hytech(TCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CAR_STATES_hytech(CAR_STATES_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CAR_STATES_hytech(CAR_STATES_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CAR_STATES_hytech(CAR_STATES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASH_INPUT_hytech(DASH_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DASH_INPUT_hytech(DASH_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_INPUT_hytech(DASH_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_EM_STATUS_hytech(EM_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_EM_STATUS_hytech(EM_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_EM_STATUS_hytech(EM_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_STEERING_DATA_hytech(STEERING_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_STEERING_DATA_hytech(STEERING_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_DATA_hytech(STEERING_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ACU_OK_hytech(ACU_OK_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_ACU_OK_hytech(ACU_OK_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACU_OK_hytech(ACU_OK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_SL_hytech(VEHM_SL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_SL_hytech(VEHM_SL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_SL_hytech(VEHM_SL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_BETA_hytech(VEHM_BETA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_BETA_hytech(VEHM_BETA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_BETA_hytech(VEHM_BETA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef HYTECH_USE_CANSTRUCT
uint32_t Pack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // HYTECH_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
