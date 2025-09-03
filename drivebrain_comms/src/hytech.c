#include "hytech.h"


// DBC file version
#if (VER_HYTECH_MAJ != (6U)) || (VER_HYTECH_MIN != (0U))
#error The HYTECH dbc source files have different versions
#endif

#ifdef HYTECH_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "hytech-fmon.h"

#endif // HYTECH_USE_DIAG_MONITORS

// This macro guard for the case when you need to enable
// using diag monitors but there is no necessity in proper
// SysTick provider. For providing one you need define macro
// before this line - in dbccodeconf.h

#ifndef GetSystemTick
#define GetSystemTick() (0u)
#endif

// This macro guard is for the case when you want to build
// app with enabled optoin auto CSM, but don't yet have
// proper getframehash implementation

#ifndef GetFrameHash
#define GetFrameHash(a,b,c,d,e) (0u)
#endif

// This function performs extension of sign for the signals
// which have non-aligned to power of 2 bit's width.
// The types 'bitext_t' and 'ubitext_t' define maximal bit width which
// can be correctly handled. You need to select type which can contain
// n+1 bits where n is the largest signed signal width. For example if
// the most wide signed signal has a width of 31 bits you need to set
// bitext_t as int32_t and ubitext_t as uint32_t
// Defined these typedefs in @dbccodeconf.h or locally in 'dbcdrvname'-config.h
static bitext_t __ext_sig__(ubitext_t val, uint8_t bits)
{
  ubitext_t const m = 1u << (bits - 1);
  return (val ^ m) - m;
}

uint32_t Unpack_IMU_hytech(IMU_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Accel_X_ro = (int16_t) __ext_sig__(( ((_d[1] & (0x03U)) << 8U) | (_d[0] & (0xFFU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Accel_X_phys = (sigfloat_t)(HYTECH_Accel_X_ro_fromS(_m->Accel_X_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Accel_Y_ro = (int16_t) __ext_sig__(( ((_d[2] & (0x0FU)) << 6U) | ((_d[1] >> 2U) & (0x3FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Accel_Y_phys = (sigfloat_t)(HYTECH_Accel_Y_ro_fromS(_m->Accel_Y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Accel_Z_ro = (int16_t) __ext_sig__(( ((_d[3] & (0x3FU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Accel_Z_phys = (sigfloat_t)(HYTECH_Accel_Z_ro_fromS(_m->Accel_Z_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Roll_Rate_ro = (int16_t) __ext_sig__(( ((_d[5] & (0x01U)) << 10U) | ((_d[4] & (0xFFU)) << 2U) | ((_d[3] >> 6U) & (0x03U)) ), 11);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Roll_Rate_phys = (sigfloat_t)(HYTECH_Roll_Rate_ro_fromS(_m->Roll_Rate_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Yaw_Rate_ro = (int16_t) __ext_sig__(( ((_d[6] & (0x0FU)) << 7U) | ((_d[5] >> 1U) & (0x7FU)) ), 11);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Yaw_Rate_phys = (sigfloat_t)(HYTECH_Yaw_Rate_ro_fromS(_m->Yaw_Rate_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Pitch_Rate_ro = (int16_t) __ext_sig__(( ((_d[7] & (0x7FU)) << 4U) | ((_d[6] >> 4U) & (0x0FU)) ), 11);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Pitch_Rate_phys = (sigfloat_t)(HYTECH_Pitch_Rate_ro_fromS(_m->Pitch_Rate_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < IMU_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_IMU_hytech(&_m->mon1, IMU_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return IMU_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_IMU_hytech(IMU_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(IMU_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Accel_X_ro = (int16_t) HYTECH_Accel_X_ro_toS(_m->Accel_X_phys);
  _m->Accel_Y_ro = (int16_t) HYTECH_Accel_Y_ro_toS(_m->Accel_Y_phys);
  _m->Accel_Z_ro = (int16_t) HYTECH_Accel_Z_ro_toS(_m->Accel_Z_phys);
  _m->Roll_Rate_ro = (int16_t) HYTECH_Roll_Rate_ro_toS(_m->Roll_Rate_phys);
  _m->Yaw_Rate_ro = (int16_t) HYTECH_Yaw_Rate_ro_toS(_m->Yaw_Rate_phys);
  _m->Pitch_Rate_ro = (int16_t) HYTECH_Pitch_Rate_ro_toS(_m->Pitch_Rate_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Accel_X_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->Accel_X_ro >> 8U) & (0x03U)) | ((_m->Accel_Y_ro & (0x3FU)) << 2U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->Accel_Y_ro >> 6U) & (0x0FU)) | ((_m->Accel_Z_ro & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->Accel_Z_ro >> 4U) & (0x3FU)) | ((_m->Roll_Rate_ro & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->Roll_Rate_ro >> 2U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->Roll_Rate_ro >> 10U) & (0x01U)) | ((_m->Yaw_Rate_ro & (0x7FU)) << 1U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->Yaw_Rate_ro >> 7U) & (0x0FU)) | ((_m->Pitch_Rate_ro & (0x0FU)) << 4U) );
  cframe->Data[7] |= (uint8_t) ( ((_m->Pitch_Rate_ro >> 4U) & (0x7FU)) );

  cframe->MsgId = (uint32_t) IMU_CANID;
  cframe->DLC = (uint8_t) IMU_DLC;
  cframe->IDE = (uint8_t) IMU_IDE;
  return IMU_CANID;
}

#else

uint32_t Pack_IMU_hytech(IMU_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(IMU_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Accel_X_ro = (int16_t) HYTECH_Accel_X_ro_toS(_m->Accel_X_phys);
  _m->Accel_Y_ro = (int16_t) HYTECH_Accel_Y_ro_toS(_m->Accel_Y_phys);
  _m->Accel_Z_ro = (int16_t) HYTECH_Accel_Z_ro_toS(_m->Accel_Z_phys);
  _m->Roll_Rate_ro = (int16_t) HYTECH_Roll_Rate_ro_toS(_m->Roll_Rate_phys);
  _m->Yaw_Rate_ro = (int16_t) HYTECH_Yaw_Rate_ro_toS(_m->Yaw_Rate_phys);
  _m->Pitch_Rate_ro = (int16_t) HYTECH_Pitch_Rate_ro_toS(_m->Pitch_Rate_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Accel_X_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->Accel_X_ro >> 8U) & (0x03U)) | ((_m->Accel_Y_ro & (0x3FU)) << 2U) );
  _d[2] |= (uint8_t) ( ((_m->Accel_Y_ro >> 6U) & (0x0FU)) | ((_m->Accel_Z_ro & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->Accel_Z_ro >> 4U) & (0x3FU)) | ((_m->Roll_Rate_ro & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( ((_m->Roll_Rate_ro >> 2U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->Roll_Rate_ro >> 10U) & (0x01U)) | ((_m->Yaw_Rate_ro & (0x7FU)) << 1U) );
  _d[6] |= (uint8_t) ( ((_m->Yaw_Rate_ro >> 7U) & (0x0FU)) | ((_m->Pitch_Rate_ro & (0x0FU)) << 4U) );
  _d[7] |= (uint8_t) ( ((_m->Pitch_Rate_ro >> 4U) & (0x7FU)) );

  *_len = (uint8_t) IMU_DLC;
  *_ide = (uint8_t) IMU_IDE;
  return IMU_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_State_hytech(State_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Temperature_Internal_ro = (int8_t) __ext_sig__(( (_d[0] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Temperature_Internal_phys = (sigfloat_t)(HYTECH_Temperature_Internal_ro_fromS(_m->Temperature_Internal_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Firmware_Version = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->sensor_0_Validity = (uint8_t) ( (_d[3] & (0x03U)) );
  _m->sensor_1_Validity = (uint8_t) ( ((_d[3] >> 2U) & (0x03U)) );
  _m->sensor_2_Validity = (uint8_t) ( ((_d[3] >> 4U) & (0x03U)) );
  _m->sensor_3_Validity = (uint8_t) ( ((_d[3] >> 6U) & (0x03U)) );
  _m->SpeedBeam_Health = (uint8_t) ( (_d[4] & (0x07U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < State_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_State_hytech(&_m->mon1, State_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return State_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_State_hytech(State_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(State_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Temperature_Internal_ro = (int8_t) HYTECH_Temperature_Internal_ro_toS(_m->Temperature_Internal_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Temperature_Internal_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->Firmware_Version & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->Firmware_Version >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->sensor_0_Validity & (0x03U)) | ((_m->sensor_1_Validity & (0x03U)) << 2U) | ((_m->sensor_2_Validity & (0x03U)) << 4U) | ((_m->sensor_3_Validity & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( (_m->SpeedBeam_Health & (0x07U)) );

  cframe->MsgId = (uint32_t) State_CANID;
  cframe->DLC = (uint8_t) State_DLC;
  cframe->IDE = (uint8_t) State_IDE;
  return State_CANID;
}

#else

uint32_t Pack_State_hytech(State_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(State_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Temperature_Internal_ro = (int8_t) HYTECH_Temperature_Internal_ro_toS(_m->Temperature_Internal_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Temperature_Internal_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->Firmware_Version & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->Firmware_Version >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->sensor_0_Validity & (0x03U)) | ((_m->sensor_1_Validity & (0x03U)) << 2U) | ((_m->sensor_2_Validity & (0x03U)) << 4U) | ((_m->sensor_3_Validity & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( (_m->SpeedBeam_Health & (0x07U)) );

  *_len = (uint8_t) State_DLC;
  *_ide = (uint8_t) State_IDE;
  return State_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Settings_hytech(Settings_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Broadcast_rate = (uint8_t) ( (_d[0] & (0x07U)) );
  _m->CAN_Termination_Setting_Bus_1 = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->CAN_Termination_Setting_Bus_2 = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->Height_Offset_ro = (int8_t) __ext_sig__(( ((_d[1] & (0x1FU)) << 3U) | ((_d[0] >> 5U) & (0x07U)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_phys = (sigfloat_t)(HYTECH_Height_Offset_ro_fromS(_m->Height_Offset_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Sensor_X_Location_ro = (int16_t) __ext_sig__(( ((_d[3] & (0x01U)) << 11U) | ((_d[2] & (0xFFU)) << 3U) | ((_d[1] >> 5U) & (0x07U)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Sensor_X_Location_phys = (sigfloat_t)(HYTECH_Sensor_X_Location_ro_fromS(_m->Sensor_X_Location_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Sensor_Y_Location_ro = (int16_t) __ext_sig__(( ((_d[4] & (0x1FU)) << 7U) | ((_d[3] >> 1U) & (0x7FU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Sensor_Y_Location_phys = (sigfloat_t)(HYTECH_Sensor_Y_Location_ro_fromS(_m->Sensor_Y_Location_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Attitude_Filtering = (uint8_t) ( ((_d[4] >> 5U) & (0x07U)) );
  _m->IMU_Filtering = (uint8_t) ( (_d[5] & (0x07U)) );
  _m->Velocity_Filtering = (uint8_t) ( ((_d[5] >> 3U) & (0x07U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < Settings_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_Settings_hytech(&_m->mon1, Settings_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return Settings_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_Settings_hytech(Settings_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Settings_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_ro = (int8_t) HYTECH_Height_Offset_ro_toS(_m->Height_Offset_phys);
  _m->Sensor_X_Location_ro = (int16_t) HYTECH_Sensor_X_Location_ro_toS(_m->Sensor_X_Location_phys);
  _m->Sensor_Y_Location_ro = (int16_t) HYTECH_Sensor_Y_Location_ro_toS(_m->Sensor_Y_Location_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Broadcast_rate & (0x07U)) | ((_m->CAN_Termination_Setting_Bus_1 & (0x01U)) << 3U) | ((_m->CAN_Termination_Setting_Bus_2 & (0x01U)) << 4U) | ((_m->Height_Offset_ro & (0x07U)) << 5U) );
  cframe->Data[1] |= (uint8_t) ( ((_m->Height_Offset_ro >> 3U) & (0x1FU)) | ((_m->Sensor_X_Location_ro & (0x07U)) << 5U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 3U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 11U) & (0x01U)) | ((_m->Sensor_Y_Location_ro & (0x7FU)) << 1U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->Sensor_Y_Location_ro >> 7U) & (0x1FU)) | ((_m->Attitude_Filtering & (0x07U)) << 5U) );
  cframe->Data[5] |= (uint8_t) ( (_m->IMU_Filtering & (0x07U)) | ((_m->Velocity_Filtering & (0x07U)) << 3U) );

  cframe->MsgId = (uint32_t) Settings_CANID;
  cframe->DLC = (uint8_t) Settings_DLC;
  cframe->IDE = (uint8_t) Settings_IDE;
  return Settings_CANID;
}

#else

uint32_t Pack_Settings_hytech(Settings_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Settings_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_ro = (int8_t) HYTECH_Height_Offset_ro_toS(_m->Height_Offset_phys);
  _m->Sensor_X_Location_ro = (int16_t) HYTECH_Sensor_X_Location_ro_toS(_m->Sensor_X_Location_phys);
  _m->Sensor_Y_Location_ro = (int16_t) HYTECH_Sensor_Y_Location_ro_toS(_m->Sensor_Y_Location_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Broadcast_rate & (0x07U)) | ((_m->CAN_Termination_Setting_Bus_1 & (0x01U)) << 3U) | ((_m->CAN_Termination_Setting_Bus_2 & (0x01U)) << 4U) | ((_m->Height_Offset_ro & (0x07U)) << 5U) );
  _d[1] |= (uint8_t) ( ((_m->Height_Offset_ro >> 3U) & (0x1FU)) | ((_m->Sensor_X_Location_ro & (0x07U)) << 5U) );
  _d[2] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 3U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 11U) & (0x01U)) | ((_m->Sensor_Y_Location_ro & (0x7FU)) << 1U) );
  _d[4] |= (uint8_t) ( ((_m->Sensor_Y_Location_ro >> 7U) & (0x1FU)) | ((_m->Attitude_Filtering & (0x07U)) << 5U) );
  _d[5] |= (uint8_t) ( (_m->IMU_Filtering & (0x07U)) | ((_m->Velocity_Filtering & (0x07U)) << 3U) );

  *_len = (uint8_t) Settings_DLC;
  *_ide = (uint8_t) Settings_IDE;
  return Settings_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Velocities_hytech(Velocities_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Velocity_x_ro = (int16_t) __ext_sig__(( ((_d[1] & (0x0FU)) << 8U) | (_d[0] & (0xFFU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_x_phys = (sigfloat_t)(HYTECH_Velocity_x_ro_fromS(_m->Velocity_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Velocity_y_ro = (int16_t) __ext_sig__(( ((_d[2] & (0x3FU)) << 4U) | ((_d[1] >> 4U) & (0x0FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_y_phys = (sigfloat_t)(HYTECH_Velocity_y_ro_fromS(_m->Velocity_y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Velocity_z_ro = (int16_t) __ext_sig__(( ((_d[3] & (0x7FU)) << 2U) | ((_d[2] >> 6U) & (0x03U)) ), 9);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_z_phys = (sigfloat_t)(HYTECH_Velocity_z_ro_fromS(_m->Velocity_z_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Velocity_x_CGCorrected_ro = (int16_t) __ext_sig__(( ((_d[5] & (0x07U)) << 9U) | ((_d[4] & (0xFFU)) << 1U) | ((_d[3] >> 7U) & (0x01U)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_x_CGCorrected_phys = (sigfloat_t)(HYTECH_Velocity_x_CGCorrected_ro_fromS(_m->Velocity_x_CGCorrected_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Velocity_y_CGCorrected_ro = (int16_t) __ext_sig__(( ((_d[6] & (0x1FU)) << 5U) | ((_d[5] >> 3U) & (0x1FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_y_CGCorrected_phys = (sigfloat_t)(HYTECH_Velocity_y_CGCorrected_ro_fromS(_m->Velocity_y_CGCorrected_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Velocity_z_CGCorrected_ro = (int16_t) __ext_sig__(( ((_d[7] & (0x3FU)) << 3U) | ((_d[6] >> 5U) & (0x07U)) ), 9);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_z_CGCorrected_phys = (sigfloat_t)(HYTECH_Velocity_z_CGCorrected_ro_fromS(_m->Velocity_z_CGCorrected_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < Velocities_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_Velocities_hytech(&_m->mon1, Velocities_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return Velocities_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_Velocities_hytech(Velocities_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Velocities_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_x_ro = (int16_t) HYTECH_Velocity_x_ro_toS(_m->Velocity_x_phys);
  _m->Velocity_y_ro = (int16_t) HYTECH_Velocity_y_ro_toS(_m->Velocity_y_phys);
  _m->Velocity_z_ro = (int16_t) HYTECH_Velocity_z_ro_toS(_m->Velocity_z_phys);
  _m->Velocity_x_CGCorrected_ro = (int16_t) HYTECH_Velocity_x_CGCorrected_ro_toS(_m->Velocity_x_CGCorrected_phys);
  _m->Velocity_y_CGCorrected_ro = (int16_t) HYTECH_Velocity_y_CGCorrected_ro_toS(_m->Velocity_y_CGCorrected_phys);
  _m->Velocity_z_CGCorrected_ro = (int16_t) HYTECH_Velocity_z_CGCorrected_ro_toS(_m->Velocity_z_CGCorrected_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Velocity_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->Velocity_x_ro >> 8U) & (0x0FU)) | ((_m->Velocity_y_ro & (0x0FU)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->Velocity_y_ro >> 4U) & (0x3FU)) | ((_m->Velocity_z_ro & (0x03U)) << 6U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->Velocity_z_ro >> 2U) & (0x7FU)) | ((_m->Velocity_x_CGCorrected_ro & (0x01U)) << 7U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->Velocity_x_CGCorrected_ro >> 1U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->Velocity_x_CGCorrected_ro >> 9U) & (0x07U)) | ((_m->Velocity_y_CGCorrected_ro & (0x1FU)) << 3U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->Velocity_y_CGCorrected_ro >> 5U) & (0x1FU)) | ((_m->Velocity_z_CGCorrected_ro & (0x07U)) << 5U) );
  cframe->Data[7] |= (uint8_t) ( ((_m->Velocity_z_CGCorrected_ro >> 3U) & (0x3FU)) );

  cframe->MsgId = (uint32_t) Velocities_CANID;
  cframe->DLC = (uint8_t) Velocities_DLC;
  cframe->IDE = (uint8_t) Velocities_IDE;
  return Velocities_CANID;
}

#else

uint32_t Pack_Velocities_hytech(Velocities_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Velocities_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Velocity_x_ro = (int16_t) HYTECH_Velocity_x_ro_toS(_m->Velocity_x_phys);
  _m->Velocity_y_ro = (int16_t) HYTECH_Velocity_y_ro_toS(_m->Velocity_y_phys);
  _m->Velocity_z_ro = (int16_t) HYTECH_Velocity_z_ro_toS(_m->Velocity_z_phys);
  _m->Velocity_x_CGCorrected_ro = (int16_t) HYTECH_Velocity_x_CGCorrected_ro_toS(_m->Velocity_x_CGCorrected_phys);
  _m->Velocity_y_CGCorrected_ro = (int16_t) HYTECH_Velocity_y_CGCorrected_ro_toS(_m->Velocity_y_CGCorrected_phys);
  _m->Velocity_z_CGCorrected_ro = (int16_t) HYTECH_Velocity_z_CGCorrected_ro_toS(_m->Velocity_z_CGCorrected_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Velocity_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->Velocity_x_ro >> 8U) & (0x0FU)) | ((_m->Velocity_y_ro & (0x0FU)) << 4U) );
  _d[2] |= (uint8_t) ( ((_m->Velocity_y_ro >> 4U) & (0x3FU)) | ((_m->Velocity_z_ro & (0x03U)) << 6U) );
  _d[3] |= (uint8_t) ( ((_m->Velocity_z_ro >> 2U) & (0x7FU)) | ((_m->Velocity_x_CGCorrected_ro & (0x01U)) << 7U) );
  _d[4] |= (uint8_t) ( ((_m->Velocity_x_CGCorrected_ro >> 1U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->Velocity_x_CGCorrected_ro >> 9U) & (0x07U)) | ((_m->Velocity_y_CGCorrected_ro & (0x1FU)) << 3U) );
  _d[6] |= (uint8_t) ( ((_m->Velocity_y_CGCorrected_ro >> 5U) & (0x1FU)) | ((_m->Velocity_z_CGCorrected_ro & (0x07U)) << 5U) );
  _d[7] |= (uint8_t) ( ((_m->Velocity_z_CGCorrected_ro >> 3U) & (0x3FU)) );

  *_len = (uint8_t) Velocities_DLC;
  *_ide = (uint8_t) Velocities_IDE;
  return Velocities_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Attitude_hytech(Attitude_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Height_ro = (int16_t) __ext_sig__(( ((_d[1] & (0x0FU)) << 8U) | (_d[0] & (0xFFU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_phys = (sigfloat_t)(HYTECH_Height_ro_fromS(_m->Height_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Pitch_angle_ro = (int16_t) __ext_sig__(( ((_d[3] & (0x01U)) << 8U) | (_d[2] & (0xFFU)) ), 9);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Pitch_angle_phys = (sigfloat_t)(HYTECH_Pitch_angle_ro_fromS(_m->Pitch_angle_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Roll_angle_ro = (int16_t) __ext_sig__(( ((_d[5] & (0x01U)) << 8U) | (_d[4] & (0xFFU)) ), 9);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Roll_angle_phys = (sigfloat_t)(HYTECH_Roll_angle_ro_fromS(_m->Roll_angle_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < Attitude_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_Attitude_hytech(&_m->mon1, Attitude_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return Attitude_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_Attitude_hytech(Attitude_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Attitude_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_ro = (int16_t) HYTECH_Height_ro_toS(_m->Height_phys);
  _m->Pitch_angle_ro = (int16_t) HYTECH_Pitch_angle_ro_toS(_m->Pitch_angle_phys);
  _m->Roll_angle_ro = (int16_t) HYTECH_Roll_angle_ro_toS(_m->Roll_angle_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Height_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->Height_ro >> 8U) & (0x0FU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->Pitch_angle_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->Pitch_angle_ro >> 8U) & (0x01U)) );
  cframe->Data[4] |= (uint8_t) ( (_m->Roll_angle_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->Roll_angle_ro >> 8U) & (0x01U)) );

  cframe->MsgId = (uint32_t) Attitude_CANID;
  cframe->DLC = (uint8_t) Attitude_DLC;
  cframe->IDE = (uint8_t) Attitude_IDE;
  return Attitude_CANID;
}

#else

uint32_t Pack_Attitude_hytech(Attitude_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Attitude_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_ro = (int16_t) HYTECH_Height_ro_toS(_m->Height_phys);
  _m->Pitch_angle_ro = (int16_t) HYTECH_Pitch_angle_ro_toS(_m->Pitch_angle_phys);
  _m->Roll_angle_ro = (int16_t) HYTECH_Roll_angle_ro_toS(_m->Roll_angle_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Height_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->Height_ro >> 8U) & (0x0FU)) );
  _d[2] |= (uint8_t) ( (_m->Pitch_angle_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->Pitch_angle_ro >> 8U) & (0x01U)) );
  _d[4] |= (uint8_t) ( (_m->Roll_angle_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->Roll_angle_ro >> 8U) & (0x01U)) );

  *_len = (uint8_t) Attitude_DLC;
  *_ide = (uint8_t) Attitude_IDE;
  return Attitude_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_Configuration_hytech(Configuration_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Height_Offset_ro = (int8_t) __ext_sig__(( (_d[0] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_phys = (sigfloat_t)(HYTECH_Height_Offset_ro_fromS(_m->Height_Offset_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Sensor_Y_Location_ro = (int16_t) __ext_sig__(( ((_d[2] & (0x0FU)) << 8U) | (_d[1] & (0xFFU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Sensor_Y_Location_phys = (sigfloat_t)(HYTECH_Sensor_Y_Location_ro_fromS(_m->Sensor_Y_Location_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Sensor_X_Location_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->Sensor_X_Location_phys = (sigfloat_t)(HYTECH_Sensor_X_Location_ro_fromS(_m->Sensor_X_Location_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->Attitude_Filtering = (uint8_t) ( (_d[4] & (0x07U)) );
  _m->IMU_Filtering = (uint8_t) ( ((_d[4] >> 3U) & (0x07U)) );
  _m->CAN_Termination_State_Bus_1 = (uint8_t) ( ((_d[4] >> 6U) & (0x01U)) );
  _m->CAN_Termination_State_Bus_2 = (uint8_t) ( ((_d[4] >> 7U) & (0x01U)) );
  _m->Velocity_Filtering = (uint8_t) ( (_d[5] & (0x07U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < Configuration_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_Configuration_hytech(&_m->mon1, Configuration_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return Configuration_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_Configuration_hytech(Configuration_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Configuration_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_ro = (int8_t) HYTECH_Height_Offset_ro_toS(_m->Height_Offset_phys);
  _m->Sensor_Y_Location_ro = (int16_t) HYTECH_Sensor_Y_Location_ro_toS(_m->Sensor_Y_Location_phys);
  _m->Sensor_X_Location_ro = (int16_t) HYTECH_Sensor_X_Location_ro_toS(_m->Sensor_X_Location_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->Height_Offset_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->Sensor_Y_Location_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->Sensor_Y_Location_ro >> 8U) & (0x0FU)) | ((_m->Sensor_X_Location_ro & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 4U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->Attitude_Filtering & (0x07U)) | ((_m->IMU_Filtering & (0x07U)) << 3U) | ((_m->CAN_Termination_State_Bus_1 & (0x01U)) << 6U) | ((_m->CAN_Termination_State_Bus_2 & (0x01U)) << 7U) );
  cframe->Data[5] |= (uint8_t) ( (_m->Velocity_Filtering & (0x07U)) );

  cframe->MsgId = (uint32_t) Configuration_CANID;
  cframe->DLC = (uint8_t) Configuration_DLC;
  cframe->IDE = (uint8_t) Configuration_IDE;
  return Configuration_CANID;
}

#else

uint32_t Pack_Configuration_hytech(Configuration_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(Configuration_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->Height_Offset_ro = (int8_t) HYTECH_Height_Offset_ro_toS(_m->Height_Offset_phys);
  _m->Sensor_Y_Location_ro = (int16_t) HYTECH_Sensor_Y_Location_ro_toS(_m->Sensor_Y_Location_phys);
  _m->Sensor_X_Location_ro = (int16_t) HYTECH_Sensor_X_Location_ro_toS(_m->Sensor_X_Location_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->Height_Offset_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->Sensor_Y_Location_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->Sensor_Y_Location_ro >> 8U) & (0x0FU)) | ((_m->Sensor_X_Location_ro & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->Sensor_X_Location_ro >> 4U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->Attitude_Filtering & (0x07U)) | ((_m->IMU_Filtering & (0x07U)) << 3U) | ((_m->CAN_Termination_State_Bus_1 & (0x01U)) << 6U) | ((_m->CAN_Termination_State_Bus_2 & (0x01U)) << 7U) );
  _d[5] |= (uint8_t) ( (_m->Velocity_Filtering & (0x07U)) );

  *_len = (uint8_t) Configuration_DLC;
  *_ide = (uint8_t) Configuration_IDE;
  return Configuration_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_STATUS_hytech(INV1_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->system_ready = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->warning = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->quit_dc_on = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->dc_on = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->quit_inverter_on = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->inverter_on = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->derating_on = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->dc_bus_voltage = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->diagnostic_number = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_STATUS_hytech(&_m->mon1, INV1_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_STATUS_hytech(INV1_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_STATUS_CANID;
  cframe->DLC = (uint8_t) INV1_STATUS_DLC;
  cframe->IDE = (uint8_t) INV1_STATUS_IDE;
  return INV1_STATUS_CANID;
}

#else

uint32_t Pack_INV1_STATUS_hytech(INV1_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_STATUS_DLC;
  *_ide = (uint8_t) INV1_STATUS_IDE;
  return INV1_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_phys = (sigfloat_t)(HYTECH_motor_temp_ro_fromS(_m->motor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->inverter_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_temp_phys = (sigfloat_t)(HYTECH_inverter_temp_ro_fromS(_m->inverter_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->igbt_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->igbt_temp_phys = (sigfloat_t)(HYTECH_igbt_temp_ro_fromS(_m->igbt_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_TEMPS_hytech(&_m->mon1, INV1_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_TEMPS_CANID;
  cframe->DLC = (uint8_t) INV1_TEMPS_DLC;
  cframe->IDE = (uint8_t) INV1_TEMPS_IDE;
  return INV1_TEMPS_CANID;
}

#else

uint32_t Pack_INV1_TEMPS_hytech(INV1_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_TEMPS_DLC;
  *_ide = (uint8_t) INV1_TEMPS_IDE;
  return INV1_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->actual_power_w = (uint32_t) ( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->actual_torque_nm_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_phys = (sigfloat_t)(HYTECH_actual_torque_nm_ro_fromS(_m->actual_torque_nm_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->actual_speed_rpm = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_DYNAMICS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_DYNAMICS_hytech(&_m->mon1, INV1_DYNAMICS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_DYNAMICS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_DYNAMICS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_DYNAMICS_CANID;
  cframe->DLC = (uint8_t) INV1_DYNAMICS_DLC;
  cframe->IDE = (uint8_t) INV1_DYNAMICS_IDE;
  return INV1_DYNAMICS_CANID;
}

#else

uint32_t Pack_INV1_DYNAMICS_hytech(INV1_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_DYNAMICS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_DYNAMICS_DLC;
  *_ide = (uint8_t) INV1_DYNAMICS_IDE;
  return INV1_DYNAMICS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_POWER_hytech(INV1_POWER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->active_power_w = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->reactive_power_var = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_POWER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_POWER_hytech(&_m->mon1, INV1_POWER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_POWER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_POWER_hytech(INV1_POWER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_POWER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_POWER_CANID;
  cframe->DLC = (uint8_t) INV1_POWER_DLC;
  cframe->IDE = (uint8_t) INV1_POWER_IDE;
  return INV1_POWER_CANID;
}

#else

uint32_t Pack_INV1_POWER_hytech(INV1_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_POWER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  *_len = (uint8_t) INV1_POWER_DLC;
  *_ide = (uint8_t) INV1_POWER_IDE;
  return INV1_POWER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_FEEDBACK_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_FEEDBACK_hytech(&_m->mon1, INV1_FEEDBACK_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_FEEDBACK_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_FEEDBACK_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_FEEDBACK_CANID;
  cframe->DLC = (uint8_t) INV1_FEEDBACK_DLC;
  cframe->IDE = (uint8_t) INV1_FEEDBACK_IDE;
  return INV1_FEEDBACK_CANID;
}

#else

uint32_t Pack_INV1_FEEDBACK_hytech(INV1_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_FEEDBACK_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_FEEDBACK_DLC;
  *_ide = (uint8_t) INV1_FEEDBACK_IDE;
  return INV1_FEEDBACK_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_STATUS_hytech(INV2_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->system_ready = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->warning = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->quit_dc_on = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->dc_on = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->quit_inverter_on = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->inverter_on = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->derating_on = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->dc_bus_voltage = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->diagnostic_number = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_STATUS_hytech(&_m->mon1, INV2_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_STATUS_hytech(INV2_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_STATUS_CANID;
  cframe->DLC = (uint8_t) INV2_STATUS_DLC;
  cframe->IDE = (uint8_t) INV2_STATUS_IDE;
  return INV2_STATUS_CANID;
}

#else

uint32_t Pack_INV2_STATUS_hytech(INV2_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_STATUS_DLC;
  *_ide = (uint8_t) INV2_STATUS_IDE;
  return INV2_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_phys = (sigfloat_t)(HYTECH_motor_temp_ro_fromS(_m->motor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->inverter_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_temp_phys = (sigfloat_t)(HYTECH_inverter_temp_ro_fromS(_m->inverter_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->igbt_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->igbt_temp_phys = (sigfloat_t)(HYTECH_igbt_temp_ro_fromS(_m->igbt_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_TEMPS_hytech(&_m->mon1, INV2_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_TEMPS_CANID;
  cframe->DLC = (uint8_t) INV2_TEMPS_DLC;
  cframe->IDE = (uint8_t) INV2_TEMPS_IDE;
  return INV2_TEMPS_CANID;
}

#else

uint32_t Pack_INV2_TEMPS_hytech(INV2_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_TEMPS_DLC;
  *_ide = (uint8_t) INV2_TEMPS_IDE;
  return INV2_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->actual_power_w = (uint32_t) ( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->actual_torque_nm_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_phys = (sigfloat_t)(HYTECH_actual_torque_nm_ro_fromS(_m->actual_torque_nm_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->actual_speed_rpm = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_DYNAMICS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_DYNAMICS_hytech(&_m->mon1, INV2_DYNAMICS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_DYNAMICS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_DYNAMICS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_DYNAMICS_CANID;
  cframe->DLC = (uint8_t) INV2_DYNAMICS_DLC;
  cframe->IDE = (uint8_t) INV2_DYNAMICS_IDE;
  return INV2_DYNAMICS_CANID;
}

#else

uint32_t Pack_INV2_DYNAMICS_hytech(INV2_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_DYNAMICS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_DYNAMICS_DLC;
  *_ide = (uint8_t) INV2_DYNAMICS_IDE;
  return INV2_DYNAMICS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_POWER_hytech(INV2_POWER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->active_power_w = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->reactive_power_var = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_POWER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_POWER_hytech(&_m->mon1, INV2_POWER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_POWER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_POWER_hytech(INV2_POWER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_POWER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_POWER_CANID;
  cframe->DLC = (uint8_t) INV2_POWER_DLC;
  cframe->IDE = (uint8_t) INV2_POWER_IDE;
  return INV2_POWER_CANID;
}

#else

uint32_t Pack_INV2_POWER_hytech(INV2_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_POWER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  *_len = (uint8_t) INV2_POWER_DLC;
  *_ide = (uint8_t) INV2_POWER_IDE;
  return INV2_POWER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_FEEDBACK_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_FEEDBACK_hytech(&_m->mon1, INV2_FEEDBACK_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_FEEDBACK_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_FEEDBACK_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_FEEDBACK_CANID;
  cframe->DLC = (uint8_t) INV2_FEEDBACK_DLC;
  cframe->IDE = (uint8_t) INV2_FEEDBACK_IDE;
  return INV2_FEEDBACK_CANID;
}

#else

uint32_t Pack_INV2_FEEDBACK_hytech(INV2_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_FEEDBACK_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_FEEDBACK_DLC;
  *_ide = (uint8_t) INV2_FEEDBACK_IDE;
  return INV2_FEEDBACK_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_STATUS_hytech(INV3_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->system_ready = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->warning = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->quit_dc_on = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->dc_on = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->quit_inverter_on = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->inverter_on = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->derating_on = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->dc_bus_voltage = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->diagnostic_number = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_STATUS_hytech(&_m->mon1, INV3_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_STATUS_hytech(INV3_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_STATUS_CANID;
  cframe->DLC = (uint8_t) INV3_STATUS_DLC;
  cframe->IDE = (uint8_t) INV3_STATUS_IDE;
  return INV3_STATUS_CANID;
}

#else

uint32_t Pack_INV3_STATUS_hytech(INV3_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_STATUS_DLC;
  *_ide = (uint8_t) INV3_STATUS_IDE;
  return INV3_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_phys = (sigfloat_t)(HYTECH_motor_temp_ro_fromS(_m->motor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->inverter_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_temp_phys = (sigfloat_t)(HYTECH_inverter_temp_ro_fromS(_m->inverter_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->igbt_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->igbt_temp_phys = (sigfloat_t)(HYTECH_igbt_temp_ro_fromS(_m->igbt_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_TEMPS_hytech(&_m->mon1, INV3_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_TEMPS_CANID;
  cframe->DLC = (uint8_t) INV3_TEMPS_DLC;
  cframe->IDE = (uint8_t) INV3_TEMPS_IDE;
  return INV3_TEMPS_CANID;
}

#else

uint32_t Pack_INV3_TEMPS_hytech(INV3_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_TEMPS_DLC;
  *_ide = (uint8_t) INV3_TEMPS_IDE;
  return INV3_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->actual_power_w = (uint32_t) ( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->actual_torque_nm_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_phys = (sigfloat_t)(HYTECH_actual_torque_nm_ro_fromS(_m->actual_torque_nm_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->actual_speed_rpm = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_DYNAMICS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_DYNAMICS_hytech(&_m->mon1, INV3_DYNAMICS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_DYNAMICS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_DYNAMICS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_DYNAMICS_CANID;
  cframe->DLC = (uint8_t) INV3_DYNAMICS_DLC;
  cframe->IDE = (uint8_t) INV3_DYNAMICS_IDE;
  return INV3_DYNAMICS_CANID;
}

#else

uint32_t Pack_INV3_DYNAMICS_hytech(INV3_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_DYNAMICS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_DYNAMICS_DLC;
  *_ide = (uint8_t) INV3_DYNAMICS_IDE;
  return INV3_DYNAMICS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_POWER_hytech(INV3_POWER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->active_power_w = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->reactive_power_var = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_POWER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_POWER_hytech(&_m->mon1, INV3_POWER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_POWER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_POWER_hytech(INV3_POWER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_POWER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_POWER_CANID;
  cframe->DLC = (uint8_t) INV3_POWER_DLC;
  cframe->IDE = (uint8_t) INV3_POWER_IDE;
  return INV3_POWER_CANID;
}

#else

uint32_t Pack_INV3_POWER_hytech(INV3_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_POWER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  *_len = (uint8_t) INV3_POWER_DLC;
  *_ide = (uint8_t) INV3_POWER_IDE;
  return INV3_POWER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_FEEDBACK_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_FEEDBACK_hytech(&_m->mon1, INV3_FEEDBACK_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_FEEDBACK_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_FEEDBACK_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_FEEDBACK_CANID;
  cframe->DLC = (uint8_t) INV3_FEEDBACK_DLC;
  cframe->IDE = (uint8_t) INV3_FEEDBACK_IDE;
  return INV3_FEEDBACK_CANID;
}

#else

uint32_t Pack_INV3_FEEDBACK_hytech(INV3_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_FEEDBACK_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_FEEDBACK_DLC;
  *_ide = (uint8_t) INV3_FEEDBACK_IDE;
  return INV3_FEEDBACK_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_STATUS_hytech(INV4_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->system_ready = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->warning = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->quit_dc_on = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->dc_on = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->quit_inverter_on = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->inverter_on = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->derating_on = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->dc_bus_voltage = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->diagnostic_number = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_STATUS_hytech(&_m->mon1, INV4_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_STATUS_hytech(INV4_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_STATUS_CANID;
  cframe->DLC = (uint8_t) INV4_STATUS_DLC;
  cframe->IDE = (uint8_t) INV4_STATUS_IDE;
  return INV4_STATUS_CANID;
}

#else

uint32_t Pack_INV4_STATUS_hytech(INV4_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->system_ready & (0x01U)) | ((_m->error & (0x01U)) << 1U) | ((_m->warning & (0x01U)) << 2U) | ((_m->quit_dc_on & (0x01U)) << 3U) | ((_m->dc_on & (0x01U)) << 4U) | ((_m->quit_inverter_on & (0x01U)) << 5U) | ((_m->inverter_on & (0x01U)) << 6U) | ((_m->derating_on & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->diagnostic_number & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->diagnostic_number >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_STATUS_DLC;
  *_ide = (uint8_t) INV4_STATUS_IDE;
  return INV4_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->motor_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_phys = (sigfloat_t)(HYTECH_motor_temp_ro_fromS(_m->motor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->inverter_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_temp_phys = (sigfloat_t)(HYTECH_inverter_temp_ro_fromS(_m->inverter_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->igbt_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->igbt_temp_phys = (sigfloat_t)(HYTECH_igbt_temp_ro_fromS(_m->igbt_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_TEMPS_hytech(&_m->mon1, INV4_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_TEMPS_CANID;
  cframe->DLC = (uint8_t) INV4_TEMPS_DLC;
  cframe->IDE = (uint8_t) INV4_TEMPS_IDE;
  return INV4_TEMPS_CANID;
}

#else

uint32_t Pack_INV4_TEMPS_hytech(INV4_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_temp_ro = (int16_t) HYTECH_motor_temp_ro_toS(_m->motor_temp_phys);
  _m->inverter_temp_ro = (int16_t) HYTECH_inverter_temp_ro_toS(_m->inverter_temp_phys);
  _m->igbt_temp_ro = (int16_t) HYTECH_igbt_temp_ro_toS(_m->igbt_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->motor_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->motor_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->inverter_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->inverter_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->igbt_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->igbt_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_TEMPS_DLC;
  *_ide = (uint8_t) INV4_TEMPS_IDE;
  return INV4_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->actual_power_w = (uint32_t) ( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->actual_torque_nm_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_phys = (sigfloat_t)(HYTECH_actual_torque_nm_ro_fromS(_m->actual_torque_nm_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->actual_speed_rpm = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_DYNAMICS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_DYNAMICS_hytech(&_m->mon1, INV4_DYNAMICS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_DYNAMICS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_DYNAMICS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_DYNAMICS_CANID;
  cframe->DLC = (uint8_t) INV4_DYNAMICS_DLC;
  cframe->IDE = (uint8_t) INV4_DYNAMICS_IDE;
  return INV4_DYNAMICS_CANID;
}

#else

uint32_t Pack_INV4_DYNAMICS_hytech(INV4_DYNAMICS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_DYNAMICS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->actual_torque_nm_ro = (int16_t) HYTECH_actual_torque_nm_ro_toS(_m->actual_torque_nm_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->actual_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->actual_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->actual_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->actual_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->actual_torque_nm_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->actual_torque_nm_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->actual_speed_rpm & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->actual_speed_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_DYNAMICS_DLC;
  *_ide = (uint8_t) INV4_DYNAMICS_IDE;
  return INV4_DYNAMICS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_POWER_hytech(INV4_POWER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->active_power_w = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
  _m->reactive_power_var = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_POWER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_POWER_hytech(&_m->mon1, INV4_POWER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_POWER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_POWER_hytech(INV4_POWER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_POWER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_POWER_CANID;
  cframe->DLC = (uint8_t) INV4_POWER_DLC;
  cframe->IDE = (uint8_t) INV4_POWER_IDE;
  return INV4_POWER_CANID;
}

#else

uint32_t Pack_INV4_POWER_hytech(INV4_POWER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_POWER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->active_power_w & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->active_power_w >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->active_power_w >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->active_power_w >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->reactive_power_var & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->reactive_power_var >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->reactive_power_var >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->reactive_power_var >> 24U) & (0xFFU)) );

  *_len = (uint8_t) INV4_POWER_DLC;
  *_ide = (uint8_t) INV4_POWER_IDE;
  return INV4_POWER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_FEEDBACK_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_FEEDBACK_hytech(&_m->mon1, INV4_FEEDBACK_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_FEEDBACK_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_FEEDBACK_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_FEEDBACK_CANID;
  cframe->DLC = (uint8_t) INV4_FEEDBACK_DLC;
  cframe->IDE = (uint8_t) INV4_FEEDBACK_IDE;
  return INV4_FEEDBACK_CANID;
}

#else

uint32_t Pack_INV4_FEEDBACK_hytech(INV4_FEEDBACK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_FEEDBACK_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_FEEDBACK_DLC;
  *_ide = (uint8_t) INV4_FEEDBACK_IDE;
  return INV4_FEEDBACK_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_enable = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->hv_enable = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->driver_enable = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->remove_error = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_CONTROL_WORD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_CONTROL_WORD_hytech(&_m->mon1, INV1_CONTROL_WORD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_CONTROL_WORD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_WORD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) INV1_CONTROL_WORD_CANID;
  cframe->DLC = (uint8_t) INV1_CONTROL_WORD_DLC;
  cframe->IDE = (uint8_t) INV1_CONTROL_WORD_IDE;
  return INV1_CONTROL_WORD_CANID;
}

#else

uint32_t Pack_INV1_CONTROL_WORD_hytech(INV1_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_WORD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  *_len = (uint8_t) INV1_CONTROL_WORD_DLC;
  *_ide = (uint8_t) INV1_CONTROL_WORD_IDE;
  return INV1_CONTROL_WORD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_enable = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->hv_enable = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->driver_enable = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->remove_error = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_CONTROL_WORD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_CONTROL_WORD_hytech(&_m->mon1, INV2_CONTROL_WORD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_CONTROL_WORD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_WORD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) INV2_CONTROL_WORD_CANID;
  cframe->DLC = (uint8_t) INV2_CONTROL_WORD_DLC;
  cframe->IDE = (uint8_t) INV2_CONTROL_WORD_IDE;
  return INV2_CONTROL_WORD_CANID;
}

#else

uint32_t Pack_INV2_CONTROL_WORD_hytech(INV2_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_WORD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  *_len = (uint8_t) INV2_CONTROL_WORD_DLC;
  *_ide = (uint8_t) INV2_CONTROL_WORD_IDE;
  return INV2_CONTROL_WORD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_enable = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->hv_enable = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->driver_enable = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->remove_error = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_CONTROL_WORD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_CONTROL_WORD_hytech(&_m->mon1, INV3_CONTROL_WORD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_CONTROL_WORD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_WORD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) INV3_CONTROL_WORD_CANID;
  cframe->DLC = (uint8_t) INV3_CONTROL_WORD_DLC;
  cframe->IDE = (uint8_t) INV3_CONTROL_WORD_IDE;
  return INV3_CONTROL_WORD_CANID;
}

#else

uint32_t Pack_INV3_CONTROL_WORD_hytech(INV3_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_WORD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  *_len = (uint8_t) INV3_CONTROL_WORD_DLC;
  *_ide = (uint8_t) INV3_CONTROL_WORD_IDE;
  return INV3_CONTROL_WORD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_enable = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->hv_enable = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->driver_enable = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->remove_error = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_CONTROL_WORD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_CONTROL_WORD_hytech(&_m->mon1, INV4_CONTROL_WORD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_CONTROL_WORD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_WORD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) INV4_CONTROL_WORD_CANID;
  cframe->DLC = (uint8_t) INV4_CONTROL_WORD_DLC;
  cframe->IDE = (uint8_t) INV4_CONTROL_WORD_IDE;
  return INV4_CONTROL_WORD_CANID;
}

#else

uint32_t Pack_INV4_CONTROL_WORD_hytech(INV4_CONTROL_WORD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_WORD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[1] |= (uint8_t) ( (_m->inverter_enable & (0x01U)) | ((_m->hv_enable & (0x01U)) << 1U) | ((_m->driver_enable & (0x01U)) << 2U) | ((_m->remove_error & (0x01U)) << 3U) );

  *_len = (uint8_t) INV4_CONTROL_WORD_DLC;
  *_ide = (uint8_t) INV4_CONTROL_WORD_IDE;
  return INV4_CONTROL_WORD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->driver_message = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->driver_msg_var_1 = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->driver_msg_var_2 = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->target_lap_time = (uint32_t) ( ((_d[7] & (0xFFU)) << 16U) | ((_d[6] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TCU_DRIVER_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TCU_DRIVER_MSG_hytech(&_m->mon1, TCU_DRIVER_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TCU_DRIVER_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_DRIVER_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->driver_message & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->driver_msg_var_1 & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->driver_msg_var_1 >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->driver_msg_var_2 & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->driver_msg_var_2 >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->target_lap_time & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->target_lap_time >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->target_lap_time >> 16U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) TCU_DRIVER_MSG_CANID;
  cframe->DLC = (uint8_t) TCU_DRIVER_MSG_DLC;
  cframe->IDE = (uint8_t) TCU_DRIVER_MSG_IDE;
  return TCU_DRIVER_MSG_CANID;
}

#else

uint32_t Pack_TCU_DRIVER_MSG_hytech(TCU_DRIVER_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_DRIVER_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->driver_message & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->driver_msg_var_1 & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->driver_msg_var_1 >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->driver_msg_var_2 & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->driver_msg_var_2 >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->target_lap_time & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->target_lap_time >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->target_lap_time >> 16U) & (0xFFU)) );

  *_len = (uint8_t) TCU_DRIVER_MSG_DLC;
  *_ide = (uint8_t) TCU_DRIVER_MSG_IDE;
  return TCU_DRIVER_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->best_lap_time = (uint32_t) ( ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->prev_lap_time = (uint32_t) ( ((_d[5] & (0xFFU)) << 16U) | ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->lap_clock_state = (uint8_t) ( (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TCU_LAP_TIMES_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TCU_LAP_TIMES_hytech(&_m->mon1, TCU_LAP_TIMES_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TCU_LAP_TIMES_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_LAP_TIMES_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->best_lap_time & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->best_lap_time >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->best_lap_time >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->prev_lap_time & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->prev_lap_time >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->prev_lap_time >> 16U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->lap_clock_state & (0xFFU)) );

  cframe->MsgId = (uint32_t) TCU_LAP_TIMES_CANID;
  cframe->DLC = (uint8_t) TCU_LAP_TIMES_DLC;
  cframe->IDE = (uint8_t) TCU_LAP_TIMES_IDE;
  return TCU_LAP_TIMES_CANID;
}

#else

uint32_t Pack_TCU_LAP_TIMES_hytech(TCU_LAP_TIMES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_LAP_TIMES_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->best_lap_time & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->best_lap_time >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->best_lap_time >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->prev_lap_time & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->prev_lap_time >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->prev_lap_time >> 16U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->lap_clock_state & (0xFFU)) );

  *_len = (uint8_t) TCU_LAP_TIMES_DLC;
  *_ide = (uint8_t) TCU_LAP_TIMES_IDE;
  return TCU_LAP_TIMES_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_setpoint_rpm = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->positive_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_phys = (sigfloat_t)(HYTECH_positive_torque_limit_ro_fromS(_m->positive_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->negative_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->negative_torque_limit_phys = (sigfloat_t)(HYTECH_negative_torque_limit_ro_fromS(_m->negative_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_CONTROL_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_CONTROL_INPUT_hytech(&_m->mon1, INV1_CONTROL_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_CONTROL_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_CONTROL_INPUT_CANID;
  cframe->DLC = (uint8_t) INV1_CONTROL_INPUT_DLC;
  cframe->IDE = (uint8_t) INV1_CONTROL_INPUT_IDE;
  return INV1_CONTROL_INPUT_CANID;
}

#else

uint32_t Pack_INV1_CONTROL_INPUT_hytech(INV1_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_CONTROL_INPUT_DLC;
  *_ide = (uint8_t) INV1_CONTROL_INPUT_IDE;
  return INV1_CONTROL_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_setpoint_rpm = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->positive_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_phys = (sigfloat_t)(HYTECH_positive_torque_limit_ro_fromS(_m->positive_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->negative_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->negative_torque_limit_phys = (sigfloat_t)(HYTECH_negative_torque_limit_ro_fromS(_m->negative_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_CONTROL_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_CONTROL_INPUT_hytech(&_m->mon1, INV2_CONTROL_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_CONTROL_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_CONTROL_INPUT_CANID;
  cframe->DLC = (uint8_t) INV2_CONTROL_INPUT_DLC;
  cframe->IDE = (uint8_t) INV2_CONTROL_INPUT_IDE;
  return INV2_CONTROL_INPUT_CANID;
}

#else

uint32_t Pack_INV2_CONTROL_INPUT_hytech(INV2_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_CONTROL_INPUT_DLC;
  *_ide = (uint8_t) INV2_CONTROL_INPUT_IDE;
  return INV2_CONTROL_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_setpoint_rpm = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->positive_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_phys = (sigfloat_t)(HYTECH_positive_torque_limit_ro_fromS(_m->positive_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->negative_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->negative_torque_limit_phys = (sigfloat_t)(HYTECH_negative_torque_limit_ro_fromS(_m->negative_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_CONTROL_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_CONTROL_INPUT_hytech(&_m->mon1, INV3_CONTROL_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_CONTROL_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_CONTROL_INPUT_CANID;
  cframe->DLC = (uint8_t) INV3_CONTROL_INPUT_DLC;
  cframe->IDE = (uint8_t) INV3_CONTROL_INPUT_IDE;
  return INV3_CONTROL_INPUT_CANID;
}

#else

uint32_t Pack_INV3_CONTROL_INPUT_hytech(INV3_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_CONTROL_INPUT_DLC;
  *_ide = (uint8_t) INV3_CONTROL_INPUT_IDE;
  return INV3_CONTROL_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->dc_bus_voltage = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->motor_power = (uint32_t) ( ((_d[5] & (0xFFU)) << 24U) | ((_d[4] & (0xFFU)) << 16U) | ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->feedback_torque = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MC1_ENERGY_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MC1_ENERGY_hytech(&_m->mon1, MC1_ENERGY_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MC1_ENERGY_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MC1_ENERGY_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motor_power & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->motor_power >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->motor_power >> 16U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->motor_power >> 24U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->feedback_torque & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->feedback_torque >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MC1_ENERGY_CANID;
  cframe->DLC = (uint8_t) MC1_ENERGY_DLC;
  cframe->IDE = (uint8_t) MC1_ENERGY_IDE;
  return MC1_ENERGY_CANID;
}

#else

uint32_t Pack_MC1_ENERGY_hytech(MC1_ENERGY_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MC1_ENERGY_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->dc_bus_voltage & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->dc_bus_voltage >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motor_power & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->motor_power >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->motor_power >> 16U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->motor_power >> 24U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->feedback_torque & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->feedback_torque >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MC1_ENERGY_DLC;
  *_ide = (uint8_t) MC1_ENERGY_IDE;
  return MC1_ENERGY_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->accel_implausible = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->brake_implausible = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->brake_pedal_active = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->accel_pedal_active = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->mechanical_brake_active = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->brake_accel_implausibility = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->implaus_exceeded_max_duration = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->accel_pedal_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->accel_pedal_phys = (sigfloat_t)(HYTECH_accel_pedal_ro_fromS(_m->accel_pedal_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_pedal_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_pedal_phys = (sigfloat_t)(HYTECH_brake_pedal_ro_fromS(_m->brake_pedal_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < PEDALS_SYSTEM_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_PEDALS_SYSTEM_DATA_hytech(&_m->mon1, PEDALS_SYSTEM_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return PEDALS_SYSTEM_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(PEDALS_SYSTEM_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->accel_pedal_ro = (uint16_t) HYTECH_accel_pedal_ro_toS(_m->accel_pedal_phys);
  _m->brake_pedal_ro = (uint16_t) HYTECH_brake_pedal_ro_toS(_m->brake_pedal_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->accel_implausible & (0x01U)) | ((_m->brake_implausible & (0x01U)) << 1U) | ((_m->brake_pedal_active & (0x01U)) << 2U) | ((_m->accel_pedal_active & (0x01U)) << 3U) | ((_m->mechanical_brake_active & (0x01U)) << 4U) | ((_m->brake_accel_implausibility & (0x01U)) << 5U) | ((_m->implaus_exceeded_max_duration & (0x01U)) << 6U) );
  cframe->Data[1] |= (uint8_t) ( (_m->accel_pedal_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->accel_pedal_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_pedal_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_pedal_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) PEDALS_SYSTEM_DATA_CANID;
  cframe->DLC = (uint8_t) PEDALS_SYSTEM_DATA_DLC;
  cframe->IDE = (uint8_t) PEDALS_SYSTEM_DATA_IDE;
  return PEDALS_SYSTEM_DATA_CANID;
}

#else

uint32_t Pack_PEDALS_SYSTEM_DATA_hytech(PEDALS_SYSTEM_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(PEDALS_SYSTEM_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->accel_pedal_ro = (uint16_t) HYTECH_accel_pedal_ro_toS(_m->accel_pedal_phys);
  _m->brake_pedal_ro = (uint16_t) HYTECH_brake_pedal_ro_toS(_m->brake_pedal_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->accel_implausible & (0x01U)) | ((_m->brake_implausible & (0x01U)) << 1U) | ((_m->brake_pedal_active & (0x01U)) << 2U) | ((_m->accel_pedal_active & (0x01U)) << 3U) | ((_m->mechanical_brake_active & (0x01U)) << 4U) | ((_m->brake_accel_implausibility & (0x01U)) << 5U) | ((_m->implaus_exceeded_max_duration & (0x01U)) << 6U) );
  _d[1] |= (uint8_t) ( (_m->accel_pedal_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->accel_pedal_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_pedal_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_pedal_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) PEDALS_SYSTEM_DATA_DLC;
  *_ide = (uint8_t) PEDALS_SYSTEM_DATA_IDE;
  return PEDALS_SYSTEM_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_STATUS_hytech(MCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->imd_ok_high = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->shutdown_b_above_threshold = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->bms_ok_high = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->shutdown_c_above_threshold = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->bspd_ok_high = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->shutdown_d_above_threshold = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->software_ok_high = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->shutdown_e_above_threshold = (uint8_t) ( ((_d[0] >> 7U) & (0x01U)) );
  _m->mechanical_brake_active = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->no_accel_implausibility = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->no_brake_implausibility = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->brake_pedal_active = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->bspd_current_high = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->bspd_brake_high = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->no_accel_or_brake_implausibility = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->drive_buzzer = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->ecu_state = (uint8_t) ( (_d[2] & (0x07U)) );
  _m->inverter_error = (uint8_t) ( ((_d[2] >> 3U) & (0x01U)) );
  _m->energy_meter_present = (uint8_t) ( ((_d[2] >> 4U) & (0x01U)) );
  _m->software_ok = (uint8_t) ( ((_d[2] >> 6U) & (0x01U)) );
  _m->launch_control_active = (uint8_t) ( ((_d[2] >> 7U) & (0x01U)) );
  _m->pack_charge_critical = (uint8_t) ( (_d[3] & (0x03U)) );
  _m->drive_mode = (uint8_t) ( ((_d[3] >> 4U) & (0x0FU)) );
  _m->max_torque = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->torque_mode = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->distance_traveled_m_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->distance_traveled_m_phys = (sigfloat_t)(HYTECH_distance_traveled_m_ro_fromS(_m->distance_traveled_m_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_STATUS_hytech(&_m->mon1, MCU_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_STATUS_hytech(MCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->distance_traveled_m_ro = (uint16_t) HYTECH_distance_traveled_m_ro_toS(_m->distance_traveled_m_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->imd_ok_high & (0x01U)) | ((_m->shutdown_b_above_threshold & (0x01U)) << 1U) | ((_m->bms_ok_high & (0x01U)) << 2U) | ((_m->shutdown_c_above_threshold & (0x01U)) << 3U) | ((_m->bspd_ok_high & (0x01U)) << 4U) | ((_m->shutdown_d_above_threshold & (0x01U)) << 5U) | ((_m->software_ok_high & (0x01U)) << 6U) | ((_m->shutdown_e_above_threshold & (0x01U)) << 7U) );
  cframe->Data[1] |= (uint8_t) ( (_m->mechanical_brake_active & (0x01U)) | ((_m->no_accel_implausibility & (0x01U)) << 1U) | ((_m->no_brake_implausibility & (0x01U)) << 2U) | ((_m->brake_pedal_active & (0x01U)) << 3U) | ((_m->bspd_current_high & (0x01U)) << 4U) | ((_m->bspd_brake_high & (0x01U)) << 5U) | ((_m->no_accel_or_brake_implausibility & (0x01U)) << 6U) | ((_m->drive_buzzer & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->ecu_state & (0x07U)) | ((_m->inverter_error & (0x01U)) << 3U) | ((_m->energy_meter_present & (0x01U)) << 4U) | ((_m->software_ok & (0x01U)) << 6U) | ((_m->launch_control_active & (0x01U)) << 7U) );
  cframe->Data[3] |= (uint8_t) ( (_m->pack_charge_critical & (0x03U)) | ((_m->drive_mode & (0x0FU)) << 4U) );
  cframe->Data[4] |= (uint8_t) ( (_m->max_torque & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->torque_mode & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->distance_traveled_m_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->distance_traveled_m_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_STATUS_CANID;
  cframe->DLC = (uint8_t) MCU_STATUS_DLC;
  cframe->IDE = (uint8_t) MCU_STATUS_IDE;
  return MCU_STATUS_CANID;
}

#else

uint32_t Pack_MCU_STATUS_hytech(MCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->distance_traveled_m_ro = (uint16_t) HYTECH_distance_traveled_m_ro_toS(_m->distance_traveled_m_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->imd_ok_high & (0x01U)) | ((_m->shutdown_b_above_threshold & (0x01U)) << 1U) | ((_m->bms_ok_high & (0x01U)) << 2U) | ((_m->shutdown_c_above_threshold & (0x01U)) << 3U) | ((_m->bspd_ok_high & (0x01U)) << 4U) | ((_m->shutdown_d_above_threshold & (0x01U)) << 5U) | ((_m->software_ok_high & (0x01U)) << 6U) | ((_m->shutdown_e_above_threshold & (0x01U)) << 7U) );
  _d[1] |= (uint8_t) ( (_m->mechanical_brake_active & (0x01U)) | ((_m->no_accel_implausibility & (0x01U)) << 1U) | ((_m->no_brake_implausibility & (0x01U)) << 2U) | ((_m->brake_pedal_active & (0x01U)) << 3U) | ((_m->bspd_current_high & (0x01U)) << 4U) | ((_m->bspd_brake_high & (0x01U)) << 5U) | ((_m->no_accel_or_brake_implausibility & (0x01U)) << 6U) | ((_m->drive_buzzer & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->ecu_state & (0x07U)) | ((_m->inverter_error & (0x01U)) << 3U) | ((_m->energy_meter_present & (0x01U)) << 4U) | ((_m->software_ok & (0x01U)) << 6U) | ((_m->launch_control_active & (0x01U)) << 7U) );
  _d[3] |= (uint8_t) ( (_m->pack_charge_critical & (0x03U)) | ((_m->drive_mode & (0x0FU)) << 4U) );
  _d[4] |= (uint8_t) ( (_m->max_torque & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->torque_mode & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->distance_traveled_m_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->distance_traveled_m_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MCU_STATUS_DLC;
  *_ide = (uint8_t) MCU_STATUS_IDE;
  return MCU_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->load_cell_fl = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->load_cell_fr = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_LOAD_CELLS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_LOAD_CELLS_hytech(&_m->mon1, MCU_LOAD_CELLS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_LOAD_CELLS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_LOAD_CELLS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->load_cell_fl & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->load_cell_fl >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->load_cell_fr & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->load_cell_fr >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_LOAD_CELLS_CANID;
  cframe->DLC = (uint8_t) MCU_LOAD_CELLS_DLC;
  cframe->IDE = (uint8_t) MCU_LOAD_CELLS_IDE;
  return MCU_LOAD_CELLS_CANID;
}

#else

uint32_t Pack_MCU_LOAD_CELLS_hytech(MCU_LOAD_CELLS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_LOAD_CELLS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->load_cell_fl & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->load_cell_fl >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->load_cell_fr & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->load_cell_fr >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MCU_LOAD_CELLS_DLC;
  *_ide = (uint8_t) MCU_LOAD_CELLS_IDE;
  return MCU_LOAD_CELLS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->potentiometer_fl = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->potentiometer_fr = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_FRONT_POTS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_FRONT_POTS_hytech(&_m->mon1, MCU_FRONT_POTS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_FRONT_POTS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_FRONT_POTS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->potentiometer_fl & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->potentiometer_fl >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->potentiometer_fr & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->potentiometer_fr >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_FRONT_POTS_CANID;
  cframe->DLC = (uint8_t) MCU_FRONT_POTS_DLC;
  cframe->IDE = (uint8_t) MCU_FRONT_POTS_IDE;
  return MCU_FRONT_POTS_CANID;
}

#else

uint32_t Pack_MCU_FRONT_POTS_hytech(MCU_FRONT_POTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_FRONT_POTS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->potentiometer_fl & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->potentiometer_fl >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->potentiometer_fr & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->potentiometer_fr >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MCU_FRONT_POTS_DLC;
  *_ide = (uint8_t) MCU_FRONT_POTS_IDE;
  return MCU_FRONT_POTS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->potentiometer_fl = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->potentiometer_fr = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->load_cell_fl = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->load_cell_fr = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_SUSPENSION_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_SUSPENSION_hytech(&_m->mon1, MCU_SUSPENSION_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_SUSPENSION_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_SUSPENSION_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->potentiometer_fl & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->potentiometer_fl >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->potentiometer_fr & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->potentiometer_fr >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->load_cell_fl & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->load_cell_fl >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->load_cell_fr & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->load_cell_fr >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_SUSPENSION_CANID;
  cframe->DLC = (uint8_t) MCU_SUSPENSION_DLC;
  cframe->IDE = (uint8_t) MCU_SUSPENSION_IDE;
  return MCU_SUSPENSION_CANID;
}

#else

uint32_t Pack_MCU_SUSPENSION_hytech(MCU_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_SUSPENSION_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->potentiometer_fl & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->potentiometer_fl >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->potentiometer_fr & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->potentiometer_fr >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->load_cell_fl & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->load_cell_fl >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->load_cell_fr & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->load_cell_fr >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MCU_SUSPENSION_DLC;
  *_ide = (uint8_t) MCU_SUSPENSION_IDE;
  return MCU_SUSPENSION_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->torque_request_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_request_phys = (sigfloat_t)(HYTECH_torque_request_ro_fromS(_m->torque_request_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->front_torque_scale_ro = (int8_t) __ext_sig__(( (_d[2] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->front_torque_scale_phys = (sigfloat_t)(HYTECH_front_torque_scale_ro_fromS(_m->front_torque_scale_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->rear_torque_scale_ro = (int8_t) __ext_sig__(( (_d[3] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->rear_torque_scale_phys = (sigfloat_t)(HYTECH_rear_torque_scale_ro_fromS(_m->rear_torque_scale_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->front_regen_scale_ro = (int8_t) __ext_sig__(( (_d[4] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->front_regen_scale_phys = (sigfloat_t)(HYTECH_front_regen_scale_ro_fromS(_m->front_regen_scale_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->rear_regen_scale_ro = (int8_t) __ext_sig__(( (_d[5] & (0xFFU)) ), 8);
#ifdef HYTECH_USE_SIGFLOAT
  _m->rear_regen_scale_phys = (sigfloat_t)(HYTECH_rear_regen_scale_ro_fromS(_m->rear_regen_scale_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->accel_request_state = (uint8_t) ( (_d[6] & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TC_SIMPLE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TC_SIMPLE_hytech(&_m->mon1, TC_SIMPLE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TC_SIMPLE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TC_SIMPLE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_request_ro = (int16_t) HYTECH_torque_request_ro_toS(_m->torque_request_phys);
  _m->front_torque_scale_ro = (int8_t) HYTECH_front_torque_scale_ro_toS(_m->front_torque_scale_phys);
  _m->rear_torque_scale_ro = (int8_t) HYTECH_rear_torque_scale_ro_toS(_m->rear_torque_scale_phys);
  _m->front_regen_scale_ro = (int8_t) HYTECH_front_regen_scale_ro_toS(_m->front_regen_scale_phys);
  _m->rear_regen_scale_ro = (int8_t) HYTECH_rear_regen_scale_ro_toS(_m->rear_regen_scale_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->torque_request_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->torque_request_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->front_torque_scale_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->rear_torque_scale_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->front_regen_scale_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->rear_regen_scale_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->accel_request_state & (0x01U)) );

  cframe->MsgId = (uint32_t) TC_SIMPLE_CANID;
  cframe->DLC = (uint8_t) TC_SIMPLE_DLC;
  cframe->IDE = (uint8_t) TC_SIMPLE_IDE;
  return TC_SIMPLE_CANID;
}

#else

uint32_t Pack_TC_SIMPLE_hytech(TC_SIMPLE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TC_SIMPLE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_request_ro = (int16_t) HYTECH_torque_request_ro_toS(_m->torque_request_phys);
  _m->front_torque_scale_ro = (int8_t) HYTECH_front_torque_scale_ro_toS(_m->front_torque_scale_phys);
  _m->rear_torque_scale_ro = (int8_t) HYTECH_rear_torque_scale_ro_toS(_m->rear_torque_scale_phys);
  _m->front_regen_scale_ro = (int8_t) HYTECH_front_regen_scale_ro_toS(_m->front_regen_scale_phys);
  _m->rear_regen_scale_ro = (int8_t) HYTECH_rear_regen_scale_ro_toS(_m->rear_regen_scale_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->torque_request_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->torque_request_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->front_torque_scale_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->rear_torque_scale_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->front_regen_scale_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->rear_regen_scale_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->accel_request_state & (0x01U)) );

  *_len = (uint8_t) TC_SIMPLE_DLC;
  *_ide = (uint8_t) TC_SIMPLE_IDE;
  return TC_SIMPLE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->launch_control_state = (uint8_t) ( (_d[0] & (0x03U)) );
  _m->algo_active = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->initial_launch_target = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->speed_setpoint_rpm = (int16_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TC_SIMPLE_LAUNCH_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TC_SIMPLE_LAUNCH_hytech(&_m->mon1, TC_SIMPLE_LAUNCH_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TC_SIMPLE_LAUNCH_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TC_SIMPLE_LAUNCH_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->launch_control_state & (0x03U)) | ((_m->algo_active & (0x01U)) << 2U) );
  cframe->Data[1] |= (uint8_t) ( (_m->initial_launch_target & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->initial_launch_target >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) TC_SIMPLE_LAUNCH_CANID;
  cframe->DLC = (uint8_t) TC_SIMPLE_LAUNCH_DLC;
  cframe->IDE = (uint8_t) TC_SIMPLE_LAUNCH_IDE;
  return TC_SIMPLE_LAUNCH_CANID;
}

#else

uint32_t Pack_TC_SIMPLE_LAUNCH_hytech(TC_SIMPLE_LAUNCH_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TC_SIMPLE_LAUNCH_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->launch_control_state & (0x03U)) | ((_m->algo_active & (0x01U)) << 2U) );
  _d[1] |= (uint8_t) ( (_m->initial_launch_target & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->initial_launch_target >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) TC_SIMPLE_LAUNCH_DLC;
  *_ide = (uint8_t) TC_SIMPLE_LAUNCH_IDE;
  return TC_SIMPLE_LAUNCH_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_sensor_analog_read_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_sensor_analog_read_phys = (sigfloat_t)(HYTECH_brake_sensor_analog_read_ro_fromS(_m->brake_sensor_analog_read_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_PRESSURE_SENSOR_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_PRESSURE_SENSOR_hytech(&_m->mon1, BRAKE_PRESSURE_SENSOR_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BRAKE_PRESSURE_SENSOR_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BRAKE_PRESSURE_SENSOR_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_sensor_analog_read_ro = (uint16_t) HYTECH_brake_sensor_analog_read_ro_toS(_m->brake_sensor_analog_read_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->brake_sensor_analog_read_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->brake_sensor_analog_read_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BRAKE_PRESSURE_SENSOR_CANID;
  cframe->DLC = (uint8_t) BRAKE_PRESSURE_SENSOR_DLC;
  cframe->IDE = (uint8_t) BRAKE_PRESSURE_SENSOR_IDE;
  return BRAKE_PRESSURE_SENSOR_CANID;
}

#else

uint32_t Pack_BRAKE_PRESSURE_SENSOR_hytech(BRAKE_PRESSURE_SENSOR_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BRAKE_PRESSURE_SENSOR_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_sensor_analog_read_ro = (uint16_t) HYTECH_brake_sensor_analog_read_ro_toS(_m->brake_sensor_analog_read_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->brake_sensor_analog_read_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->brake_sensor_analog_read_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BRAKE_PRESSURE_SENSOR_DLC;
  *_ide = (uint8_t) BRAKE_PRESSURE_SENSOR_IDE;
  return BRAKE_PRESSURE_SENSOR_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->steering_1 = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->steering_2 = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->hall_effect_current = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
  _m->glv_battery_voltage = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_ANALOG_READINGS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_ANALOG_READINGS_hytech(&_m->mon1, MCU_ANALOG_READINGS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_ANALOG_READINGS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_ANALOG_READINGS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->steering_1 & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->steering_1 >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->steering_2 & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->steering_2 >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->hall_effect_current & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->hall_effect_current >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->glv_battery_voltage & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->glv_battery_voltage >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_ANALOG_READINGS_CANID;
  cframe->DLC = (uint8_t) MCU_ANALOG_READINGS_DLC;
  cframe->IDE = (uint8_t) MCU_ANALOG_READINGS_IDE;
  return MCU_ANALOG_READINGS_CANID;
}

#else

uint32_t Pack_MCU_ANALOG_READINGS_hytech(MCU_ANALOG_READINGS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_ANALOG_READINGS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->steering_1 & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->steering_1 >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->steering_2 & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->steering_2 >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->hall_effect_current & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->hall_effect_current >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->glv_battery_voltage & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->glv_battery_voltage >> 8U) & (0xFFU)) );

  *_len = (uint8_t) MCU_ANALOG_READINGS_DLC;
  *_ide = (uint8_t) MCU_ANALOG_READINGS_IDE;
  return MCU_ANALOG_READINGS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->accel_1_raw = (uint16_t) ( ((_d[1] & (0x0FU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->accel_2_raw = (uint16_t) ( ((_d[2] & (0xFFU)) << 4U) | ((_d[1] >> 4U) & (0x0FU)) );
  _m->brake_1_raw = (uint16_t) ( ((_d[4] & (0x0FU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->brake_2_raw = (uint16_t) ( ((_d[5] & (0xFFU)) << 4U) | ((_d[4] >> 4U) & (0x0FU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_PEDAL_RAW_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_PEDAL_RAW_hytech(&_m->mon1, MCU_PEDAL_RAW_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_PEDAL_RAW_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_PEDAL_RAW_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->accel_1_raw & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->accel_1_raw >> 8U) & (0x0FU)) | ((_m->accel_2_raw & (0x0FU)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->accel_2_raw >> 4U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_1_raw & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_1_raw >> 8U) & (0x0FU)) | ((_m->brake_2_raw & (0x0FU)) << 4U) );
  cframe->Data[5] |= (uint8_t) ( ((_m->brake_2_raw >> 4U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) MCU_PEDAL_RAW_CANID;
  cframe->DLC = (uint8_t) MCU_PEDAL_RAW_DLC;
  cframe->IDE = (uint8_t) MCU_PEDAL_RAW_IDE;
  return MCU_PEDAL_RAW_CANID;
}

#else

uint32_t Pack_MCU_PEDAL_RAW_hytech(MCU_PEDAL_RAW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_PEDAL_RAW_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->accel_1_raw & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->accel_1_raw >> 8U) & (0x0FU)) | ((_m->accel_2_raw & (0x0FU)) << 4U) );
  _d[2] |= (uint8_t) ( ((_m->accel_2_raw >> 4U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_1_raw & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_1_raw >> 8U) & (0x0FU)) | ((_m->brake_2_raw & (0x0FU)) << 4U) );
  _d[5] |= (uint8_t) ( ((_m->brake_2_raw >> 4U) & (0xFFU)) );

  *_len = (uint8_t) MCU_PEDAL_RAW_DLC;
  *_ide = (uint8_t) MCU_PEDAL_RAW_IDE;
  return MCU_PEDAL_RAW_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_VEL_hytech(VN_VEL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_body_vel_x_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_body_vel_x_phys = (sigfloat_t)(HYTECH_vn_body_vel_x_ro_fromS(_m->vn_body_vel_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_body_vel_y_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_body_vel_y_phys = (sigfloat_t)(HYTECH_vn_body_vel_y_ro_fromS(_m->vn_body_vel_y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_body_vel_z_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_body_vel_z_phys = (sigfloat_t)(HYTECH_vn_body_vel_z_ro_fromS(_m->vn_body_vel_z_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_vel_uncertainty_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_vel_uncertainty_phys = (sigfloat_t)(HYTECH_vn_vel_uncertainty_ro_fromS(_m->vn_vel_uncertainty_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_VEL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_VEL_hytech(&_m->mon1, VN_VEL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_VEL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_VEL_hytech(VN_VEL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_VEL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_body_vel_x_ro = (int16_t) HYTECH_vn_body_vel_x_ro_toS(_m->vn_body_vel_x_phys);
  _m->vn_body_vel_y_ro = (int16_t) HYTECH_vn_body_vel_y_ro_toS(_m->vn_body_vel_y_phys);
  _m->vn_body_vel_z_ro = (int16_t) HYTECH_vn_body_vel_z_ro_toS(_m->vn_body_vel_z_phys);
  _m->vn_vel_uncertainty_ro = (uint16_t) HYTECH_vn_vel_uncertainty_ro_toS(_m->vn_vel_uncertainty_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_body_vel_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_body_vel_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vn_body_vel_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_body_vel_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_body_vel_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_body_vel_z_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->vn_vel_uncertainty_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vn_vel_uncertainty_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_VEL_CANID;
  cframe->DLC = (uint8_t) VN_VEL_DLC;
  cframe->IDE = (uint8_t) VN_VEL_IDE;
  return VN_VEL_CANID;
}

#else

uint32_t Pack_VN_VEL_hytech(VN_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_VEL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_body_vel_x_ro = (int16_t) HYTECH_vn_body_vel_x_ro_toS(_m->vn_body_vel_x_phys);
  _m->vn_body_vel_y_ro = (int16_t) HYTECH_vn_body_vel_y_ro_toS(_m->vn_body_vel_y_phys);
  _m->vn_body_vel_z_ro = (int16_t) HYTECH_vn_body_vel_z_ro_toS(_m->vn_body_vel_z_phys);
  _m->vn_vel_uncertainty_ro = (uint16_t) HYTECH_vn_vel_uncertainty_ro_toS(_m->vn_vel_uncertainty_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_body_vel_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_body_vel_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vn_body_vel_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_body_vel_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_body_vel_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_body_vel_z_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->vn_vel_uncertainty_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vn_vel_uncertainty_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VN_VEL_DLC;
  *_ide = (uint8_t) VN_VEL_IDE;
  return VN_VEL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_lin_ins_accel_x_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_ins_accel_x_phys = (sigfloat_t)(HYTECH_vn_lin_ins_accel_x_ro_fromS(_m->vn_lin_ins_accel_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_lin_ins_accel_y_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_ins_accel_y_phys = (sigfloat_t)(HYTECH_vn_lin_ins_accel_y_ro_fromS(_m->vn_lin_ins_accel_y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_lin_ins_accel_z_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_ins_accel_z_phys = (sigfloat_t)(HYTECH_vn_lin_ins_accel_z_ro_fromS(_m->vn_lin_ins_accel_z_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_LINEAR_ACCEL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_LINEAR_ACCEL_hytech(&_m->mon1, VN_LINEAR_ACCEL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_LINEAR_ACCEL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LINEAR_ACCEL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_ins_accel_x_ro = (int16_t) HYTECH_vn_lin_ins_accel_x_ro_toS(_m->vn_lin_ins_accel_x_phys);
  _m->vn_lin_ins_accel_y_ro = (int16_t) HYTECH_vn_lin_ins_accel_y_ro_toS(_m->vn_lin_ins_accel_y_phys);
  _m->vn_lin_ins_accel_z_ro = (int16_t) HYTECH_vn_lin_ins_accel_z_ro_toS(_m->vn_lin_ins_accel_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_lin_ins_accel_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_lin_ins_accel_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vn_lin_ins_accel_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_lin_ins_accel_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_lin_ins_accel_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_lin_ins_accel_z_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_LINEAR_ACCEL_CANID;
  cframe->DLC = (uint8_t) VN_LINEAR_ACCEL_DLC;
  cframe->IDE = (uint8_t) VN_LINEAR_ACCEL_IDE;
  return VN_LINEAR_ACCEL_CANID;
}

#else

uint32_t Pack_VN_LINEAR_ACCEL_hytech(VN_LINEAR_ACCEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LINEAR_ACCEL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_ins_accel_x_ro = (int16_t) HYTECH_vn_lin_ins_accel_x_ro_toS(_m->vn_lin_ins_accel_x_phys);
  _m->vn_lin_ins_accel_y_ro = (int16_t) HYTECH_vn_lin_ins_accel_y_ro_toS(_m->vn_lin_ins_accel_y_phys);
  _m->vn_lin_ins_accel_z_ro = (int16_t) HYTECH_vn_lin_ins_accel_z_ro_toS(_m->vn_lin_ins_accel_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_lin_ins_accel_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_lin_ins_accel_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vn_lin_ins_accel_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_lin_ins_accel_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_lin_ins_accel_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_lin_ins_accel_z_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VN_LINEAR_ACCEL_DLC;
  *_ide = (uint8_t) VN_LINEAR_ACCEL_IDE;
  return VN_LINEAR_ACCEL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_lin_uncomp_accel_x_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_uncomp_accel_x_phys = (sigfloat_t)(HYTECH_vn_lin_uncomp_accel_x_ro_fromS(_m->vn_lin_uncomp_accel_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_lin_uncomp_accel_y_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_uncomp_accel_y_phys = (sigfloat_t)(HYTECH_vn_lin_uncomp_accel_y_ro_fromS(_m->vn_lin_uncomp_accel_y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_lin_uncomp_accel_z_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_uncomp_accel_z_phys = (sigfloat_t)(HYTECH_vn_lin_uncomp_accel_z_ro_fromS(_m->vn_lin_uncomp_accel_z_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_LINEAR_ACCEL_UNCOMP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_LINEAR_ACCEL_UNCOMP_hytech(&_m->mon1, VN_LINEAR_ACCEL_UNCOMP_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_LINEAR_ACCEL_UNCOMP_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LINEAR_ACCEL_UNCOMP_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_uncomp_accel_x_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_x_ro_toS(_m->vn_lin_uncomp_accel_x_phys);
  _m->vn_lin_uncomp_accel_y_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_y_ro_toS(_m->vn_lin_uncomp_accel_y_phys);
  _m->vn_lin_uncomp_accel_z_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_z_ro_toS(_m->vn_lin_uncomp_accel_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_z_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_LINEAR_ACCEL_UNCOMP_CANID;
  cframe->DLC = (uint8_t) VN_LINEAR_ACCEL_UNCOMP_DLC;
  cframe->IDE = (uint8_t) VN_LINEAR_ACCEL_UNCOMP_IDE;
  return VN_LINEAR_ACCEL_UNCOMP_CANID;
}

#else

uint32_t Pack_VN_LINEAR_ACCEL_UNCOMP_hytech(VN_LINEAR_ACCEL_UNCOMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LINEAR_ACCEL_UNCOMP_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_lin_uncomp_accel_x_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_x_ro_toS(_m->vn_lin_uncomp_accel_x_phys);
  _m->vn_lin_uncomp_accel_y_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_y_ro_toS(_m->vn_lin_uncomp_accel_y_phys);
  _m->vn_lin_uncomp_accel_z_ro = (int16_t) HYTECH_vn_lin_uncomp_accel_z_ro_toS(_m->vn_lin_uncomp_accel_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_lin_uncomp_accel_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_lin_uncomp_accel_z_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VN_LINEAR_ACCEL_UNCOMP_DLC;
  *_ide = (uint8_t) VN_LINEAR_ACCEL_UNCOMP_IDE;
  return VN_LINEAR_ACCEL_UNCOMP_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_YPR_hytech(VN_YPR_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_yaw_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_yaw_phys = (sigfloat_t)(HYTECH_vn_yaw_ro_fromS(_m->vn_yaw_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_pitch_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_pitch_phys = (sigfloat_t)(HYTECH_vn_pitch_ro_fromS(_m->vn_pitch_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_roll_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_roll_phys = (sigfloat_t)(HYTECH_vn_roll_ro_fromS(_m->vn_roll_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_YPR_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_YPR_hytech(&_m->mon1, VN_YPR_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_YPR_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_YPR_hytech(VN_YPR_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_YPR_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_yaw_ro = (int16_t) HYTECH_vn_yaw_ro_toS(_m->vn_yaw_phys);
  _m->vn_pitch_ro = (int16_t) HYTECH_vn_pitch_ro_toS(_m->vn_pitch_phys);
  _m->vn_roll_ro = (int16_t) HYTECH_vn_roll_ro_toS(_m->vn_roll_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_yaw_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_yaw_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vn_pitch_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_pitch_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_roll_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_roll_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_YPR_CANID;
  cframe->DLC = (uint8_t) VN_YPR_DLC;
  cframe->IDE = (uint8_t) VN_YPR_IDE;
  return VN_YPR_CANID;
}

#else

uint32_t Pack_VN_YPR_hytech(VN_YPR_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_YPR_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_yaw_ro = (int16_t) HYTECH_vn_yaw_ro_toS(_m->vn_yaw_phys);
  _m->vn_pitch_ro = (int16_t) HYTECH_vn_pitch_ro_toS(_m->vn_pitch_phys);
  _m->vn_roll_ro = (int16_t) HYTECH_vn_roll_ro_toS(_m->vn_roll_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_yaw_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_yaw_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vn_pitch_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_pitch_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_roll_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_roll_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VN_YPR_DLC;
  *_ide = (uint8_t) VN_YPR_IDE;
  return VN_YPR_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_gps_lat_ro = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_gps_lat_phys = (sigfloat_t)(HYTECH_vn_gps_lat_ro_fromS(_m->vn_gps_lat_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_gps_lon_ro = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_gps_lon_phys = (sigfloat_t)(HYTECH_vn_gps_lon_ro_fromS(_m->vn_gps_lon_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_LAT_LON_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_LAT_LON_hytech(&_m->mon1, VN_LAT_LON_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_LAT_LON_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LAT_LON_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_gps_lat_ro = (int32_t) HYTECH_vn_gps_lat_ro_toS(_m->vn_gps_lat_phys);
  _m->vn_gps_lon_ro = (int32_t) HYTECH_vn_gps_lon_ro_toS(_m->vn_gps_lon_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_gps_lat_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_gps_lon_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_LAT_LON_CANID;
  cframe->DLC = (uint8_t) VN_LAT_LON_DLC;
  cframe->IDE = (uint8_t) VN_LAT_LON_IDE;
  return VN_LAT_LON_CANID;
}

#else

uint32_t Pack_VN_LAT_LON_hytech(VN_LAT_LON_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_LAT_LON_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_gps_lat_ro = (int32_t) HYTECH_vn_gps_lat_ro_toS(_m->vn_gps_lat_phys);
  _m->vn_gps_lon_ro = (int32_t) HYTECH_vn_gps_lon_ro_toS(_m->vn_gps_lon_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_gps_lat_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_gps_lat_ro >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_gps_lon_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vn_gps_lon_ro >> 24U) & (0xFFU)) );

  *_len = (uint8_t) VN_LAT_LON_DLC;
  *_ide = (uint8_t) VN_LAT_LON_IDE;
  return VN_LAT_LON_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->max_board_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_phys = (sigfloat_t)(HYTECH_max_board_temp_ro_fromS(_m->max_board_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->low_cell_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->low_cell_temp_phys = (sigfloat_t)(HYTECH_low_cell_temp_ro_fromS(_m->low_cell_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->high_cell_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->high_cell_temp_phys = (sigfloat_t)(HYTECH_high_cell_temp_ro_fromS(_m->high_cell_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_ONBOARD_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_ONBOARD_TEMPS_hytech(&_m->mon1, BMS_ONBOARD_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_ONBOARD_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_ONBOARD_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_ro = (int16_t) HYTECH_max_board_temp_ro_toS(_m->max_board_temp_phys);
  _m->low_cell_temp_ro = (int16_t) HYTECH_low_cell_temp_ro_toS(_m->low_cell_temp_phys);
  _m->high_cell_temp_ro = (int16_t) HYTECH_high_cell_temp_ro_toS(_m->high_cell_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->max_board_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->max_board_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->low_cell_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->low_cell_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->high_cell_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->high_cell_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_ONBOARD_TEMPS_CANID;
  cframe->DLC = (uint8_t) BMS_ONBOARD_TEMPS_DLC;
  cframe->IDE = (uint8_t) BMS_ONBOARD_TEMPS_IDE;
  return BMS_ONBOARD_TEMPS_CANID;
}

#else

uint32_t Pack_BMS_ONBOARD_TEMPS_hytech(BMS_ONBOARD_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_ONBOARD_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_ro = (int16_t) HYTECH_max_board_temp_ro_toS(_m->max_board_temp_phys);
  _m->low_cell_temp_ro = (int16_t) HYTECH_low_cell_temp_ro_toS(_m->low_cell_temp_phys);
  _m->high_cell_temp_ro = (int16_t) HYTECH_high_cell_temp_ro_toS(_m->high_cell_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->max_board_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->max_board_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->low_cell_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->low_cell_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->high_cell_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->high_cell_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_ONBOARD_TEMPS_DLC;
  *_ide = (uint8_t) BMS_ONBOARD_TEMPS_IDE;
  return BMS_ONBOARD_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->ic_id = (uint8_t) ( (_d[0] & (0x0FU)) );
  _m->temp_0_ro = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->temp_0_phys = (sigfloat_t)(HYTECH_temp_0_ro_fromS(_m->temp_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->temp_1_ro = (int16_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->temp_1_phys = (sigfloat_t)(HYTECH_temp_1_ro_fromS(_m->temp_1_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_ONBOARD_DETAILED_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_ONBOARD_DETAILED_TEMPS_hytech(&_m->mon1, BMS_ONBOARD_DETAILED_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_ONBOARD_DETAILED_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_ONBOARD_DETAILED_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->temp_0_ro = (int16_t) HYTECH_temp_0_ro_toS(_m->temp_0_phys);
  _m->temp_1_ro = (int16_t) HYTECH_temp_1_ro_toS(_m->temp_1_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->ic_id & (0x0FU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->temp_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->temp_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->temp_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->temp_1_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_ONBOARD_DETAILED_TEMPS_CANID;
  cframe->DLC = (uint8_t) BMS_ONBOARD_DETAILED_TEMPS_DLC;
  cframe->IDE = (uint8_t) BMS_ONBOARD_DETAILED_TEMPS_IDE;
  return BMS_ONBOARD_DETAILED_TEMPS_CANID;
}

#else

uint32_t Pack_BMS_ONBOARD_DETAILED_TEMPS_hytech(BMS_ONBOARD_DETAILED_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_ONBOARD_DETAILED_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->temp_0_ro = (int16_t) HYTECH_temp_0_ro_toS(_m->temp_0_phys);
  _m->temp_1_ro = (int16_t) HYTECH_temp_1_ro_toS(_m->temp_1_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->ic_id & (0x0FU)) );
  _d[1] |= (uint8_t) ( (_m->temp_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->temp_0_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->temp_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->temp_1_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_ONBOARD_DETAILED_TEMPS_DLC;
  *_ide = (uint8_t) BMS_ONBOARD_DETAILED_TEMPS_IDE;
  return BMS_ONBOARD_DETAILED_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->average_voltage_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->average_voltage_phys = (sigfloat_t)(HYTECH_average_voltage_ro_fromS(_m->average_voltage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->low_voltage_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->low_voltage_phys = (sigfloat_t)(HYTECH_low_voltage_ro_fromS(_m->low_voltage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->high_voltage_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->high_voltage_phys = (sigfloat_t)(HYTECH_high_voltage_ro_fromS(_m->high_voltage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->total_voltage_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->total_voltage_phys = (sigfloat_t)(HYTECH_total_voltage_ro_fromS(_m->total_voltage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_VOLTAGES_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_VOLTAGES_hytech(&_m->mon1, BMS_VOLTAGES_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_VOLTAGES_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_VOLTAGES_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->average_voltage_ro = (uint16_t) HYTECH_average_voltage_ro_toS(_m->average_voltage_phys);
  _m->low_voltage_ro = (uint16_t) HYTECH_low_voltage_ro_toS(_m->low_voltage_phys);
  _m->high_voltage_ro = (uint16_t) HYTECH_high_voltage_ro_toS(_m->high_voltage_phys);
  _m->total_voltage_ro = (uint16_t) HYTECH_total_voltage_ro_toS(_m->total_voltage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->average_voltage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->average_voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->low_voltage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->low_voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->high_voltage_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->high_voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->total_voltage_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->total_voltage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_VOLTAGES_CANID;
  cframe->DLC = (uint8_t) BMS_VOLTAGES_DLC;
  cframe->IDE = (uint8_t) BMS_VOLTAGES_IDE;
  return BMS_VOLTAGES_CANID;
}

#else

uint32_t Pack_BMS_VOLTAGES_hytech(BMS_VOLTAGES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_VOLTAGES_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->average_voltage_ro = (uint16_t) HYTECH_average_voltage_ro_toS(_m->average_voltage_phys);
  _m->low_voltage_ro = (uint16_t) HYTECH_low_voltage_ro_toS(_m->low_voltage_phys);
  _m->high_voltage_ro = (uint16_t) HYTECH_high_voltage_ro_toS(_m->high_voltage_phys);
  _m->total_voltage_ro = (uint16_t) HYTECH_total_voltage_ro_toS(_m->total_voltage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->average_voltage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->average_voltage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->low_voltage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->low_voltage_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->high_voltage_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->high_voltage_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->total_voltage_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->total_voltage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_VOLTAGES_DLC;
  *_ide = (uint8_t) BMS_VOLTAGES_IDE;
  return BMS_VOLTAGES_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->group_id = (uint8_t) ( (_d[0] & (0x0FU)) );
  _m->ic_id = (uint8_t) ( ((_d[0] >> 4U) & (0x0FU)) );
  _m->voltage_0_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->voltage_0_phys = (sigfloat_t)(HYTECH_voltage_0_ro_fromS(_m->voltage_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->voltage_1_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->voltage_1_phys = (sigfloat_t)(HYTECH_voltage_1_ro_fromS(_m->voltage_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->voltage_2_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->voltage_2_phys = (sigfloat_t)(HYTECH_voltage_2_ro_fromS(_m->voltage_2_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_DETAILED_VOLTAGES_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_DETAILED_VOLTAGES_hytech(&_m->mon1, BMS_DETAILED_VOLTAGES_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_DETAILED_VOLTAGES_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_DETAILED_VOLTAGES_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->voltage_0_ro = (uint16_t) HYTECH_voltage_0_ro_toS(_m->voltage_0_phys);
  _m->voltage_1_ro = (uint16_t) HYTECH_voltage_1_ro_toS(_m->voltage_1_phys);
  _m->voltage_2_ro = (uint16_t) HYTECH_voltage_2_ro_toS(_m->voltage_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->group_id & (0x0FU)) | ((_m->ic_id & (0x0FU)) << 4U) );
  cframe->Data[1] |= (uint8_t) ( (_m->voltage_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->voltage_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->voltage_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->voltage_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->voltage_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->voltage_2_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_DETAILED_VOLTAGES_CANID;
  cframe->DLC = (uint8_t) BMS_DETAILED_VOLTAGES_DLC;
  cframe->IDE = (uint8_t) BMS_DETAILED_VOLTAGES_IDE;
  return BMS_DETAILED_VOLTAGES_CANID;
}

#else

uint32_t Pack_BMS_DETAILED_VOLTAGES_hytech(BMS_DETAILED_VOLTAGES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_DETAILED_VOLTAGES_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->voltage_0_ro = (uint16_t) HYTECH_voltage_0_ro_toS(_m->voltage_0_phys);
  _m->voltage_1_ro = (uint16_t) HYTECH_voltage_1_ro_toS(_m->voltage_1_phys);
  _m->voltage_2_ro = (uint16_t) HYTECH_voltage_2_ro_toS(_m->voltage_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->group_id & (0x0FU)) | ((_m->ic_id & (0x0FU)) << 4U) );
  _d[1] |= (uint8_t) ( (_m->voltage_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->voltage_0_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->voltage_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->voltage_1_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->voltage_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->voltage_2_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_DETAILED_VOLTAGES_DLC;
  *_ide = (uint8_t) BMS_DETAILED_VOLTAGES_IDE;
  return BMS_DETAILED_VOLTAGES_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->max_board_temp_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_phys = (sigfloat_t)(HYTECH_max_board_temp_ro_fromS(_m->max_board_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->low_cell_temp_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->low_cell_temp_phys = (sigfloat_t)(HYTECH_low_cell_temp_ro_fromS(_m->low_cell_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->high_cell_temp_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->high_cell_temp_phys = (sigfloat_t)(HYTECH_high_cell_temp_ro_fromS(_m->high_cell_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_TEMPS_hytech(&_m->mon1, BMS_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_ro = (int16_t) HYTECH_max_board_temp_ro_toS(_m->max_board_temp_phys);
  _m->low_cell_temp_ro = (int16_t) HYTECH_low_cell_temp_ro_toS(_m->low_cell_temp_phys);
  _m->high_cell_temp_ro = (int16_t) HYTECH_high_cell_temp_ro_toS(_m->high_cell_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->max_board_temp_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->max_board_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->low_cell_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->low_cell_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->high_cell_temp_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->high_cell_temp_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_TEMPS_CANID;
  cframe->DLC = (uint8_t) BMS_TEMPS_DLC;
  cframe->IDE = (uint8_t) BMS_TEMPS_IDE;
  return BMS_TEMPS_CANID;
}

#else

uint32_t Pack_BMS_TEMPS_hytech(BMS_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->max_board_temp_ro = (int16_t) HYTECH_max_board_temp_ro_toS(_m->max_board_temp_phys);
  _m->low_cell_temp_ro = (int16_t) HYTECH_low_cell_temp_ro_toS(_m->low_cell_temp_phys);
  _m->high_cell_temp_ro = (int16_t) HYTECH_high_cell_temp_ro_toS(_m->high_cell_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->max_board_temp_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->max_board_temp_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->low_cell_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->low_cell_temp_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->high_cell_temp_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->high_cell_temp_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_TEMPS_DLC;
  *_ide = (uint8_t) BMS_TEMPS_IDE;
  return BMS_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->group_id = (uint8_t) ( (_d[0] & (0x0FU)) );
  _m->ic_id = (uint8_t) ( ((_d[0] >> 4U) & (0x0FU)) );
  _m->thermistor_id_0_ro = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_id_0_phys = (sigfloat_t)(HYTECH_thermistor_id_0_ro_fromS(_m->thermistor_id_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_id_1_ro = (int16_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_id_1_phys = (sigfloat_t)(HYTECH_thermistor_id_1_ro_fromS(_m->thermistor_id_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_id_2_ro = (int16_t) __ext_sig__(( ((_d[6] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_id_2_phys = (sigfloat_t)(HYTECH_thermistor_id_2_ro_fromS(_m->thermistor_id_2_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_DETAILED_TEMPS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_DETAILED_TEMPS_hytech(&_m->mon1, BMS_DETAILED_TEMPS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_DETAILED_TEMPS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_DETAILED_TEMPS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_id_0_ro = (int16_t) HYTECH_thermistor_id_0_ro_toS(_m->thermistor_id_0_phys);
  _m->thermistor_id_1_ro = (int16_t) HYTECH_thermistor_id_1_ro_toS(_m->thermistor_id_1_phys);
  _m->thermistor_id_2_ro = (int16_t) HYTECH_thermistor_id_2_ro_toS(_m->thermistor_id_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->group_id & (0x0FU)) | ((_m->ic_id & (0x0FU)) << 4U) );
  cframe->Data[1] |= (uint8_t) ( (_m->thermistor_id_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->thermistor_id_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->thermistor_id_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->thermistor_id_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->thermistor_id_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->thermistor_id_2_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_DETAILED_TEMPS_CANID;
  cframe->DLC = (uint8_t) BMS_DETAILED_TEMPS_DLC;
  cframe->IDE = (uint8_t) BMS_DETAILED_TEMPS_IDE;
  return BMS_DETAILED_TEMPS_CANID;
}

#else

uint32_t Pack_BMS_DETAILED_TEMPS_hytech(BMS_DETAILED_TEMPS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_DETAILED_TEMPS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_id_0_ro = (int16_t) HYTECH_thermistor_id_0_ro_toS(_m->thermistor_id_0_phys);
  _m->thermistor_id_1_ro = (int16_t) HYTECH_thermistor_id_1_ro_toS(_m->thermistor_id_1_phys);
  _m->thermistor_id_2_ro = (int16_t) HYTECH_thermistor_id_2_ro_toS(_m->thermistor_id_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->group_id & (0x0FU)) | ((_m->ic_id & (0x0FU)) << 4U) );
  _d[1] |= (uint8_t) ( (_m->thermistor_id_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->thermistor_id_0_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->thermistor_id_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->thermistor_id_1_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->thermistor_id_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->thermistor_id_2_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) BMS_DETAILED_TEMPS_DLC;
  *_ide = (uint8_t) BMS_DETAILED_TEMPS_IDE;
  return BMS_DETAILED_TEMPS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_STATUS_hytech(BMS_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->state = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->overvoltage_error = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->undervoltage_error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->total_voltage_high_error = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->discharge_overcurrent_error = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->charge_overcurrent_error = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->discharge_overtemp_error = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->charge_overtemp_error = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->undertemp_error = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->overtemp_error = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->current_ro = (int16_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->current_phys = (sigfloat_t)(HYTECH_current_ro_fromS(_m->current_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->shutdown_g_above_threshold_error = (uint8_t) ( (_d[5] & (0x01U)) );
  _m->shutdown_h_above_threshold_error = (uint8_t) ( ((_d[5] >> 1U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_STATUS_hytech(&_m->mon1, BMS_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_STATUS_hytech(BMS_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->current_ro = (int16_t) HYTECH_current_ro_toS(_m->current_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->state & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->overvoltage_error & (0x01U)) | ((_m->undervoltage_error & (0x01U)) << 1U) | ((_m->total_voltage_high_error & (0x01U)) << 2U) | ((_m->discharge_overcurrent_error & (0x01U)) << 3U) | ((_m->charge_overcurrent_error & (0x01U)) << 4U) | ((_m->discharge_overtemp_error & (0x01U)) << 5U) | ((_m->charge_overtemp_error & (0x01U)) << 6U) | ((_m->undertemp_error & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->overtemp_error & (0x01U)) );
  cframe->Data[3] |= (uint8_t) ( (_m->current_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->current_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->shutdown_g_above_threshold_error & (0x01U)) | ((_m->shutdown_h_above_threshold_error & (0x01U)) << 1U) );

  cframe->MsgId = (uint32_t) BMS_STATUS_CANID;
  cframe->DLC = (uint8_t) BMS_STATUS_DLC;
  cframe->IDE = (uint8_t) BMS_STATUS_IDE;
  return BMS_STATUS_CANID;
}

#else

uint32_t Pack_BMS_STATUS_hytech(BMS_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->current_ro = (int16_t) HYTECH_current_ro_toS(_m->current_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->state & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->overvoltage_error & (0x01U)) | ((_m->undervoltage_error & (0x01U)) << 1U) | ((_m->total_voltage_high_error & (0x01U)) << 2U) | ((_m->discharge_overcurrent_error & (0x01U)) << 3U) | ((_m->charge_overcurrent_error & (0x01U)) << 4U) | ((_m->discharge_overtemp_error & (0x01U)) << 5U) | ((_m->charge_overtemp_error & (0x01U)) << 6U) | ((_m->undertemp_error & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->overtemp_error & (0x01U)) );
  _d[3] |= (uint8_t) ( (_m->current_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->current_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->shutdown_g_above_threshold_error & (0x01U)) | ((_m->shutdown_h_above_threshold_error & (0x01U)) << 1U) );

  *_len = (uint8_t) BMS_STATUS_DLC;
  *_ide = (uint8_t) BMS_STATUS_IDE;
  return BMS_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CCU_STATUS_hytech(CCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->charger_enabled = (uint8_t) ( (_d[0] & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CCU_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CCU_STATUS_hytech(&_m->mon1, CCU_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CCU_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CCU_STATUS_hytech(CCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CCU_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->charger_enabled & (0x01U)) );

  cframe->MsgId = (uint32_t) CCU_STATUS_CANID;
  cframe->DLC = (uint8_t) CCU_STATUS_DLC;
  cframe->IDE = (uint8_t) CCU_STATUS_IDE;
  return CCU_STATUS_CANID;
}

#else

uint32_t Pack_CCU_STATUS_hytech(CCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CCU_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->charger_enabled & (0x01U)) );

  *_len = (uint8_t) CCU_STATUS_DLC;
  *_ide = (uint8_t) CCU_STATUS_IDE;
  return CCU_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->hall_curr_ref = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->hall_curr_signal = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < PENTHOUSE_ACCUM_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_PENTHOUSE_ACCUM_MSG_hytech(&_m->mon1, PENTHOUSE_ACCUM_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return PENTHOUSE_ACCUM_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(PENTHOUSE_ACCUM_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->hall_curr_ref & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->hall_curr_ref >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->hall_curr_signal & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->hall_curr_signal >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) PENTHOUSE_ACCUM_MSG_CANID;
  cframe->DLC = (uint8_t) PENTHOUSE_ACCUM_MSG_DLC;
  cframe->IDE = (uint8_t) PENTHOUSE_ACCUM_MSG_IDE;
  return PENTHOUSE_ACCUM_MSG_CANID;
}

#else

uint32_t Pack_PENTHOUSE_ACCUM_MSG_hytech(PENTHOUSE_ACCUM_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(PENTHOUSE_ACCUM_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->hall_curr_ref & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->hall_curr_ref >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->hall_curr_signal & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->hall_curr_signal >> 8U) & (0xFFU)) );

  *_len = (uint8_t) PENTHOUSE_ACCUM_MSG_DLC;
  *_ide = (uint8_t) PENTHOUSE_ACCUM_MSG_IDE;
  return PENTHOUSE_ACCUM_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_gps_time = (uint64_t) ( ((uint64_t)(_d[7] & (0xFFU)) << 56U) | ((uint64_t)(_d[6] & (0xFFU)) << 48U) | ((uint64_t)(_d[5] & (0xFFU)) << 40U) | ((uint64_t)(_d[4] & (0xFFU)) << 32U) | ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_GPS_TIME_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_GPS_TIME_MSG_hytech(&_m->mon1, VN_GPS_TIME_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_GPS_TIME_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_GPS_TIME_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->vn_gps_time & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_gps_time >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->vn_gps_time >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_gps_time >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->vn_gps_time >> 32U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_gps_time >> 40U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->vn_gps_time >> 48U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vn_gps_time >> 56U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_GPS_TIME_MSG_CANID;
  cframe->DLC = (uint8_t) VN_GPS_TIME_MSG_DLC;
  cframe->IDE = (uint8_t) VN_GPS_TIME_MSG_IDE;
  return VN_GPS_TIME_MSG_CANID;
}

#else

uint32_t Pack_VN_GPS_TIME_MSG_hytech(VN_GPS_TIME_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_GPS_TIME_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->vn_gps_time & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_gps_time >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->vn_gps_time >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_gps_time >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->vn_gps_time >> 32U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_gps_time >> 40U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->vn_gps_time >> 48U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vn_gps_time >> 56U) & (0xFFU)) );

  *_len = (uint8_t) VN_GPS_TIME_MSG_DLC;
  *_ide = (uint8_t) VN_GPS_TIME_MSG_IDE;
  return VN_GPS_TIME_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_STATUS_hytech(VN_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_gps_status = (uint8_t) ( (_d[0] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_STATUS_hytech(&_m->mon1, VN_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_STATUS_hytech(VN_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->vn_gps_status & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_STATUS_CANID;
  cframe->DLC = (uint8_t) VN_STATUS_DLC;
  cframe->IDE = (uint8_t) VN_STATUS_IDE;
  return VN_STATUS_CANID;
}

#else

uint32_t Pack_VN_STATUS_hytech(VN_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->vn_gps_status & (0xFFU)) );

  *_len = (uint8_t) VN_STATUS_DLC;
  *_ide = (uint8_t) VN_STATUS_IDE;
  return VN_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->total_charge_ro = (uint32_t) ( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->total_charge_phys = (sigfloat_t)(HYTECH_total_charge_ro_fromS(_m->total_charge_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->total_discharge_ro = (uint32_t) ( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->total_discharge_phys = (sigfloat_t)(HYTECH_total_discharge_ro_fromS(_m->total_discharge_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BMS_COULOMB_COUNTS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BMS_COULOMB_COUNTS_hytech(&_m->mon1, BMS_COULOMB_COUNTS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BMS_COULOMB_COUNTS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_COULOMB_COUNTS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->total_charge_ro = (uint32_t) HYTECH_total_charge_ro_toS(_m->total_charge_phys);
  _m->total_discharge_ro = (uint32_t) HYTECH_total_discharge_ro_toS(_m->total_discharge_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->total_charge_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->total_charge_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->total_charge_ro >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->total_charge_ro >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->total_discharge_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->total_discharge_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->total_discharge_ro >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->total_discharge_ro >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) BMS_COULOMB_COUNTS_CANID;
  cframe->DLC = (uint8_t) BMS_COULOMB_COUNTS_DLC;
  cframe->IDE = (uint8_t) BMS_COULOMB_COUNTS_IDE;
  return BMS_COULOMB_COUNTS_CANID;
}

#else

uint32_t Pack_BMS_COULOMB_COUNTS_hytech(BMS_COULOMB_COUNTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BMS_COULOMB_COUNTS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->total_charge_ro = (uint32_t) HYTECH_total_charge_ro_toS(_m->total_charge_phys);
  _m->total_discharge_ro = (uint32_t) HYTECH_total_discharge_ro_toS(_m->total_discharge_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->total_charge_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->total_charge_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->total_charge_ro >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->total_charge_ro >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->total_discharge_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->total_discharge_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->total_discharge_ro >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->total_discharge_ro >> 24U) & (0xFFU)) );

  *_len = (uint8_t) BMS_COULOMB_COUNTS_DLC;
  *_ide = (uint8_t) BMS_COULOMB_COUNTS_IDE;
  return BMS_COULOMB_COUNTS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->angular_rate_x_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->angular_rate_x_phys = (sigfloat_t)(HYTECH_angular_rate_x_ro_fromS(_m->angular_rate_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->angular_rate_y_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->angular_rate_y_phys = (sigfloat_t)(HYTECH_angular_rate_y_ro_fromS(_m->angular_rate_y_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->angular_rate_z_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->angular_rate_z_phys = (sigfloat_t)(HYTECH_angular_rate_z_ro_fromS(_m->angular_rate_z_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_ANGULAR_RATE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_ANGULAR_RATE_hytech(&_m->mon1, VN_ANGULAR_RATE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_ANGULAR_RATE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ANGULAR_RATE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->angular_rate_x_ro = (int16_t) HYTECH_angular_rate_x_ro_toS(_m->angular_rate_x_phys);
  _m->angular_rate_y_ro = (int16_t) HYTECH_angular_rate_y_ro_toS(_m->angular_rate_y_phys);
  _m->angular_rate_z_ro = (int16_t) HYTECH_angular_rate_z_ro_toS(_m->angular_rate_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->angular_rate_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->angular_rate_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->angular_rate_y_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->angular_rate_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->angular_rate_z_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->angular_rate_z_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_ANGULAR_RATE_CANID;
  cframe->DLC = (uint8_t) VN_ANGULAR_RATE_DLC;
  cframe->IDE = (uint8_t) VN_ANGULAR_RATE_IDE;
  return VN_ANGULAR_RATE_CANID;
}

#else

uint32_t Pack_VN_ANGULAR_RATE_hytech(VN_ANGULAR_RATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ANGULAR_RATE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->angular_rate_x_ro = (int16_t) HYTECH_angular_rate_x_ro_toS(_m->angular_rate_x_phys);
  _m->angular_rate_y_ro = (int16_t) HYTECH_angular_rate_y_ro_toS(_m->angular_rate_y_phys);
  _m->angular_rate_z_ro = (int16_t) HYTECH_angular_rate_z_ro_toS(_m->angular_rate_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->angular_rate_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->angular_rate_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->angular_rate_y_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->angular_rate_y_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->angular_rate_z_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->angular_rate_z_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VN_ANGULAR_RATE_DLC;
  *_ide = (uint8_t) VN_ANGULAR_RATE_IDE;
  return VN_ANGULAR_RATE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->rl_load_cell = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->rr_load_cell = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->rl_shock_pot = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->rr_shock_pot = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < REAR_SUSPENSION_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_REAR_SUSPENSION_hytech(&_m->mon1, REAR_SUSPENSION_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return REAR_SUSPENSION_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(REAR_SUSPENSION_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->rl_load_cell & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->rl_load_cell >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->rr_load_cell & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->rr_load_cell >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->rl_shock_pot & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->rl_shock_pot >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->rr_shock_pot & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->rr_shock_pot >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) REAR_SUSPENSION_CANID;
  cframe->DLC = (uint8_t) REAR_SUSPENSION_DLC;
  cframe->IDE = (uint8_t) REAR_SUSPENSION_IDE;
  return REAR_SUSPENSION_CANID;
}

#else

uint32_t Pack_REAR_SUSPENSION_hytech(REAR_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(REAR_SUSPENSION_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->rl_load_cell & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->rl_load_cell >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->rr_load_cell & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->rr_load_cell >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->rl_shock_pot & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->rl_shock_pot >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->rr_shock_pot & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->rr_shock_pot >> 8U) & (0xFFU)) );

  *_len = (uint8_t) REAR_SUSPENSION_DLC;
  *_ide = (uint8_t) REAR_SUSPENSION_IDE;
  return REAR_SUSPENSION_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->thermistor_inv1_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_inv1_phys = (sigfloat_t)(HYTECH_thermistor_inv1_ro_fromS(_m->thermistor_inv1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_inv2_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_inv2_phys = (sigfloat_t)(HYTECH_thermistor_inv2_ro_fromS(_m->thermistor_inv2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_acc1_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_acc1_phys = (sigfloat_t)(HYTECH_thermistor_acc1_ro_fromS(_m->thermistor_acc1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_acc2_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_acc2_phys = (sigfloat_t)(HYTECH_thermistor_acc2_ro_fromS(_m->thermistor_acc2_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAB_THERMISTORS_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAB_THERMISTORS_1_hytech(&_m->mon1, SAB_THERMISTORS_1_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return SAB_THERMISTORS_1_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(SAB_THERMISTORS_1_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_inv1_ro = (uint16_t) HYTECH_thermistor_inv1_ro_toS(_m->thermistor_inv1_phys);
  _m->thermistor_inv2_ro = (uint16_t) HYTECH_thermistor_inv2_ro_toS(_m->thermistor_inv2_phys);
  _m->thermistor_acc1_ro = (uint16_t) HYTECH_thermistor_acc1_ro_toS(_m->thermistor_acc1_phys);
  _m->thermistor_acc2_ro = (uint16_t) HYTECH_thermistor_acc2_ro_toS(_m->thermistor_acc2_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->thermistor_inv1_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->thermistor_inv1_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->thermistor_inv2_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->thermistor_inv2_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->thermistor_acc1_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->thermistor_acc1_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->thermistor_acc2_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->thermistor_acc2_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) SAB_THERMISTORS_1_CANID;
  cframe->DLC = (uint8_t) SAB_THERMISTORS_1_DLC;
  cframe->IDE = (uint8_t) SAB_THERMISTORS_1_IDE;
  return SAB_THERMISTORS_1_CANID;
}

#else

uint32_t Pack_SAB_THERMISTORS_1_hytech(SAB_THERMISTORS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(SAB_THERMISTORS_1_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_inv1_ro = (uint16_t) HYTECH_thermistor_inv1_ro_toS(_m->thermistor_inv1_phys);
  _m->thermistor_inv2_ro = (uint16_t) HYTECH_thermistor_inv2_ro_toS(_m->thermistor_inv2_phys);
  _m->thermistor_acc1_ro = (uint16_t) HYTECH_thermistor_acc1_ro_toS(_m->thermistor_acc1_phys);
  _m->thermistor_acc2_ro = (uint16_t) HYTECH_thermistor_acc2_ro_toS(_m->thermistor_acc2_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->thermistor_inv1_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->thermistor_inv1_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->thermistor_inv2_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->thermistor_inv2_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->thermistor_acc1_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->thermistor_acc1_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->thermistor_acc2_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->thermistor_acc2_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) SAB_THERMISTORS_1_DLC;
  *_ide = (uint8_t) SAB_THERMISTORS_1_IDE;
  return SAB_THERMISTORS_1_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->thermistor_motor_rl_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_rl_phys = (sigfloat_t)(HYTECH_thermistor_motor_rl_ro_fromS(_m->thermistor_motor_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_motor_rr_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_rr_phys = (sigfloat_t)(HYTECH_thermistor_motor_rr_ro_fromS(_m->thermistor_motor_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_pump_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_pump_phys = (sigfloat_t)(HYTECH_thermistor_pump_ro_fromS(_m->thermistor_pump_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < SAB_THERMISTORS_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_SAB_THERMISTORS_2_hytech(&_m->mon1, SAB_THERMISTORS_2_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return SAB_THERMISTORS_2_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(SAB_THERMISTORS_2_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_rl_ro = (uint16_t) HYTECH_thermistor_motor_rl_ro_toS(_m->thermistor_motor_rl_phys);
  _m->thermistor_motor_rr_ro = (uint16_t) HYTECH_thermistor_motor_rr_ro_toS(_m->thermistor_motor_rr_phys);
  _m->thermistor_pump_ro = (uint16_t) HYTECH_thermistor_pump_ro_toS(_m->thermistor_pump_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->thermistor_motor_rl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->thermistor_motor_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->thermistor_motor_rr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->thermistor_motor_rr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->thermistor_pump_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->thermistor_pump_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) SAB_THERMISTORS_2_CANID;
  cframe->DLC = (uint8_t) SAB_THERMISTORS_2_DLC;
  cframe->IDE = (uint8_t) SAB_THERMISTORS_2_IDE;
  return SAB_THERMISTORS_2_CANID;
}

#else

uint32_t Pack_SAB_THERMISTORS_2_hytech(SAB_THERMISTORS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(SAB_THERMISTORS_2_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_rl_ro = (uint16_t) HYTECH_thermistor_motor_rl_ro_toS(_m->thermistor_motor_rl_phys);
  _m->thermistor_motor_rr_ro = (uint16_t) HYTECH_thermistor_motor_rr_ro_toS(_m->thermistor_motor_rr_phys);
  _m->thermistor_pump_ro = (uint16_t) HYTECH_thermistor_pump_ro_toS(_m->thermistor_pump_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->thermistor_motor_rl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->thermistor_motor_rl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->thermistor_motor_rr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->thermistor_motor_rr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->thermistor_pump_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->thermistor_pump_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) SAB_THERMISTORS_2_DLC;
  *_ide = (uint8_t) SAB_THERMISTORS_2_IDE;
  return SAB_THERMISTORS_2_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_ecef_pos_x_ro = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_x_phys = (sigfloat_t)(HYTECH_vn_ecef_pos_x_ro_fromS(_m->vn_ecef_pos_x_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vn_ecef_pos_y_ro = (int32_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 24U) | ((_d[6] & (0xFFU)) << 16U) | ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_y_phys = (sigfloat_t)(HYTECH_vn_ecef_pos_y_ro_fromS(_m->vn_ecef_pos_y_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_ECEF_POS_XY_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_ECEF_POS_XY_hytech(&_m->mon1, VN_ECEF_POS_XY_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_ECEF_POS_XY_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ECEF_POS_XY_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_x_ro = (int32_t) HYTECH_vn_ecef_pos_x_ro_toS(_m->vn_ecef_pos_x_phys);
  _m->vn_ecef_pos_y_ro = (int32_t) HYTECH_vn_ecef_pos_y_ro_toS(_m->vn_ecef_pos_y_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_ecef_pos_x_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 24U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vn_ecef_pos_y_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 16U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_ECEF_POS_XY_CANID;
  cframe->DLC = (uint8_t) VN_ECEF_POS_XY_DLC;
  cframe->IDE = (uint8_t) VN_ECEF_POS_XY_IDE;
  return VN_ECEF_POS_XY_CANID;
}

#else

uint32_t Pack_VN_ECEF_POS_XY_hytech(VN_ECEF_POS_XY_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ECEF_POS_XY_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_x_ro = (int32_t) HYTECH_vn_ecef_pos_x_ro_toS(_m->vn_ecef_pos_x_phys);
  _m->vn_ecef_pos_y_ro = (int32_t) HYTECH_vn_ecef_pos_y_ro_toS(_m->vn_ecef_pos_y_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_ecef_pos_x_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_ecef_pos_x_ro >> 24U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vn_ecef_pos_y_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 16U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vn_ecef_pos_y_ro >> 24U) & (0xFFU)) );

  *_len = (uint8_t) VN_ECEF_POS_XY_DLC;
  *_ide = (uint8_t) VN_ECEF_POS_XY_IDE;
  return VN_ECEF_POS_XY_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCU_STATUS_hytech(TCU_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->shutdown_status = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->tcu_recording_state = (uint8_t) ( (_d[2] & (0x03U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TCU_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TCU_STATUS_hytech(&_m->mon1, TCU_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TCU_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TCU_STATUS_hytech(TCU_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->shutdown_status & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->shutdown_status >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->tcu_recording_state & (0x03U)) );

  cframe->MsgId = (uint32_t) TCU_STATUS_CANID;
  cframe->DLC = (uint8_t) TCU_STATUS_DLC;
  cframe->IDE = (uint8_t) TCU_STATUS_IDE;
  return TCU_STATUS_CANID;
}

#else

uint32_t Pack_TCU_STATUS_hytech(TCU_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCU_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->shutdown_status & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->shutdown_status >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->tcu_recording_state & (0x03U)) );

  *_len = (uint8_t) TCU_STATUS_DLC;
  *_ide = (uint8_t) TCU_STATUS_IDE;
  return TCU_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_ecef_pos_z_ro = (int32_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_z_phys = (sigfloat_t)(HYTECH_vn_ecef_pos_z_ro_fromS(_m->vn_ecef_pos_z_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_ECEF_POS_Z_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_ECEF_POS_Z_MSG_hytech(&_m->mon1, VN_ECEF_POS_Z_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_ECEF_POS_Z_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ECEF_POS_Z_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_z_ro = (int32_t) HYTECH_vn_ecef_pos_z_ro_toS(_m->vn_ecef_pos_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vn_ecef_pos_z_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_ECEF_POS_Z_MSG_CANID;
  cframe->DLC = (uint8_t) VN_ECEF_POS_Z_MSG_DLC;
  cframe->IDE = (uint8_t) VN_ECEF_POS_Z_MSG_IDE;
  return VN_ECEF_POS_Z_MSG_CANID;
}

#else

uint32_t Pack_VN_ECEF_POS_Z_MSG_hytech(VN_ECEF_POS_Z_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_ECEF_POS_Z_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vn_ecef_pos_z_ro = (int32_t) HYTECH_vn_ecef_pos_z_ro_toS(_m->vn_ecef_pos_z_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vn_ecef_pos_z_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vn_ecef_pos_z_ro >> 24U) & (0xFFU)) );

  *_len = (uint8_t) VN_ECEF_POS_Z_MSG_DLC;
  *_ide = (uint8_t) VN_ECEF_POS_Z_MSG_IDE;
  return VN_ECEF_POS_Z_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->num_sats_pvt_1 = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->num_sats_rtk_1 = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->highest_cn0_1_ro = (uint8_t) ( (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->highest_cn0_1_phys = (sigfloat_t)(HYTECH_highest_cn0_1_ro_fromS(_m->highest_cn0_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->num_sats_pvt_2 = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->num_sats_rtk_2 = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->highest_cn0_2_ro = (uint8_t) ( (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->highest_cn0_2_phys = (sigfloat_t)(HYTECH_highest_cn0_2_ro_fromS(_m->highest_cn0_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->num_com_sats_pvt = (uint8_t) ( (_d[6] & (0xFFU)) );
  _m->num_com_sats_rtk = (uint8_t) ( (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VN_GNSS_COMP_SIG_HEALTH_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VN_GNSS_COMP_SIG_HEALTH_hytech(&_m->mon1, VN_GNSS_COMP_SIG_HEALTH_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VN_GNSS_COMP_SIG_HEALTH_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_GNSS_COMP_SIG_HEALTH_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->highest_cn0_1_ro = (uint8_t) HYTECH_highest_cn0_1_ro_toS(_m->highest_cn0_1_phys);
  _m->highest_cn0_2_ro = (uint8_t) HYTECH_highest_cn0_2_ro_toS(_m->highest_cn0_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->num_sats_pvt_1 & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->num_sats_rtk_1 & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->highest_cn0_1_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->num_sats_pvt_2 & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->num_sats_rtk_2 & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->highest_cn0_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->num_com_sats_pvt & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->num_com_sats_rtk & (0xFFU)) );

  cframe->MsgId = (uint32_t) VN_GNSS_COMP_SIG_HEALTH_CANID;
  cframe->DLC = (uint8_t) VN_GNSS_COMP_SIG_HEALTH_DLC;
  cframe->IDE = (uint8_t) VN_GNSS_COMP_SIG_HEALTH_IDE;
  return VN_GNSS_COMP_SIG_HEALTH_CANID;
}

#else

uint32_t Pack_VN_GNSS_COMP_SIG_HEALTH_hytech(VN_GNSS_COMP_SIG_HEALTH_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VN_GNSS_COMP_SIG_HEALTH_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->highest_cn0_1_ro = (uint8_t) HYTECH_highest_cn0_1_ro_toS(_m->highest_cn0_1_phys);
  _m->highest_cn0_2_ro = (uint8_t) HYTECH_highest_cn0_2_ro_toS(_m->highest_cn0_2_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->num_sats_pvt_1 & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->num_sats_rtk_1 & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->highest_cn0_1_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->num_sats_pvt_2 & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->num_sats_rtk_2 & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->highest_cn0_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->num_com_sats_pvt & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->num_com_sats_rtk & (0xFFU)) );

  *_len = (uint8_t) VN_GNSS_COMP_SIG_HEALTH_DLC;
  *_ide = (uint8_t) VN_GNSS_COMP_SIG_HEALTH_IDE;
  return VN_GNSS_COMP_SIG_HEALTH_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->bots_led = (uint8_t) ( (_d[0] & (0x03U)) );
  _m->launch_control_led = (uint8_t) ( ((_d[0] >> 2U) & (0x03U)) );
  _m->mode_led = (uint8_t) ( ((_d[0] >> 4U) & (0x03U)) );
  _m->mechanical_brake_led = (uint8_t) ( ((_d[0] >> 6U) & (0x03U)) );
  _m->cockpit_brb_led = (uint8_t) ( (_d[1] & (0x03U)) );
  _m->inertia_status_led = (uint8_t) ( ((_d[1] >> 2U) & (0x03U)) );
  _m->start_status_led = (uint8_t) ( ((_d[1] >> 4U) & (0x03U)) );
  _m->motor_controller_error_led = (uint8_t) ( ((_d[1] >> 6U) & (0x03U)) );
  _m->imd_led = (uint8_t) ( (_d[2] & (0x03U)) );
  _m->ams_led = (uint8_t) ( ((_d[2] >> 2U) & (0x03U)) );
  _m->drive_buzzer = (uint8_t) ( ((_d[2] >> 4U) & (0x01U)) );
  _m->glv_led = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->pack_charge_led = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->dial_state = (uint8_t) ( (_d[5] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASHBOARD_MCU_STATE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASHBOARD_MCU_STATE_hytech(&_m->mon1, DASHBOARD_MCU_STATE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DASHBOARD_MCU_STATE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_MCU_STATE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->bots_led & (0x03U)) | ((_m->launch_control_led & (0x03U)) << 2U) | ((_m->mode_led & (0x03U)) << 4U) | ((_m->mechanical_brake_led & (0x03U)) << 6U) );
  cframe->Data[1] |= (uint8_t) ( (_m->cockpit_brb_led & (0x03U)) | ((_m->inertia_status_led & (0x03U)) << 2U) | ((_m->start_status_led & (0x03U)) << 4U) | ((_m->motor_controller_error_led & (0x03U)) << 6U) );
  cframe->Data[2] |= (uint8_t) ( (_m->imd_led & (0x03U)) | ((_m->ams_led & (0x03U)) << 2U) | ((_m->drive_buzzer & (0x01U)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( (_m->glv_led & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->pack_charge_led & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->dial_state & (0xFFU)) );

  cframe->MsgId = (uint32_t) DASHBOARD_MCU_STATE_CANID;
  cframe->DLC = (uint8_t) DASHBOARD_MCU_STATE_DLC;
  cframe->IDE = (uint8_t) DASHBOARD_MCU_STATE_IDE;
  return DASHBOARD_MCU_STATE_CANID;
}

#else

uint32_t Pack_DASHBOARD_MCU_STATE_hytech(DASHBOARD_MCU_STATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_MCU_STATE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->bots_led & (0x03U)) | ((_m->launch_control_led & (0x03U)) << 2U) | ((_m->mode_led & (0x03U)) << 4U) | ((_m->mechanical_brake_led & (0x03U)) << 6U) );
  _d[1] |= (uint8_t) ( (_m->cockpit_brb_led & (0x03U)) | ((_m->inertia_status_led & (0x03U)) << 2U) | ((_m->start_status_led & (0x03U)) << 4U) | ((_m->motor_controller_error_led & (0x03U)) << 6U) );
  _d[2] |= (uint8_t) ( (_m->imd_led & (0x03U)) | ((_m->ams_led & (0x03U)) << 2U) | ((_m->drive_buzzer & (0x01U)) << 4U) );
  _d[3] |= (uint8_t) ( (_m->glv_led & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->pack_charge_led & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->dial_state & (0xFFU)) );

  *_len = (uint8_t) DASHBOARD_MCU_STATE_DLC;
  *_ide = (uint8_t) DASHBOARD_MCU_STATE_IDE;
  return DASHBOARD_MCU_STATE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->start_button = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->mark_button = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->mode_button = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->motor_controller_cycle_button = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->launch_ctrl_button = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->torque_mode_button = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->led_dimmer_button = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->left_shifter_button = (uint8_t) ( ((_d[0] >> 7U) & (0x01U)) );
  _m->right_shifter_button = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->shutdown_h_above_threshold = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->ssok_above_threshold = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->drive_buzzer = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->tcu_recording_state = (uint8_t) ( ((_d[1] >> 4U) & (0x03U)) );
  _m->dial_state = (uint8_t) ( (_d[2] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASHBOARD_STATE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASHBOARD_STATE_hytech(&_m->mon1, DASHBOARD_STATE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DASHBOARD_STATE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_STATE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->start_button & (0x01U)) | ((_m->mark_button & (0x01U)) << 1U) | ((_m->mode_button & (0x01U)) << 2U) | ((_m->motor_controller_cycle_button & (0x01U)) << 3U) | ((_m->launch_ctrl_button & (0x01U)) << 4U) | ((_m->torque_mode_button & (0x01U)) << 5U) | ((_m->led_dimmer_button & (0x01U)) << 6U) | ((_m->left_shifter_button & (0x01U)) << 7U) );
  cframe->Data[1] |= (uint8_t) ( (_m->right_shifter_button & (0x01U)) | ((_m->shutdown_h_above_threshold & (0x01U)) << 1U) | ((_m->ssok_above_threshold & (0x01U)) << 2U) | ((_m->drive_buzzer & (0x01U)) << 3U) | ((_m->tcu_recording_state & (0x03U)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( (_m->dial_state & (0xFFU)) );

  cframe->MsgId = (uint32_t) DASHBOARD_STATE_CANID;
  cframe->DLC = (uint8_t) DASHBOARD_STATE_DLC;
  cframe->IDE = (uint8_t) DASHBOARD_STATE_IDE;
  return DASHBOARD_STATE_CANID;
}

#else

uint32_t Pack_DASHBOARD_STATE_hytech(DASHBOARD_STATE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_STATE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->start_button & (0x01U)) | ((_m->mark_button & (0x01U)) << 1U) | ((_m->mode_button & (0x01U)) << 2U) | ((_m->motor_controller_cycle_button & (0x01U)) << 3U) | ((_m->launch_ctrl_button & (0x01U)) << 4U) | ((_m->torque_mode_button & (0x01U)) << 5U) | ((_m->led_dimmer_button & (0x01U)) << 6U) | ((_m->left_shifter_button & (0x01U)) << 7U) );
  _d[1] |= (uint8_t) ( (_m->right_shifter_button & (0x01U)) | ((_m->shutdown_h_above_threshold & (0x01U)) << 1U) | ((_m->ssok_above_threshold & (0x01U)) << 2U) | ((_m->drive_buzzer & (0x01U)) << 3U) | ((_m->tcu_recording_state & (0x03U)) << 4U) );
  _d[2] |= (uint8_t) ( (_m->dial_state & (0xFFU)) );

  *_len = (uint8_t) DASHBOARD_STATE_DLC;
  *_ide = (uint8_t) DASHBOARD_STATE_IDE;
  return DASHBOARD_STATE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->fl_load_cell = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->fl_shock_pot = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->fr_load_cell = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->fr_shock_pot = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FRONT_SUSPENSION_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FRONT_SUSPENSION_hytech(&_m->mon1, FRONT_SUSPENSION_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FRONT_SUSPENSION_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FRONT_SUSPENSION_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->fl_load_cell & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->fl_load_cell >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->fl_shock_pot & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->fl_shock_pot >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->fr_load_cell & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->fr_load_cell >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->fr_shock_pot & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->fr_shock_pot >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) FRONT_SUSPENSION_CANID;
  cframe->DLC = (uint8_t) FRONT_SUSPENSION_DLC;
  cframe->IDE = (uint8_t) FRONT_SUSPENSION_IDE;
  return FRONT_SUSPENSION_CANID;
}

#else

uint32_t Pack_FRONT_SUSPENSION_hytech(FRONT_SUSPENSION_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FRONT_SUSPENSION_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->fl_load_cell & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->fl_load_cell >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->fl_shock_pot & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->fl_shock_pot >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->fr_load_cell & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->fr_load_cell >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->fr_shock_pot & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->fr_shock_pot >> 8U) & (0xFFU)) );

  *_len = (uint8_t) FRONT_SUSPENSION_DLC;
  *_ide = (uint8_t) FRONT_SUSPENSION_IDE;
  return FRONT_SUSPENSION_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->drivebrain_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_phys = (sigfloat_t)(HYTECH_drivebrain_torque_fl_ro_fromS(_m->drivebrain_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fr_phys = (sigfloat_t)(HYTECH_drivebrain_torque_fr_ro_fromS(_m->drivebrain_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_rl_phys = (sigfloat_t)(HYTECH_drivebrain_torque_rl_ro_fromS(_m->drivebrain_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_rr_phys = (sigfloat_t)(HYTECH_drivebrain_torque_rr_ro_fromS(_m->drivebrain_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVEBRAIN_TORQUE_LIM_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(&_m->mon1, DRIVEBRAIN_TORQUE_LIM_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVEBRAIN_TORQUE_LIM_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_TORQUE_LIM_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_ro = (int16_t) HYTECH_drivebrain_torque_fl_ro_toS(_m->drivebrain_torque_fl_phys);
  _m->drivebrain_torque_fr_ro = (int16_t) HYTECH_drivebrain_torque_fr_ro_toS(_m->drivebrain_torque_fr_phys);
  _m->drivebrain_torque_rl_ro = (int16_t) HYTECH_drivebrain_torque_rl_ro_toS(_m->drivebrain_torque_rl_phys);
  _m->drivebrain_torque_rr_ro = (int16_t) HYTECH_drivebrain_torque_rr_ro_toS(_m->drivebrain_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->drivebrain_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->drivebrain_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->drivebrain_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->drivebrain_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->drivebrain_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->drivebrain_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->drivebrain_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->drivebrain_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVEBRAIN_TORQUE_LIM_INPUT_CANID;
  cframe->DLC = (uint8_t) DRIVEBRAIN_TORQUE_LIM_INPUT_DLC;
  cframe->IDE = (uint8_t) DRIVEBRAIN_TORQUE_LIM_INPUT_IDE;
  return DRIVEBRAIN_TORQUE_LIM_INPUT_CANID;
}

#else

uint32_t Pack_DRIVEBRAIN_TORQUE_LIM_INPUT_hytech(DRIVEBRAIN_TORQUE_LIM_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_TORQUE_LIM_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_ro = (int16_t) HYTECH_drivebrain_torque_fl_ro_toS(_m->drivebrain_torque_fl_phys);
  _m->drivebrain_torque_fr_ro = (int16_t) HYTECH_drivebrain_torque_fr_ro_toS(_m->drivebrain_torque_fr_phys);
  _m->drivebrain_torque_rl_ro = (int16_t) HYTECH_drivebrain_torque_rl_ro_toS(_m->drivebrain_torque_rl_phys);
  _m->drivebrain_torque_rr_ro = (int16_t) HYTECH_drivebrain_torque_rr_ro_toS(_m->drivebrain_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->drivebrain_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->drivebrain_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->drivebrain_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->drivebrain_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->drivebrain_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->drivebrain_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->drivebrain_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->drivebrain_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVEBRAIN_TORQUE_LIM_INPUT_DLC;
  *_ide = (uint8_t) DRIVEBRAIN_TORQUE_LIM_INPUT_IDE;
  return DRIVEBRAIN_TORQUE_LIM_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->drivebrain_set_rpm_fl = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->drivebrain_set_rpm_fr = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->drivebrain_set_rpm_rl = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->drivebrain_set_rpm_rr = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVEBRAIN_SPEED_SET_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVEBRAIN_SPEED_SET_INPUT_hytech(&_m->mon1, DRIVEBRAIN_SPEED_SET_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVEBRAIN_SPEED_SET_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_SPEED_SET_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->drivebrain_set_rpm_fl & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->drivebrain_set_rpm_fl >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->drivebrain_set_rpm_fr & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->drivebrain_set_rpm_fr >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->drivebrain_set_rpm_rl & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->drivebrain_set_rpm_rl >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->drivebrain_set_rpm_rr & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->drivebrain_set_rpm_rr >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVEBRAIN_SPEED_SET_INPUT_CANID;
  cframe->DLC = (uint8_t) DRIVEBRAIN_SPEED_SET_INPUT_DLC;
  cframe->IDE = (uint8_t) DRIVEBRAIN_SPEED_SET_INPUT_IDE;
  return DRIVEBRAIN_SPEED_SET_INPUT_CANID;
}

#else

uint32_t Pack_DRIVEBRAIN_SPEED_SET_INPUT_hytech(DRIVEBRAIN_SPEED_SET_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_SPEED_SET_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->drivebrain_set_rpm_fl & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->drivebrain_set_rpm_fl >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->drivebrain_set_rpm_fr & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->drivebrain_set_rpm_fr >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->drivebrain_set_rpm_rl & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->drivebrain_set_rpm_rl >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->drivebrain_set_rpm_rr & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->drivebrain_set_rpm_rr >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVEBRAIN_SPEED_SET_INPUT_DLC;
  *_ide = (uint8_t) DRIVEBRAIN_SPEED_SET_INPUT_IDE;
  return DRIVEBRAIN_SPEED_SET_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->drivebrain_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_phys = (sigfloat_t)(HYTECH_drivebrain_torque_fl_ro_fromS(_m->drivebrain_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fr_phys = (sigfloat_t)(HYTECH_drivebrain_torque_fr_ro_fromS(_m->drivebrain_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_rl_phys = (sigfloat_t)(HYTECH_drivebrain_torque_rl_ro_fromS(_m->drivebrain_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivebrain_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_rr_phys = (sigfloat_t)(HYTECH_drivebrain_torque_rr_ro_fromS(_m->drivebrain_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(&_m->mon1, DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_ro = (int16_t) HYTECH_drivebrain_torque_fl_ro_toS(_m->drivebrain_torque_fl_phys);
  _m->drivebrain_torque_fr_ro = (int16_t) HYTECH_drivebrain_torque_fr_ro_toS(_m->drivebrain_torque_fr_phys);
  _m->drivebrain_torque_rl_ro = (int16_t) HYTECH_drivebrain_torque_rl_ro_toS(_m->drivebrain_torque_rl_phys);
  _m->drivebrain_torque_rr_ro = (int16_t) HYTECH_drivebrain_torque_rr_ro_toS(_m->drivebrain_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->drivebrain_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->drivebrain_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->drivebrain_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->drivebrain_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->drivebrain_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->drivebrain_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->drivebrain_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->drivebrain_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID;
  cframe->DLC = (uint8_t) DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC;
  cframe->IDE = (uint8_t) DRIVEBRAIN_DESIRED_TORQUE_INPUT_IDE;
  return DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID;
}

#else

uint32_t Pack_DRIVEBRAIN_DESIRED_TORQUE_INPUT_hytech(DRIVEBRAIN_DESIRED_TORQUE_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivebrain_torque_fl_ro = (int16_t) HYTECH_drivebrain_torque_fl_ro_toS(_m->drivebrain_torque_fl_phys);
  _m->drivebrain_torque_fr_ro = (int16_t) HYTECH_drivebrain_torque_fr_ro_toS(_m->drivebrain_torque_fr_phys);
  _m->drivebrain_torque_rl_ro = (int16_t) HYTECH_drivebrain_torque_rl_ro_toS(_m->drivebrain_torque_rl_phys);
  _m->drivebrain_torque_rr_ro = (int16_t) HYTECH_drivebrain_torque_rr_ro_toS(_m->drivebrain_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->drivebrain_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->drivebrain_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->drivebrain_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->drivebrain_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->drivebrain_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->drivebrain_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->drivebrain_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->drivebrain_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVEBRAIN_DESIRED_TORQUE_INPUT_DLC;
  *_ide = (uint8_t) DRIVEBRAIN_DESIRED_TORQUE_INPUT_IDE;
  return DRIVEBRAIN_DESIRED_TORQUE_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->thermistor_motor_fl_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_fl_phys = (sigfloat_t)(HYTECH_thermistor_motor_fl_ro_fromS(_m->thermistor_motor_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->thermistor_motor_fr_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_fr_phys = (sigfloat_t)(HYTECH_thermistor_motor_fr_ro_fromS(_m->thermistor_motor_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FRONT_THERMISTORS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FRONT_THERMISTORS_hytech(&_m->mon1, FRONT_THERMISTORS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FRONT_THERMISTORS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FRONT_THERMISTORS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_fl_ro = (uint16_t) HYTECH_thermistor_motor_fl_ro_toS(_m->thermistor_motor_fl_phys);
  _m->thermistor_motor_fr_ro = (uint16_t) HYTECH_thermistor_motor_fr_ro_toS(_m->thermistor_motor_fr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->thermistor_motor_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->thermistor_motor_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->thermistor_motor_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->thermistor_motor_fr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) FRONT_THERMISTORS_CANID;
  cframe->DLC = (uint8_t) FRONT_THERMISTORS_DLC;
  cframe->IDE = (uint8_t) FRONT_THERMISTORS_IDE;
  return FRONT_THERMISTORS_CANID;
}

#else

uint32_t Pack_FRONT_THERMISTORS_hytech(FRONT_THERMISTORS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FRONT_THERMISTORS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->thermistor_motor_fl_ro = (uint16_t) HYTECH_thermistor_motor_fl_ro_toS(_m->thermistor_motor_fl_phys);
  _m->thermistor_motor_fr_ro = (uint16_t) HYTECH_thermistor_motor_fr_ro_toS(_m->thermistor_motor_fr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->thermistor_motor_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->thermistor_motor_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->thermistor_motor_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->thermistor_motor_fr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) FRONT_THERMISTORS_DLC;
  *_ide = (uint8_t) FRONT_THERMISTORS_IDE;
  return FRONT_THERMISTORS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->em_current_ro = (int32_t) __ext_sig__(( ((_d[0] & (0xFFU)) << 24U) | ((_d[1] & (0xFFU)) << 16U) | ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_current_phys = (sigfloat_t)(HYTECH_em_current_ro_fromS(_m->em_current_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_voltage_ro = (int32_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 24U) | ((_d[5] & (0xFFU)) << 16U) | ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) ), 32);
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_voltage_phys = (sigfloat_t)(HYTECH_em_voltage_ro_fromS(_m->em_voltage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < EM_MEASUREMENT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_EM_MEASUREMENT_hytech(&_m->mon1, EM_MEASUREMENT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return EM_MEASUREMENT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(EM_MEASUREMENT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->em_current_ro = (int32_t) HYTECH_em_current_ro_toS(_m->em_current_phys);
  _m->em_voltage_ro = (int32_t) HYTECH_em_voltage_ro_toS(_m->em_voltage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->em_current_ro >> 24U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->em_current_ro >> 16U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->em_current_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->em_current_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->em_voltage_ro >> 24U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->em_voltage_ro >> 16U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->em_voltage_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->em_voltage_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) EM_MEASUREMENT_CANID;
  cframe->DLC = (uint8_t) EM_MEASUREMENT_DLC;
  cframe->IDE = (uint8_t) EM_MEASUREMENT_IDE;
  return EM_MEASUREMENT_CANID;
}

#else

uint32_t Pack_EM_MEASUREMENT_hytech(EM_MEASUREMENT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(EM_MEASUREMENT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->em_current_ro = (int32_t) HYTECH_em_current_ro_toS(_m->em_current_phys);
  _m->em_voltage_ro = (int32_t) HYTECH_em_voltage_ro_toS(_m->em_voltage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->em_current_ro >> 24U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->em_current_ro >> 16U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->em_current_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->em_current_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->em_voltage_ro >> 24U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->em_voltage_ro >> 16U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->em_voltage_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->em_voltage_ro & (0xFFU)) );

  *_len = (uint8_t) EM_MEASUREMENT_DLC;
  *_ide = (uint8_t) EM_MEASUREMENT_IDE;
  return EM_MEASUREMENT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->torque_controller_mux_status = (uint8_t) ( (_d[0] & (0x07U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MCU_ERROR_STATES_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MCU_ERROR_STATES_hytech(&_m->mon1, MCU_ERROR_STATES_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return MCU_ERROR_STATES_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_ERROR_STATES_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->torque_controller_mux_status & (0x07U)) );

  cframe->MsgId = (uint32_t) MCU_ERROR_STATES_CANID;
  cframe->DLC = (uint8_t) MCU_ERROR_STATES_DLC;
  cframe->IDE = (uint8_t) MCU_ERROR_STATES_IDE;
  return MCU_ERROR_STATES_CANID;
}

#else

uint32_t Pack_MCU_ERROR_STATES_hytech(MCU_ERROR_STATES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(MCU_ERROR_STATES_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->torque_controller_mux_status & (0x07U)) );

  *_len = (uint8_t) MCU_ERROR_STATES_DLC;
  *_ide = (uint8_t) MCU_ERROR_STATES_IDE;
  return MCU_ERROR_STATES_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_setpoint_rpm = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->positive_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_phys = (sigfloat_t)(HYTECH_positive_torque_limit_ro_fromS(_m->positive_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->negative_torque_limit_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->negative_torque_limit_phys = (sigfloat_t)(HYTECH_negative_torque_limit_ro_fromS(_m->negative_torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_CONTROL_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_CONTROL_INPUT_hytech(&_m->mon1, INV4_CONTROL_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_CONTROL_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_CONTROL_INPUT_CANID;
  cframe->DLC = (uint8_t) INV4_CONTROL_INPUT_DLC;
  cframe->IDE = (uint8_t) INV4_CONTROL_INPUT_IDE;
  return INV4_CONTROL_INPUT_CANID;
}

#else

uint32_t Pack_INV4_CONTROL_INPUT_hytech(INV4_CONTROL_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->positive_torque_limit_ro = (int16_t) HYTECH_positive_torque_limit_ro_toS(_m->positive_torque_limit_phys);
  _m->negative_torque_limit_ro = (int16_t) HYTECH_negative_torque_limit_ro_toS(_m->negative_torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->speed_setpoint_rpm & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_setpoint_rpm >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->positive_torque_limit_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->positive_torque_limit_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->negative_torque_limit_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->negative_torque_limit_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_CONTROL_INPUT_DLC;
  *_ide = (uint8_t) INV4_CONTROL_INPUT_IDE;
  return INV4_CONTROL_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_CONTROL_PARAMETER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_CONTROL_PARAMETER_hytech(&_m->mon1, INV1_CONTROL_PARAMETER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_CONTROL_PARAMETER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_PARAMETER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_CONTROL_PARAMETER_CANID;
  cframe->DLC = (uint8_t) INV1_CONTROL_PARAMETER_DLC;
  cframe->IDE = (uint8_t) INV1_CONTROL_PARAMETER_IDE;
  return INV1_CONTROL_PARAMETER_CANID;
}

#else

uint32_t Pack_INV1_CONTROL_PARAMETER_hytech(INV1_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_CONTROL_PARAMETER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_CONTROL_PARAMETER_DLC;
  *_ide = (uint8_t) INV1_CONTROL_PARAMETER_IDE;
  return INV1_CONTROL_PARAMETER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_CONTROL_PARAMETER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_CONTROL_PARAMETER_hytech(&_m->mon1, INV2_CONTROL_PARAMETER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_CONTROL_PARAMETER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_PARAMETER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_CONTROL_PARAMETER_CANID;
  cframe->DLC = (uint8_t) INV2_CONTROL_PARAMETER_DLC;
  cframe->IDE = (uint8_t) INV2_CONTROL_PARAMETER_IDE;
  return INV2_CONTROL_PARAMETER_CANID;
}

#else

uint32_t Pack_INV2_CONTROL_PARAMETER_hytech(INV2_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_CONTROL_PARAMETER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_CONTROL_PARAMETER_DLC;
  *_ide = (uint8_t) INV2_CONTROL_PARAMETER_IDE;
  return INV2_CONTROL_PARAMETER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_CONTROL_PARAMETER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_CONTROL_PARAMETER_hytech(&_m->mon1, INV3_CONTROL_PARAMETER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_CONTROL_PARAMETER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_PARAMETER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_CONTROL_PARAMETER_CANID;
  cframe->DLC = (uint8_t) INV3_CONTROL_PARAMETER_DLC;
  cframe->IDE = (uint8_t) INV3_CONTROL_PARAMETER_IDE;
  return INV3_CONTROL_PARAMETER_CANID;
}

#else

uint32_t Pack_INV3_CONTROL_PARAMETER_hytech(INV3_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_CONTROL_PARAMETER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_CONTROL_PARAMETER_DLC;
  *_ide = (uint8_t) INV3_CONTROL_PARAMETER_IDE;
  return INV3_CONTROL_PARAMETER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_control_kp = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->speed_control_ki = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->speed_control_kd = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_CONTROL_PARAMETER_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_CONTROL_PARAMETER_hytech(&_m->mon1, INV4_CONTROL_PARAMETER_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_CONTROL_PARAMETER_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_PARAMETER_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_CONTROL_PARAMETER_CANID;
  cframe->DLC = (uint8_t) INV4_CONTROL_PARAMETER_DLC;
  cframe->IDE = (uint8_t) INV4_CONTROL_PARAMETER_IDE;
  return INV4_CONTROL_PARAMETER_CANID;
}

#else

uint32_t Pack_INV4_CONTROL_PARAMETER_hytech(INV4_CONTROL_PARAMETER_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_CONTROL_PARAMETER_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->speed_control_kp & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->speed_control_kp >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->speed_control_ki & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->speed_control_ki >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->speed_control_kd & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->speed_control_kd >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_CONTROL_PARAMETER_DLC;
  *_ide = (uint8_t) INV4_CONTROL_PARAMETER_IDE;
  return INV4_CONTROL_PARAMETER_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->current_A = (uint32_t) ( ((_d[0] & (0xFFU)) << 24U) | ((_d[1] & (0xFFU)) << 16U) | ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->voltage_V = (uint32_t) ( ((_d[4] & (0xFFU)) << 24U) | ((_d[5] & (0xFFU)) << 16U) | ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENERGY_METER_MEAS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENERGY_METER_MEAS_hytech(&_m->mon1, ENERGY_METER_MEAS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return ENERGY_METER_MEAS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_MEAS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( ((_m->current_A >> 24U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->current_A >> 16U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->current_A >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->current_A & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->voltage_V >> 24U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->voltage_V >> 16U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->voltage_V >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->voltage_V & (0xFFU)) );

  cframe->MsgId = (uint32_t) ENERGY_METER_MEAS_CANID;
  cframe->DLC = (uint8_t) ENERGY_METER_MEAS_DLC;
  cframe->IDE = (uint8_t) ENERGY_METER_MEAS_IDE;
  return ENERGY_METER_MEAS_CANID;
}

#else

uint32_t Pack_ENERGY_METER_MEAS_hytech(ENERGY_METER_MEAS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_MEAS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( ((_m->current_A >> 24U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->current_A >> 16U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->current_A >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->current_A & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->voltage_V >> 24U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->voltage_V >> 16U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->voltage_V >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->voltage_V & (0xFFU)) );

  *_len = (uint8_t) ENERGY_METER_MEAS_DLC;
  *_ide = (uint8_t) ENERGY_METER_MEAS_IDE;
  return ENERGY_METER_MEAS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->drivetrain_traj_torque_lim_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_fl_phys = (sigfloat_t)(HYTECH_drivetrain_traj_torque_lim_fl_ro_fromS(_m->drivetrain_traj_torque_lim_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivetrain_traj_torque_lim_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_fr_phys = (sigfloat_t)(HYTECH_drivetrain_traj_torque_lim_fr_ro_fromS(_m->drivetrain_traj_torque_lim_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivetrain_traj_torque_lim_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_rl_phys = (sigfloat_t)(HYTECH_drivetrain_traj_torque_lim_rl_ro_fromS(_m->drivetrain_traj_torque_lim_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->drivetrain_traj_torque_lim_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_rr_phys = (sigfloat_t)(HYTECH_drivetrain_traj_torque_lim_rr_ro_fromS(_m->drivetrain_traj_torque_lim_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_COMMAND_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_COMMAND_hytech(&_m->mon1, DRIVETRAIN_COMMAND_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_COMMAND_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_COMMAND_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_fl_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_fl_ro_toS(_m->drivetrain_traj_torque_lim_fl_phys);
  _m->drivetrain_traj_torque_lim_fr_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_fr_ro_toS(_m->drivetrain_traj_torque_lim_fr_phys);
  _m->drivetrain_traj_torque_lim_rl_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_rl_ro_toS(_m->drivetrain_traj_torque_lim_rl_phys);
  _m->drivetrain_traj_torque_lim_rr_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_rr_ro_toS(_m->drivetrain_traj_torque_lim_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_COMMAND_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_COMMAND_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_COMMAND_IDE;
  return DRIVETRAIN_COMMAND_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_COMMAND_hytech(DRIVETRAIN_COMMAND_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_COMMAND_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->drivetrain_traj_torque_lim_fl_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_fl_ro_toS(_m->drivetrain_traj_torque_lim_fl_phys);
  _m->drivetrain_traj_torque_lim_fr_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_fr_ro_toS(_m->drivetrain_traj_torque_lim_fr_phys);
  _m->drivetrain_traj_torque_lim_rl_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_rl_ro_toS(_m->drivetrain_traj_torque_lim_rl_phys);
  _m->drivetrain_traj_torque_lim_rr_ro = (int16_t) HYTECH_drivetrain_traj_torque_lim_rr_ro_toS(_m->drivetrain_traj_torque_lim_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->drivetrain_traj_torque_lim_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->drivetrain_traj_torque_lim_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_COMMAND_DLC;
  *_ide = (uint8_t) DRIVETRAIN_COMMAND_IDE;
  return DRIVETRAIN_COMMAND_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_overload_percentage_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_phys = (sigfloat_t)(HYTECH_inverter_overload_percentage_ro_fromS(_m->inverter_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->motor_overload_percentage_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_overload_percentage_phys = (sigfloat_t)(HYTECH_motor_overload_percentage_ro_fromS(_m->motor_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV1_OVERLOAD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV1_OVERLOAD_hytech(&_m->mon1, INV1_OVERLOAD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV1_OVERLOAD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_OVERLOAD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV1_OVERLOAD_CANID;
  cframe->DLC = (uint8_t) INV1_OVERLOAD_DLC;
  cframe->IDE = (uint8_t) INV1_OVERLOAD_IDE;
  return INV1_OVERLOAD_CANID;
}

#else

uint32_t Pack_INV1_OVERLOAD_hytech(INV1_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV1_OVERLOAD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV1_OVERLOAD_DLC;
  *_ide = (uint8_t) INV1_OVERLOAD_IDE;
  return INV1_OVERLOAD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_overload_percentage_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_phys = (sigfloat_t)(HYTECH_inverter_overload_percentage_ro_fromS(_m->inverter_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->motor_overload_percentage_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_overload_percentage_phys = (sigfloat_t)(HYTECH_motor_overload_percentage_ro_fromS(_m->motor_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV2_OVERLOAD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV2_OVERLOAD_hytech(&_m->mon1, INV2_OVERLOAD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV2_OVERLOAD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_OVERLOAD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV2_OVERLOAD_CANID;
  cframe->DLC = (uint8_t) INV2_OVERLOAD_DLC;
  cframe->IDE = (uint8_t) INV2_OVERLOAD_IDE;
  return INV2_OVERLOAD_CANID;
}

#else

uint32_t Pack_INV2_OVERLOAD_hytech(INV2_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV2_OVERLOAD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV2_OVERLOAD_DLC;
  *_ide = (uint8_t) INV2_OVERLOAD_IDE;
  return INV2_OVERLOAD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_overload_percentage_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_phys = (sigfloat_t)(HYTECH_inverter_overload_percentage_ro_fromS(_m->inverter_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->motor_overload_percentage_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_overload_percentage_phys = (sigfloat_t)(HYTECH_motor_overload_percentage_ro_fromS(_m->motor_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV3_OVERLOAD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV3_OVERLOAD_hytech(&_m->mon1, INV3_OVERLOAD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV3_OVERLOAD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_OVERLOAD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV3_OVERLOAD_CANID;
  cframe->DLC = (uint8_t) INV3_OVERLOAD_DLC;
  cframe->IDE = (uint8_t) INV3_OVERLOAD_IDE;
  return INV3_OVERLOAD_CANID;
}

#else

uint32_t Pack_INV3_OVERLOAD_hytech(INV3_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV3_OVERLOAD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV3_OVERLOAD_DLC;
  *_ide = (uint8_t) INV3_OVERLOAD_IDE;
  return INV3_OVERLOAD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->inverter_overload_percentage_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_phys = (sigfloat_t)(HYTECH_inverter_overload_percentage_ro_fromS(_m->inverter_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->motor_overload_percentage_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->motor_overload_percentage_phys = (sigfloat_t)(HYTECH_motor_overload_percentage_ro_fromS(_m->motor_overload_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INV4_OVERLOAD_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INV4_OVERLOAD_hytech(&_m->mon1, INV4_OVERLOAD_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return INV4_OVERLOAD_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_OVERLOAD_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) INV4_OVERLOAD_CANID;
  cframe->DLC = (uint8_t) INV4_OVERLOAD_DLC;
  cframe->IDE = (uint8_t) INV4_OVERLOAD_IDE;
  return INV4_OVERLOAD_CANID;
}

#else

uint32_t Pack_INV4_OVERLOAD_hytech(INV4_OVERLOAD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(INV4_OVERLOAD_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->inverter_overload_percentage_ro = (uint16_t) HYTECH_inverter_overload_percentage_ro_toS(_m->inverter_overload_percentage_phys);
  _m->motor_overload_percentage_ro = (uint16_t) HYTECH_motor_overload_percentage_ro_toS(_m->motor_overload_percentage_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->inverter_overload_percentage_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->inverter_overload_percentage_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->motor_overload_percentage_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->motor_overload_percentage_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) INV4_OVERLOAD_DLC;
  *_ide = (uint8_t) INV4_OVERLOAD_IDE;
  return INV4_OVERLOAD_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->fr_motor_rpm = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->fl_motor_rpm = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->rr_motor_rpm = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
  _m->rl_motor_rpm = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_RPMS_TELEM_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_RPMS_TELEM_hytech(&_m->mon1, DRIVETRAIN_RPMS_TELEM_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_RPMS_TELEM_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_RPMS_TELEM_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->fr_motor_rpm & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->fr_motor_rpm >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->fl_motor_rpm & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->fl_motor_rpm >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->rr_motor_rpm & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->rr_motor_rpm >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->rl_motor_rpm & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->rl_motor_rpm >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_RPMS_TELEM_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_RPMS_TELEM_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_RPMS_TELEM_IDE;
  return DRIVETRAIN_RPMS_TELEM_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_RPMS_TELEM_hytech(DRIVETRAIN_RPMS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_RPMS_TELEM_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->fr_motor_rpm & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->fr_motor_rpm >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->fl_motor_rpm & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->fl_motor_rpm >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->rr_motor_rpm & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->rr_motor_rpm >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->rl_motor_rpm & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->rl_motor_rpm >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_RPMS_TELEM_DLC;
  *_ide = (uint8_t) DRIVETRAIN_RPMS_TELEM_IDE;
  return DRIVETRAIN_RPMS_TELEM_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->mc1_diagnostic_number = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->mc2_diagnostic_number = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
  _m->mc3_diagnostic_number = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
  _m->mc4_diagnostic_number = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_ERR_STATUS_TELEM_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_ERR_STATUS_TELEM_hytech(&_m->mon1, DRIVETRAIN_ERR_STATUS_TELEM_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_ERR_STATUS_TELEM_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_ERR_STATUS_TELEM_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->mc1_diagnostic_number & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->mc1_diagnostic_number >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->mc2_diagnostic_number & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->mc2_diagnostic_number >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->mc3_diagnostic_number & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->mc3_diagnostic_number >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->mc4_diagnostic_number & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->mc4_diagnostic_number >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_ERR_STATUS_TELEM_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_ERR_STATUS_TELEM_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_ERR_STATUS_TELEM_IDE;
  return DRIVETRAIN_ERR_STATUS_TELEM_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_ERR_STATUS_TELEM_hytech(DRIVETRAIN_ERR_STATUS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_ERR_STATUS_TELEM_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->mc1_diagnostic_number & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->mc1_diagnostic_number >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->mc2_diagnostic_number & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->mc2_diagnostic_number >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->mc3_diagnostic_number & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->mc3_diagnostic_number >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->mc4_diagnostic_number & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->mc4_diagnostic_number >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_ERR_STATUS_TELEM_DLC;
  *_ide = (uint8_t) DRIVETRAIN_ERR_STATUS_TELEM_IDE;
  return DRIVETRAIN_ERR_STATUS_TELEM_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->mc1_dc_on = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->mc1_derating_on = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->mc1_error = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->mc1_inverter_on = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->mc1_quit_dc = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->mc1_quit_inverter_on = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->mc1_system_ready = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->mc1_warning = (uint8_t) ( ((_d[0] >> 7U) & (0x01U)) );
  _m->mc2_dc_on = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->mc2_derating_on = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->mc2_error = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->mc2_inverter_on = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->mc2_quit_dc = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->mc2_quit_inverter_on = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );
  _m->mc2_system_ready = (uint8_t) ( ((_d[1] >> 6U) & (0x01U)) );
  _m->mc2_warning = (uint8_t) ( ((_d[1] >> 7U) & (0x01U)) );
  _m->mc3_dc_on = (uint8_t) ( (_d[2] & (0x01U)) );
  _m->mc3_derating_on = (uint8_t) ( ((_d[2] >> 1U) & (0x01U)) );
  _m->mc3_error = (uint8_t) ( ((_d[2] >> 2U) & (0x01U)) );
  _m->mc3_inverter_on = (uint8_t) ( ((_d[2] >> 3U) & (0x01U)) );
  _m->mc3_quit_dc = (uint8_t) ( ((_d[2] >> 4U) & (0x01U)) );
  _m->mc3_quit_inverter_on = (uint8_t) ( ((_d[2] >> 5U) & (0x01U)) );
  _m->mc3_system_ready = (uint8_t) ( ((_d[2] >> 6U) & (0x01U)) );
  _m->mc3_warning = (uint8_t) ( ((_d[2] >> 7U) & (0x01U)) );
  _m->mc4_dc_on = (uint8_t) ( (_d[3] & (0x01U)) );
  _m->mc4_derating_on = (uint8_t) ( ((_d[3] >> 1U) & (0x01U)) );
  _m->mc4_error = (uint8_t) ( ((_d[3] >> 2U) & (0x01U)) );
  _m->mc4_inverter_on = (uint8_t) ( ((_d[3] >> 3U) & (0x01U)) );
  _m->mc4_quit_dc = (uint8_t) ( ((_d[3] >> 4U) & (0x01U)) );
  _m->mc4_quit_inverter_on = (uint8_t) ( ((_d[3] >> 5U) & (0x01U)) );
  _m->mc4_system_ready = (uint8_t) ( ((_d[3] >> 6U) & (0x01U)) );
  _m->mc4_warning = (uint8_t) ( ((_d[3] >> 7U) & (0x01U)) );
  _m->accel_implausible = (uint8_t) ( (_d[4] & (0x01U)) );
  _m->brake_implausible = (uint8_t) ( ((_d[4] >> 1U) & (0x01U)) );
  _m->brake_percent = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->accel_percent = (uint8_t) ( (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_STATUS_TELEM_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_STATUS_TELEM_hytech(&_m->mon1, DRIVETRAIN_STATUS_TELEM_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_STATUS_TELEM_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_STATUS_TELEM_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->mc1_dc_on & (0x01U)) | ((_m->mc1_derating_on & (0x01U)) << 1U) | ((_m->mc1_error & (0x01U)) << 2U) | ((_m->mc1_inverter_on & (0x01U)) << 3U) | ((_m->mc1_quit_dc & (0x01U)) << 4U) | ((_m->mc1_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc1_system_ready & (0x01U)) << 6U) | ((_m->mc1_warning & (0x01U)) << 7U) );
  cframe->Data[1] |= (uint8_t) ( (_m->mc2_dc_on & (0x01U)) | ((_m->mc2_derating_on & (0x01U)) << 1U) | ((_m->mc2_error & (0x01U)) << 2U) | ((_m->mc2_inverter_on & (0x01U)) << 3U) | ((_m->mc2_quit_dc & (0x01U)) << 4U) | ((_m->mc2_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc2_system_ready & (0x01U)) << 6U) | ((_m->mc2_warning & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( (_m->mc3_dc_on & (0x01U)) | ((_m->mc3_derating_on & (0x01U)) << 1U) | ((_m->mc3_error & (0x01U)) << 2U) | ((_m->mc3_inverter_on & (0x01U)) << 3U) | ((_m->mc3_quit_dc & (0x01U)) << 4U) | ((_m->mc3_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc3_system_ready & (0x01U)) << 6U) | ((_m->mc3_warning & (0x01U)) << 7U) );
  cframe->Data[3] |= (uint8_t) ( (_m->mc4_dc_on & (0x01U)) | ((_m->mc4_derating_on & (0x01U)) << 1U) | ((_m->mc4_error & (0x01U)) << 2U) | ((_m->mc4_inverter_on & (0x01U)) << 3U) | ((_m->mc4_quit_dc & (0x01U)) << 4U) | ((_m->mc4_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc4_system_ready & (0x01U)) << 6U) | ((_m->mc4_warning & (0x01U)) << 7U) );
  cframe->Data[4] |= (uint8_t) ( (_m->accel_implausible & (0x01U)) | ((_m->brake_implausible & (0x01U)) << 1U) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_percent & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->accel_percent & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_STATUS_TELEM_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_STATUS_TELEM_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_STATUS_TELEM_IDE;
  return DRIVETRAIN_STATUS_TELEM_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_STATUS_TELEM_hytech(DRIVETRAIN_STATUS_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_STATUS_TELEM_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->mc1_dc_on & (0x01U)) | ((_m->mc1_derating_on & (0x01U)) << 1U) | ((_m->mc1_error & (0x01U)) << 2U) | ((_m->mc1_inverter_on & (0x01U)) << 3U) | ((_m->mc1_quit_dc & (0x01U)) << 4U) | ((_m->mc1_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc1_system_ready & (0x01U)) << 6U) | ((_m->mc1_warning & (0x01U)) << 7U) );
  _d[1] |= (uint8_t) ( (_m->mc2_dc_on & (0x01U)) | ((_m->mc2_derating_on & (0x01U)) << 1U) | ((_m->mc2_error & (0x01U)) << 2U) | ((_m->mc2_inverter_on & (0x01U)) << 3U) | ((_m->mc2_quit_dc & (0x01U)) << 4U) | ((_m->mc2_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc2_system_ready & (0x01U)) << 6U) | ((_m->mc2_warning & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( (_m->mc3_dc_on & (0x01U)) | ((_m->mc3_derating_on & (0x01U)) << 1U) | ((_m->mc3_error & (0x01U)) << 2U) | ((_m->mc3_inverter_on & (0x01U)) << 3U) | ((_m->mc3_quit_dc & (0x01U)) << 4U) | ((_m->mc3_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc3_system_ready & (0x01U)) << 6U) | ((_m->mc3_warning & (0x01U)) << 7U) );
  _d[3] |= (uint8_t) ( (_m->mc4_dc_on & (0x01U)) | ((_m->mc4_derating_on & (0x01U)) << 1U) | ((_m->mc4_error & (0x01U)) << 2U) | ((_m->mc4_inverter_on & (0x01U)) << 3U) | ((_m->mc4_quit_dc & (0x01U)) << 4U) | ((_m->mc4_quit_inverter_on & (0x01U)) << 5U) | ((_m->mc4_system_ready & (0x01U)) << 6U) | ((_m->mc4_warning & (0x01U)) << 7U) );
  _d[4] |= (uint8_t) ( (_m->accel_implausible & (0x01U)) | ((_m->brake_implausible & (0x01U)) << 1U) );
  _d[5] |= (uint8_t) ( (_m->brake_percent & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->accel_percent & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_STATUS_TELEM_DLC;
  *_ide = (uint8_t) DRIVETRAIN_STATUS_TELEM_IDE;
  return DRIVETRAIN_STATUS_TELEM_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->fl_motor_torque = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->fr_motor_torque = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->rl_motor_torque = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
  _m->rr_motor_torque = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_TORQUE_TELEM_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_TORQUE_TELEM_hytech(&_m->mon1, DRIVETRAIN_TORQUE_TELEM_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_TORQUE_TELEM_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_TORQUE_TELEM_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->fl_motor_torque & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->fl_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->fr_motor_torque & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->fr_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->rl_motor_torque & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->rl_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->rr_motor_torque & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->rr_motor_torque >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_TORQUE_TELEM_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_TORQUE_TELEM_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_TORQUE_TELEM_IDE;
  return DRIVETRAIN_TORQUE_TELEM_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_TORQUE_TELEM_hytech(DRIVETRAIN_TORQUE_TELEM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_TORQUE_TELEM_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->fl_motor_torque & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->fl_motor_torque >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->fr_motor_torque & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->fr_motor_torque >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->rl_motor_torque & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->rl_motor_torque >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->rr_motor_torque & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->rr_motor_torque >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_TORQUE_TELEM_DLC;
  *_ide = (uint8_t) DRIVETRAIN_TORQUE_TELEM_IDE;
  return DRIVETRAIN_TORQUE_TELEM_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->fl_motor_torque = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
  _m->fr_motor_torque = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
  _m->rl_motor_torque = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
  _m->rr_motor_torque = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(&_m->mon1, DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->fl_motor_torque & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->fl_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->fr_motor_torque & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->fr_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->rl_motor_torque & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->rl_motor_torque >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->rr_motor_torque & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->rr_motor_torque >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID;
  cframe->DLC = (uint8_t) DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC;
  cframe->IDE = (uint8_t) DRIVETRAIN_FILTER_OUT_TORQUE_TEL_IDE;
  return DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID;
}

#else

uint32_t Pack_DRIVETRAIN_FILTER_OUT_TORQUE_TEL_hytech(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->fl_motor_torque & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->fl_motor_torque >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->fr_motor_torque & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->fr_motor_torque >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->rl_motor_torque & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->rl_motor_torque >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->rr_motor_torque & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->rr_motor_torque >> 8U) & (0xFFU)) );

  *_len = (uint8_t) DRIVETRAIN_FILTER_OUT_TORQUE_TEL_DLC;
  *_ide = (uint8_t) DRIVETRAIN_FILTER_OUT_TORQUE_TEL_IDE;
  return DRIVETRAIN_FILTER_OUT_TORQUE_TEL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CAR_STATES_hytech(CAR_STATES_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehicle_state = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->drivetrain_state = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->drivebrain_in_control = (uint8_t) ( (_d[2] & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CAR_STATES_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CAR_STATES_hytech(&_m->mon1, CAR_STATES_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CAR_STATES_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CAR_STATES_hytech(CAR_STATES_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CAR_STATES_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->vehicle_state & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->drivetrain_state & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->drivebrain_in_control & (0x01U)) );

  cframe->MsgId = (uint32_t) CAR_STATES_CANID;
  cframe->DLC = (uint8_t) CAR_STATES_DLC;
  cframe->IDE = (uint8_t) CAR_STATES_IDE;
  return CAR_STATES_CANID;
}

#else

uint32_t Pack_CAR_STATES_hytech(CAR_STATES_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CAR_STATES_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->vehicle_state & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->drivetrain_state & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->drivebrain_in_control & (0x01U)) );

  *_len = (uint8_t) CAR_STATES_DLC;
  *_ide = (uint8_t) CAR_STATES_IDE;
  return CAR_STATES_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vn_gps_status = (uint8_t) ( (_d[0] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DRIVEBRAIN_STATE_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DRIVEBRAIN_STATE_DATA_hytech(&_m->mon1, DRIVEBRAIN_STATE_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DRIVEBRAIN_STATE_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_STATE_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->vn_gps_status & (0xFFU)) );

  cframe->MsgId = (uint32_t) DRIVEBRAIN_STATE_DATA_CANID;
  cframe->DLC = (uint8_t) DRIVEBRAIN_STATE_DATA_DLC;
  cframe->IDE = (uint8_t) DRIVEBRAIN_STATE_DATA_IDE;
  return DRIVEBRAIN_STATE_DATA_CANID;
}

#else

uint32_t Pack_DRIVEBRAIN_STATE_DATA_hytech(DRIVEBRAIN_STATE_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DRIVEBRAIN_STATE_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->vn_gps_status & (0xFFU)) );

  *_len = (uint8_t) DRIVEBRAIN_STATE_DATA_DLC;
  *_ide = (uint8_t) DRIVEBRAIN_STATE_DATA_IDE;
  return DRIVEBRAIN_STATE_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASH_INPUT_hytech(DASH_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->led_dimmer_button = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->preset_button = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->motor_controller_cycle_button = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->mode_button = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->start_button = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->data_button_is_pressed = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->left_shifter_button = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->right_shifter_button = (uint8_t) ( ((_d[0] >> 7U) & (0x01U)) );
  _m->dash_dial_mode = (uint8_t) ( (_d[1] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASH_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASH_INPUT_hytech(&_m->mon1, DASH_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DASH_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DASH_INPUT_hytech(DASH_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASH_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->led_dimmer_button & (0x01U)) | ((_m->preset_button & (0x01U)) << 1U) | ((_m->motor_controller_cycle_button & (0x01U)) << 2U) | ((_m->mode_button & (0x01U)) << 3U) | ((_m->start_button & (0x01U)) << 4U) | ((_m->data_button_is_pressed & (0x01U)) << 5U) | ((_m->left_shifter_button & (0x01U)) << 6U) | ((_m->right_shifter_button & (0x01U)) << 7U) );
  cframe->Data[1] |= (uint8_t) ( (_m->dash_dial_mode & (0xFFU)) );

  cframe->MsgId = (uint32_t) DASH_INPUT_CANID;
  cframe->DLC = (uint8_t) DASH_INPUT_DLC;
  cframe->IDE = (uint8_t) DASH_INPUT_IDE;
  return DASH_INPUT_CANID;
}

#else

uint32_t Pack_DASH_INPUT_hytech(DASH_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASH_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->led_dimmer_button & (0x01U)) | ((_m->preset_button & (0x01U)) << 1U) | ((_m->motor_controller_cycle_button & (0x01U)) << 2U) | ((_m->mode_button & (0x01U)) << 3U) | ((_m->start_button & (0x01U)) << 4U) | ((_m->data_button_is_pressed & (0x01U)) << 5U) | ((_m->left_shifter_button & (0x01U)) << 6U) | ((_m->right_shifter_button & (0x01U)) << 7U) );
  _d[1] |= (uint8_t) ( (_m->dash_dial_mode & (0xFFU)) );

  *_len = (uint8_t) DASH_INPUT_DLC;
  *_ide = (uint8_t) DASH_INPUT_IDE;
  return DASH_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_4_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_phys = (sigfloat_t)(HYTECH_aero_channel_4_ro_fromS(_m->aero_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_5_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_5_phys = (sigfloat_t)(HYTECH_aero_channel_5_ro_fromS(_m->aero_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_6_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_6_phys = (sigfloat_t)(HYTECH_aero_channel_6_ro_fromS(_m->aero_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_7_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_7_phys = (sigfloat_t)(HYTECH_aero_channel_7_ro_fromS(_m->aero_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_22_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_22_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_22_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_22_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_22_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_22_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_22_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_22_IDE;
  return AERO_PRESSURE_SENSOR_22_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_22_hytech(AERO_PRESSURE_SENSOR_22_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_22_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_22_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_22_IDE;
  return AERO_PRESSURE_SENSOR_22_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_4_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_phys = (sigfloat_t)(HYTECH_aero_channel_4_ro_fromS(_m->aero_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_5_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_5_phys = (sigfloat_t)(HYTECH_aero_channel_5_ro_fromS(_m->aero_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_6_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_6_phys = (sigfloat_t)(HYTECH_aero_channel_6_ro_fromS(_m->aero_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_7_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_7_phys = (sigfloat_t)(HYTECH_aero_channel_7_ro_fromS(_m->aero_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_32_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_32_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_32_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_32_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_32_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_32_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_32_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_32_IDE;
  return AERO_PRESSURE_SENSOR_32_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_32_hytech(AERO_PRESSURE_SENSOR_32_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_32_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_32_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_32_IDE;
  return AERO_PRESSURE_SENSOR_32_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_4_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_phys = (sigfloat_t)(HYTECH_aero_channel_4_ro_fromS(_m->aero_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_5_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_5_phys = (sigfloat_t)(HYTECH_aero_channel_5_ro_fromS(_m->aero_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_6_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_6_phys = (sigfloat_t)(HYTECH_aero_channel_6_ro_fromS(_m->aero_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_7_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_7_phys = (sigfloat_t)(HYTECH_aero_channel_7_ro_fromS(_m->aero_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_42_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_42_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_42_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_42_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_42_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_42_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_42_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_42_IDE;
  return AERO_PRESSURE_SENSOR_42_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_42_hytech(AERO_PRESSURE_SENSOR_42_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_42_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_42_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_42_IDE;
  return AERO_PRESSURE_SENSOR_42_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_0_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_phys = (sigfloat_t)(HYTECH_aero_channel_0_ro_fromS(_m->aero_channel_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_1_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_1_phys = (sigfloat_t)(HYTECH_aero_channel_1_ro_fromS(_m->aero_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_2_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_2_phys = (sigfloat_t)(HYTECH_aero_channel_2_ro_fromS(_m->aero_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_3_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_3_phys = (sigfloat_t)(HYTECH_aero_channel_3_ro_fromS(_m->aero_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_11_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_11_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_11_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_11_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_11_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_11_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_11_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_11_IDE;
  return AERO_PRESSURE_SENSOR_11_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_11_hytech(AERO_PRESSURE_SENSOR_11_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_11_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_11_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_11_IDE;
  return AERO_PRESSURE_SENSOR_11_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_4_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_phys = (sigfloat_t)(HYTECH_aero_channel_4_ro_fromS(_m->aero_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_5_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_5_phys = (sigfloat_t)(HYTECH_aero_channel_5_ro_fromS(_m->aero_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_6_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_6_phys = (sigfloat_t)(HYTECH_aero_channel_6_ro_fromS(_m->aero_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_7_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_7_phys = (sigfloat_t)(HYTECH_aero_channel_7_ro_fromS(_m->aero_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_12_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_12_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_12_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_12_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_12_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_12_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_12_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_12_IDE;
  return AERO_PRESSURE_SENSOR_12_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_12_hytech(AERO_PRESSURE_SENSOR_12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_12_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_4_ro = (uint16_t) HYTECH_aero_channel_4_ro_toS(_m->aero_channel_4_phys);
  _m->aero_channel_5_ro = (uint16_t) HYTECH_aero_channel_5_ro_toS(_m->aero_channel_5_phys);
  _m->aero_channel_6_ro = (uint16_t) HYTECH_aero_channel_6_ro_toS(_m->aero_channel_6_phys);
  _m->aero_channel_7_ro = (uint16_t) HYTECH_aero_channel_7_ro_toS(_m->aero_channel_7_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_4_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_4_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_5_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_5_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_6_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_6_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_7_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_7_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_12_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_12_IDE;
  return AERO_PRESSURE_SENSOR_12_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_EM_STATUS_hytech(EM_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->voltage_gain = (uint8_t) ( (_d[0] & (0x0FU)) );
  _m->current_gain = (uint8_t) ( ((_d[0] >> 4U) & (0x0FU)) );
  _m->overvoltage_error = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->overpower_error = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->logging = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < EM_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_EM_STATUS_hytech(&_m->mon1, EM_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return EM_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_EM_STATUS_hytech(EM_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(EM_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->voltage_gain & (0x0FU)) | ((_m->current_gain & (0x0FU)) << 4U) );
  cframe->Data[1] |= (uint8_t) ( (_m->overvoltage_error & (0x01U)) | ((_m->overpower_error & (0x01U)) << 1U) | ((_m->logging & (0x01U)) << 2U) );

  cframe->MsgId = (uint32_t) EM_STATUS_CANID;
  cframe->DLC = (uint8_t) EM_STATUS_DLC;
  cframe->IDE = (uint8_t) EM_STATUS_IDE;
  return EM_STATUS_CANID;
}

#else

uint32_t Pack_EM_STATUS_hytech(EM_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(EM_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->voltage_gain & (0x0FU)) | ((_m->current_gain & (0x0FU)) << 4U) );
  _d[1] |= (uint8_t) ( (_m->overvoltage_error & (0x01U)) | ((_m->overpower_error & (0x01U)) << 1U) | ((_m->logging & (0x01U)) << 2U) );

  *_len = (uint8_t) EM_STATUS_DLC;
  *_ide = (uint8_t) EM_STATUS_IDE;
  return EM_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->current_shunt_read_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->current_shunt_read_phys = (sigfloat_t)(HYTECH_current_shunt_read_ro_fromS(_m->current_shunt_read_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->pack_filtered_read_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->pack_filtered_read_phys = (sigfloat_t)(HYTECH_pack_filtered_read_ro_fromS(_m->pack_filtered_read_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->ts_out_filtered_read_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->ts_out_filtered_read_phys = (sigfloat_t)(HYTECH_ts_out_filtered_read_ro_fromS(_m->ts_out_filtered_read_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACU_SHUNT_MEASUREMENTS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACU_SHUNT_MEASUREMENTS_hytech(&_m->mon1, ACU_SHUNT_MEASUREMENTS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return ACU_SHUNT_MEASUREMENTS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ACU_SHUNT_MEASUREMENTS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->current_shunt_read_ro = (uint16_t) HYTECH_current_shunt_read_ro_toS(_m->current_shunt_read_phys);
  _m->pack_filtered_read_ro = (uint16_t) HYTECH_pack_filtered_read_ro_toS(_m->pack_filtered_read_phys);
  _m->ts_out_filtered_read_ro = (uint16_t) HYTECH_ts_out_filtered_read_ro_toS(_m->ts_out_filtered_read_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->current_shunt_read_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->current_shunt_read_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->pack_filtered_read_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->pack_filtered_read_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->ts_out_filtered_read_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->ts_out_filtered_read_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) ACU_SHUNT_MEASUREMENTS_CANID;
  cframe->DLC = (uint8_t) ACU_SHUNT_MEASUREMENTS_DLC;
  cframe->IDE = (uint8_t) ACU_SHUNT_MEASUREMENTS_IDE;
  return ACU_SHUNT_MEASUREMENTS_CANID;
}

#else

uint32_t Pack_ACU_SHUNT_MEASUREMENTS_hytech(ACU_SHUNT_MEASUREMENTS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ACU_SHUNT_MEASUREMENTS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->current_shunt_read_ro = (uint16_t) HYTECH_current_shunt_read_ro_toS(_m->current_shunt_read_phys);
  _m->pack_filtered_read_ro = (uint16_t) HYTECH_pack_filtered_read_ro_toS(_m->pack_filtered_read_phys);
  _m->ts_out_filtered_read_ro = (uint16_t) HYTECH_ts_out_filtered_read_ro_toS(_m->ts_out_filtered_read_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->current_shunt_read_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->current_shunt_read_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->pack_filtered_read_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->pack_filtered_read_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->ts_out_filtered_read_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->ts_out_filtered_read_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) ACU_SHUNT_MEASUREMENTS_DLC;
  *_ide = (uint8_t) ACU_SHUNT_MEASUREMENTS_IDE;
  return ACU_SHUNT_MEASUREMENTS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->em_violation = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->em_logging = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->em_energy_w_hr = (uint32_t) ( ((_d[1] & (0xFFU)) << 24U) | ((_d[2] & (0xFFU)) << 16U) | ((_d[3] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENERGY_METER_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENERGY_METER_STATUS_hytech(&_m->mon1, ENERGY_METER_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return ENERGY_METER_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->em_violation & (0x01U)) | ((_m->em_logging & (0x01U)) << 1U) );
  cframe->Data[1] |= (uint8_t) ( ((_m->em_energy_w_hr >> 24U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->em_energy_w_hr >> 16U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->em_energy_w_hr >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->em_energy_w_hr & (0xFFU)) );

  cframe->MsgId = (uint32_t) ENERGY_METER_STATUS_CANID;
  cframe->DLC = (uint8_t) ENERGY_METER_STATUS_DLC;
  cframe->IDE = (uint8_t) ENERGY_METER_STATUS_IDE;
  return ENERGY_METER_STATUS_CANID;
}

#else

uint32_t Pack_ENERGY_METER_STATUS_hytech(ENERGY_METER_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->em_violation & (0x01U)) | ((_m->em_logging & (0x01U)) << 1U) );
  _d[1] |= (uint8_t) ( ((_m->em_energy_w_hr >> 24U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->em_energy_w_hr >> 16U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->em_energy_w_hr >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->em_energy_w_hr & (0xFFU)) );

  *_len = (uint8_t) ENERGY_METER_STATUS_DLC;
  *_ide = (uint8_t) ENERGY_METER_STATUS_IDE;
  return ENERGY_METER_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->charge_percentage_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->charge_percentage_phys = (sigfloat_t)(HYTECH_charge_percentage_ro_fromS(_m->charge_percentage_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->min_cell_voltage_est_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->min_cell_voltage_est_phys = (sigfloat_t)(HYTECH_min_cell_voltage_est_ro_fromS(_m->min_cell_voltage_est_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->charge_coulombs_ro = (uint32_t) ( ((_d[4] & (0xFFU)) << 24U) | ((_d[5] & (0xFFU)) << 16U) | ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->charge_coulombs_phys = (sigfloat_t)(HYTECH_charge_coulombs_ro_fromS(_m->charge_coulombs_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STATE_OF_CHARGE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STATE_OF_CHARGE_hytech(&_m->mon1, STATE_OF_CHARGE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return STATE_OF_CHARGE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(STATE_OF_CHARGE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->charge_percentage_ro = (uint16_t) HYTECH_charge_percentage_ro_toS(_m->charge_percentage_phys);
  _m->min_cell_voltage_est_ro = (uint16_t) HYTECH_min_cell_voltage_est_ro_toS(_m->min_cell_voltage_est_phys);
  _m->charge_coulombs_ro = (uint32_t) HYTECH_charge_coulombs_ro_toS(_m->charge_coulombs_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->charge_percentage_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->charge_percentage_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->min_cell_voltage_est_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->min_cell_voltage_est_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 24U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 16U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->charge_coulombs_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) STATE_OF_CHARGE_CANID;
  cframe->DLC = (uint8_t) STATE_OF_CHARGE_DLC;
  cframe->IDE = (uint8_t) STATE_OF_CHARGE_IDE;
  return STATE_OF_CHARGE_CANID;
}

#else

uint32_t Pack_STATE_OF_CHARGE_hytech(STATE_OF_CHARGE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(STATE_OF_CHARGE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->charge_percentage_ro = (uint16_t) HYTECH_charge_percentage_ro_toS(_m->charge_percentage_phys);
  _m->min_cell_voltage_est_ro = (uint16_t) HYTECH_min_cell_voltage_est_ro_toS(_m->min_cell_voltage_est_phys);
  _m->charge_coulombs_ro = (uint32_t) HYTECH_charge_coulombs_ro_toS(_m->charge_coulombs_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->charge_percentage_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->charge_percentage_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->min_cell_voltage_est_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->min_cell_voltage_est_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 24U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 16U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->charge_coulombs_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->charge_coulombs_ro & (0xFFU)) );

  *_len = (uint8_t) STATE_OF_CHARGE_DLC;
  *_ide = (uint8_t) STATE_OF_CHARGE_IDE;
  return STATE_OF_CHARGE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_STEERING_DATA_hytech(STEERING_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->steering_analog_raw = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->steering_digital_raw = (uint32_t) ( ((_d[5] & (0xFFU)) << 24U) | ((_d[4] & (0xFFU)) << 16U) | ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < STEERING_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_STEERING_DATA_hytech(&_m->mon1, STEERING_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return STEERING_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_STEERING_DATA_hytech(STEERING_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(STEERING_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->steering_analog_raw & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->steering_analog_raw >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->steering_digital_raw & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->steering_digital_raw >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->steering_digital_raw >> 16U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->steering_digital_raw >> 24U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) STEERING_DATA_CANID;
  cframe->DLC = (uint8_t) STEERING_DATA_DLC;
  cframe->IDE = (uint8_t) STEERING_DATA_IDE;
  return STEERING_DATA_CANID;
}

#else

uint32_t Pack_STEERING_DATA_hytech(STEERING_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(STEERING_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->steering_analog_raw & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->steering_analog_raw >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->steering_digital_raw & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->steering_digital_raw >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->steering_digital_raw >> 16U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->steering_digital_raw >> 24U) & (0xFFU)) );

  *_len = (uint8_t) STEERING_DATA_DLC;
  *_ide = (uint8_t) STEERING_DATA_IDE;
  return STEERING_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_SN = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->LF_TTPMS_BAT_V = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->LF_TTPMS_P = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
  _m->LF_TTPMS_P_GAUGE = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_1_hytech(&_m->mon1, LF_TTPMS_1_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_1_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_1_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_SN >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_SN & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_BAT_V & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_P >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_P & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_P_GAUGE & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_1_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_1_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_1_IDE;
  return LF_TTPMS_1_CANID;
}

#else

uint32_t Pack_LF_TTPMS_1_hytech(LF_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_1_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_SN >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_SN & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_BAT_V & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_P >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_P & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_P_GAUGE & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_1_DLC;
  *_ide = (uint8_t) LF_TTPMS_1_IDE;
  return LF_TTPMS_1_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_T1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T1_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T1_ro_fromS(_m->LF_TTPMS_T1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T2_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T2_ro_fromS(_m->LF_TTPMS_T2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T3_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T3_ro_fromS(_m->LF_TTPMS_T3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T4_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T4_ro_fromS(_m->LF_TTPMS_T4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_2_hytech(&_m->mon1, LF_TTPMS_2_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_2_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_2_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T1_ro = (uint16_t) HYTECH_LF_TTPMS_T1_ro_toS(_m->LF_TTPMS_T1_phys);
  _m->LF_TTPMS_T2_ro = (uint16_t) HYTECH_LF_TTPMS_T2_ro_toS(_m->LF_TTPMS_T2_phys);
  _m->LF_TTPMS_T3_ro = (uint16_t) HYTECH_LF_TTPMS_T3_ro_toS(_m->LF_TTPMS_T3_phys);
  _m->LF_TTPMS_T4_ro = (uint16_t) HYTECH_LF_TTPMS_T4_ro_toS(_m->LF_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_T1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_T2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_T3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_T4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_2_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_2_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_2_IDE;
  return LF_TTPMS_2_CANID;
}

#else

uint32_t Pack_LF_TTPMS_2_hytech(LF_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_2_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T1_ro = (uint16_t) HYTECH_LF_TTPMS_T1_ro_toS(_m->LF_TTPMS_T1_phys);
  _m->LF_TTPMS_T2_ro = (uint16_t) HYTECH_LF_TTPMS_T2_ro_toS(_m->LF_TTPMS_T2_phys);
  _m->LF_TTPMS_T3_ro = (uint16_t) HYTECH_LF_TTPMS_T3_ro_toS(_m->LF_TTPMS_T3_phys);
  _m->LF_TTPMS_T4_ro = (uint16_t) HYTECH_LF_TTPMS_T4_ro_toS(_m->LF_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_T1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_T2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_T3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_T4_ro & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_2_DLC;
  *_ide = (uint8_t) LF_TTPMS_2_IDE;
  return LF_TTPMS_2_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_T5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T5_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T5_ro_fromS(_m->LF_TTPMS_T5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T6_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T6_ro_fromS(_m->LF_TTPMS_T6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T7_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T7_ro_fromS(_m->LF_TTPMS_T7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T8_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T8_ro_fromS(_m->LF_TTPMS_T8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_3_hytech(&_m->mon1, LF_TTPMS_3_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_3_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_3_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T5_ro = (uint16_t) HYTECH_LF_TTPMS_T5_ro_toS(_m->LF_TTPMS_T5_phys);
  _m->LF_TTPMS_T6_ro = (uint16_t) HYTECH_LF_TTPMS_T6_ro_toS(_m->LF_TTPMS_T6_phys);
  _m->LF_TTPMS_T7_ro = (uint16_t) HYTECH_LF_TTPMS_T7_ro_toS(_m->LF_TTPMS_T7_phys);
  _m->LF_TTPMS_T8_ro = (uint16_t) HYTECH_LF_TTPMS_T8_ro_toS(_m->LF_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_T5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_T6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_T7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_T8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_3_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_3_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_3_IDE;
  return LF_TTPMS_3_CANID;
}

#else

uint32_t Pack_LF_TTPMS_3_hytech(LF_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_3_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T5_ro = (uint16_t) HYTECH_LF_TTPMS_T5_ro_toS(_m->LF_TTPMS_T5_phys);
  _m->LF_TTPMS_T6_ro = (uint16_t) HYTECH_LF_TTPMS_T6_ro_toS(_m->LF_TTPMS_T6_phys);
  _m->LF_TTPMS_T7_ro = (uint16_t) HYTECH_LF_TTPMS_T7_ro_toS(_m->LF_TTPMS_T7_phys);
  _m->LF_TTPMS_T8_ro = (uint16_t) HYTECH_LF_TTPMS_T8_ro_toS(_m->LF_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_T5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_T6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_T7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_T8_ro & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_3_DLC;
  *_ide = (uint8_t) LF_TTPMS_3_IDE;
  return LF_TTPMS_3_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_T9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T9_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T9_ro_fromS(_m->LF_TTPMS_T9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T10_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T10_ro_fromS(_m->LF_TTPMS_T10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T11_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T11_ro_fromS(_m->LF_TTPMS_T11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T12_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T12_ro_fromS(_m->LF_TTPMS_T12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_4_hytech(&_m->mon1, LF_TTPMS_4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T9_ro = (uint16_t) HYTECH_LF_TTPMS_T9_ro_toS(_m->LF_TTPMS_T9_phys);
  _m->LF_TTPMS_T10_ro = (uint16_t) HYTECH_LF_TTPMS_T10_ro_toS(_m->LF_TTPMS_T10_phys);
  _m->LF_TTPMS_T11_ro = (uint16_t) HYTECH_LF_TTPMS_T11_ro_toS(_m->LF_TTPMS_T11_phys);
  _m->LF_TTPMS_T12_ro = (uint16_t) HYTECH_LF_TTPMS_T12_ro_toS(_m->LF_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_T9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_T10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_T11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_T12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_4_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_4_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_4_IDE;
  return LF_TTPMS_4_CANID;
}

#else

uint32_t Pack_LF_TTPMS_4_hytech(LF_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T9_ro = (uint16_t) HYTECH_LF_TTPMS_T9_ro_toS(_m->LF_TTPMS_T9_phys);
  _m->LF_TTPMS_T10_ro = (uint16_t) HYTECH_LF_TTPMS_T10_ro_toS(_m->LF_TTPMS_T10_phys);
  _m->LF_TTPMS_T11_ro = (uint16_t) HYTECH_LF_TTPMS_T11_ro_toS(_m->LF_TTPMS_T11_phys);
  _m->LF_TTPMS_T12_ro = (uint16_t) HYTECH_LF_TTPMS_T12_ro_toS(_m->LF_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_T9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_T10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_T11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_T12_ro & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_4_DLC;
  *_ide = (uint8_t) LF_TTPMS_4_IDE;
  return LF_TTPMS_4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_T13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T13_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T13_ro_fromS(_m->LF_TTPMS_T13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T14_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T14_ro_fromS(_m->LF_TTPMS_T14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T15_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T15_ro_fromS(_m->LF_TTPMS_T15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_T16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T16_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T16_ro_fromS(_m->LF_TTPMS_T16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_5_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_5_hytech(&_m->mon1, LF_TTPMS_5_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_5_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_5_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T13_ro = (uint16_t) HYTECH_LF_TTPMS_T13_ro_toS(_m->LF_TTPMS_T13_phys);
  _m->LF_TTPMS_T14_ro = (uint16_t) HYTECH_LF_TTPMS_T14_ro_toS(_m->LF_TTPMS_T14_phys);
  _m->LF_TTPMS_T15_ro = (uint16_t) HYTECH_LF_TTPMS_T15_ro_toS(_m->LF_TTPMS_T15_phys);
  _m->LF_TTPMS_T16_ro = (uint16_t) HYTECH_LF_TTPMS_T16_ro_toS(_m->LF_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_T13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_T14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_T15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_T16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_5_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_5_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_5_IDE;
  return LF_TTPMS_5_CANID;
}

#else

uint32_t Pack_LF_TTPMS_5_hytech(LF_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_5_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T13_ro = (uint16_t) HYTECH_LF_TTPMS_T13_ro_toS(_m->LF_TTPMS_T13_phys);
  _m->LF_TTPMS_T14_ro = (uint16_t) HYTECH_LF_TTPMS_T14_ro_toS(_m->LF_TTPMS_T14_phys);
  _m->LF_TTPMS_T15_ro = (uint16_t) HYTECH_LF_TTPMS_T15_ro_toS(_m->LF_TTPMS_T15_phys);
  _m->LF_TTPMS_T16_ro = (uint16_t) HYTECH_LF_TTPMS_T16_ro_toS(_m->LF_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_T13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_T14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_T15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_T16_ro & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_5_DLC;
  *_ide = (uint8_t) LF_TTPMS_5_IDE;
  return LF_TTPMS_5_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LF_TTPMS_TC = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->LF_TTPMS_RSSI = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
  _m->LF_TTPMS_T_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T_phys = (sigfloat_t)(HYTECH_LF_TTPMS_T_ro_fromS(_m->LF_TTPMS_T_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LF_TTPMS_NODE_ID = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LF_TTPMS_6_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LF_TTPMS_6_hytech(&_m->mon1, LF_TTPMS_6_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LF_TTPMS_6_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_6_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T_ro = (uint16_t) HYTECH_LF_TTPMS_T_ro_toS(_m->LF_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LF_TTPMS_TC >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LF_TTPMS_TC & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LF_TTPMS_RSSI >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LF_TTPMS_RSSI & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LF_TTPMS_T_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LF_TTPMS_T_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LF_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LF_TTPMS_NODE_ID & (0xFFU)) );

  cframe->MsgId = (uint32_t) LF_TTPMS_6_CANID;
  cframe->DLC = (uint8_t) LF_TTPMS_6_DLC;
  cframe->IDE = (uint8_t) LF_TTPMS_6_IDE;
  return LF_TTPMS_6_CANID;
}

#else

uint32_t Pack_LF_TTPMS_6_hytech(LF_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LF_TTPMS_6_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LF_TTPMS_T_ro = (uint16_t) HYTECH_LF_TTPMS_T_ro_toS(_m->LF_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LF_TTPMS_TC >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LF_TTPMS_TC & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LF_TTPMS_RSSI >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LF_TTPMS_RSSI & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LF_TTPMS_T_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LF_TTPMS_T_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LF_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LF_TTPMS_NODE_ID & (0xFFU)) );

  *_len = (uint8_t) LF_TTPMS_6_DLC;
  *_ide = (uint8_t) LF_TTPMS_6_IDE;
  return LF_TTPMS_6_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_SN = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->LR_TTPMS_BAT_V = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->LR_TTPMS_P = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
  _m->LR_TTPMS_P_GAUGE = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_1_hytech(&_m->mon1, LR_TTPMS_1_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_1_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_1_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_SN >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_SN & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_BAT_V & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_P >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_P & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_P_GAUGE & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_1_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_1_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_1_IDE;
  return LR_TTPMS_1_CANID;
}

#else

uint32_t Pack_LR_TTPMS_1_hytech(LR_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_1_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_SN >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_SN & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_BAT_V & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_P >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_P & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_P_GAUGE & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_1_DLC;
  *_ide = (uint8_t) LR_TTPMS_1_IDE;
  return LR_TTPMS_1_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_T1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T1_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T1_ro_fromS(_m->LR_TTPMS_T1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T2_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T2_ro_fromS(_m->LR_TTPMS_T2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T3_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T3_ro_fromS(_m->LR_TTPMS_T3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T4_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T4_ro_fromS(_m->LR_TTPMS_T4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_2_hytech(&_m->mon1, LR_TTPMS_2_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_2_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_2_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T1_ro = (uint16_t) HYTECH_LR_TTPMS_T1_ro_toS(_m->LR_TTPMS_T1_phys);
  _m->LR_TTPMS_T2_ro = (uint16_t) HYTECH_LR_TTPMS_T2_ro_toS(_m->LR_TTPMS_T2_phys);
  _m->LR_TTPMS_T3_ro = (uint16_t) HYTECH_LR_TTPMS_T3_ro_toS(_m->LR_TTPMS_T3_phys);
  _m->LR_TTPMS_T4_ro = (uint16_t) HYTECH_LR_TTPMS_T4_ro_toS(_m->LR_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_T1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_T2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_T3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_T4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_2_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_2_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_2_IDE;
  return LR_TTPMS_2_CANID;
}

#else

uint32_t Pack_LR_TTPMS_2_hytech(LR_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_2_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T1_ro = (uint16_t) HYTECH_LR_TTPMS_T1_ro_toS(_m->LR_TTPMS_T1_phys);
  _m->LR_TTPMS_T2_ro = (uint16_t) HYTECH_LR_TTPMS_T2_ro_toS(_m->LR_TTPMS_T2_phys);
  _m->LR_TTPMS_T3_ro = (uint16_t) HYTECH_LR_TTPMS_T3_ro_toS(_m->LR_TTPMS_T3_phys);
  _m->LR_TTPMS_T4_ro = (uint16_t) HYTECH_LR_TTPMS_T4_ro_toS(_m->LR_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_T1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_T2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_T3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_T4_ro & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_2_DLC;
  *_ide = (uint8_t) LR_TTPMS_2_IDE;
  return LR_TTPMS_2_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_T5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T5_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T5_ro_fromS(_m->LR_TTPMS_T5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T6_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T6_ro_fromS(_m->LR_TTPMS_T6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T7_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T7_ro_fromS(_m->LR_TTPMS_T7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T8_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T8_ro_fromS(_m->LR_TTPMS_T8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_3_hytech(&_m->mon1, LR_TTPMS_3_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_3_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_3_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T5_ro = (uint16_t) HYTECH_LR_TTPMS_T5_ro_toS(_m->LR_TTPMS_T5_phys);
  _m->LR_TTPMS_T6_ro = (uint16_t) HYTECH_LR_TTPMS_T6_ro_toS(_m->LR_TTPMS_T6_phys);
  _m->LR_TTPMS_T7_ro = (uint16_t) HYTECH_LR_TTPMS_T7_ro_toS(_m->LR_TTPMS_T7_phys);
  _m->LR_TTPMS_T8_ro = (uint16_t) HYTECH_LR_TTPMS_T8_ro_toS(_m->LR_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_T5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_T6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_T7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_T8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_3_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_3_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_3_IDE;
  return LR_TTPMS_3_CANID;
}

#else

uint32_t Pack_LR_TTPMS_3_hytech(LR_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_3_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T5_ro = (uint16_t) HYTECH_LR_TTPMS_T5_ro_toS(_m->LR_TTPMS_T5_phys);
  _m->LR_TTPMS_T6_ro = (uint16_t) HYTECH_LR_TTPMS_T6_ro_toS(_m->LR_TTPMS_T6_phys);
  _m->LR_TTPMS_T7_ro = (uint16_t) HYTECH_LR_TTPMS_T7_ro_toS(_m->LR_TTPMS_T7_phys);
  _m->LR_TTPMS_T8_ro = (uint16_t) HYTECH_LR_TTPMS_T8_ro_toS(_m->LR_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_T5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_T6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_T7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_T8_ro & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_3_DLC;
  *_ide = (uint8_t) LR_TTPMS_3_IDE;
  return LR_TTPMS_3_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_T9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T9_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T9_ro_fromS(_m->LR_TTPMS_T9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T10_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T10_ro_fromS(_m->LR_TTPMS_T10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T11_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T11_ro_fromS(_m->LR_TTPMS_T11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T12_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T12_ro_fromS(_m->LR_TTPMS_T12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_4_hytech(&_m->mon1, LR_TTPMS_4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T9_ro = (uint16_t) HYTECH_LR_TTPMS_T9_ro_toS(_m->LR_TTPMS_T9_phys);
  _m->LR_TTPMS_T10_ro = (uint16_t) HYTECH_LR_TTPMS_T10_ro_toS(_m->LR_TTPMS_T10_phys);
  _m->LR_TTPMS_T11_ro = (uint16_t) HYTECH_LR_TTPMS_T11_ro_toS(_m->LR_TTPMS_T11_phys);
  _m->LR_TTPMS_T12_ro = (uint16_t) HYTECH_LR_TTPMS_T12_ro_toS(_m->LR_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_T9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_T10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_T11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_T12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_4_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_4_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_4_IDE;
  return LR_TTPMS_4_CANID;
}

#else

uint32_t Pack_LR_TTPMS_4_hytech(LR_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T9_ro = (uint16_t) HYTECH_LR_TTPMS_T9_ro_toS(_m->LR_TTPMS_T9_phys);
  _m->LR_TTPMS_T10_ro = (uint16_t) HYTECH_LR_TTPMS_T10_ro_toS(_m->LR_TTPMS_T10_phys);
  _m->LR_TTPMS_T11_ro = (uint16_t) HYTECH_LR_TTPMS_T11_ro_toS(_m->LR_TTPMS_T11_phys);
  _m->LR_TTPMS_T12_ro = (uint16_t) HYTECH_LR_TTPMS_T12_ro_toS(_m->LR_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_T9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_T10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_T11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_T12_ro & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_4_DLC;
  *_ide = (uint8_t) LR_TTPMS_4_IDE;
  return LR_TTPMS_4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_T13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T13_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T13_ro_fromS(_m->LR_TTPMS_T13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T14_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T14_ro_fromS(_m->LR_TTPMS_T14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T15_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T15_ro_fromS(_m->LR_TTPMS_T15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_T16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T16_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T16_ro_fromS(_m->LR_TTPMS_T16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_5_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_5_hytech(&_m->mon1, LR_TTPMS_5_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_5_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_5_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T13_ro = (uint16_t) HYTECH_LR_TTPMS_T13_ro_toS(_m->LR_TTPMS_T13_phys);
  _m->LR_TTPMS_T14_ro = (uint16_t) HYTECH_LR_TTPMS_T14_ro_toS(_m->LR_TTPMS_T14_phys);
  _m->LR_TTPMS_T15_ro = (uint16_t) HYTECH_LR_TTPMS_T15_ro_toS(_m->LR_TTPMS_T15_phys);
  _m->LR_TTPMS_T16_ro = (uint16_t) HYTECH_LR_TTPMS_T16_ro_toS(_m->LR_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_T13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_T14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_T15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_T16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_5_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_5_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_5_IDE;
  return LR_TTPMS_5_CANID;
}

#else

uint32_t Pack_LR_TTPMS_5_hytech(LR_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_5_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T13_ro = (uint16_t) HYTECH_LR_TTPMS_T13_ro_toS(_m->LR_TTPMS_T13_phys);
  _m->LR_TTPMS_T14_ro = (uint16_t) HYTECH_LR_TTPMS_T14_ro_toS(_m->LR_TTPMS_T14_phys);
  _m->LR_TTPMS_T15_ro = (uint16_t) HYTECH_LR_TTPMS_T15_ro_toS(_m->LR_TTPMS_T15_phys);
  _m->LR_TTPMS_T16_ro = (uint16_t) HYTECH_LR_TTPMS_T16_ro_toS(_m->LR_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_T13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_T14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_T15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_T16_ro & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_5_DLC;
  *_ide = (uint8_t) LR_TTPMS_5_IDE;
  return LR_TTPMS_5_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->LR_TTPMS_TC = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->LR_TTPMS_RSSI = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
  _m->LR_TTPMS_T_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T_phys = (sigfloat_t)(HYTECH_LR_TTPMS_T_ro_fromS(_m->LR_TTPMS_T_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->LR_TTPMS_NODE_ID = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < LR_TTPMS_6_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_LR_TTPMS_6_hytech(&_m->mon1, LR_TTPMS_6_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return LR_TTPMS_6_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_6_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T_ro = (uint16_t) HYTECH_LR_TTPMS_T_ro_toS(_m->LR_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->LR_TTPMS_TC >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->LR_TTPMS_TC & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->LR_TTPMS_RSSI >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->LR_TTPMS_RSSI & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->LR_TTPMS_T_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->LR_TTPMS_T_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->LR_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->LR_TTPMS_NODE_ID & (0xFFU)) );

  cframe->MsgId = (uint32_t) LR_TTPMS_6_CANID;
  cframe->DLC = (uint8_t) LR_TTPMS_6_DLC;
  cframe->IDE = (uint8_t) LR_TTPMS_6_IDE;
  return LR_TTPMS_6_CANID;
}

#else

uint32_t Pack_LR_TTPMS_6_hytech(LR_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(LR_TTPMS_6_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->LR_TTPMS_T_ro = (uint16_t) HYTECH_LR_TTPMS_T_ro_toS(_m->LR_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->LR_TTPMS_TC >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->LR_TTPMS_TC & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->LR_TTPMS_RSSI >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->LR_TTPMS_RSSI & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->LR_TTPMS_T_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->LR_TTPMS_T_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->LR_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->LR_TTPMS_NODE_ID & (0xFFU)) );

  *_len = (uint8_t) LR_TTPMS_6_DLC;
  *_ide = (uint8_t) LR_TTPMS_6_IDE;
  return LR_TTPMS_6_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_SN = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->RF_TTPMS_BAT_V = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->RF_TTPMS_P = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
  _m->RF_TTPMS_P_GAUGE = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_1_hytech(&_m->mon1, RF_TTPMS_1_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_1_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_1_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_SN >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_SN & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_BAT_V & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_P >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_P & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_P_GAUGE & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_1_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_1_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_1_IDE;
  return RF_TTPMS_1_CANID;
}

#else

uint32_t Pack_RF_TTPMS_1_hytech(RF_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_1_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_SN >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_SN & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_BAT_V & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_P >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_P & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_P_GAUGE & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_1_DLC;
  *_ide = (uint8_t) RF_TTPMS_1_IDE;
  return RF_TTPMS_1_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_T1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T1_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T1_ro_fromS(_m->RF_TTPMS_T1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T2_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T2_ro_fromS(_m->RF_TTPMS_T2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T3_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T3_ro_fromS(_m->RF_TTPMS_T3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T4_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T4_ro_fromS(_m->RF_TTPMS_T4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_2_hytech(&_m->mon1, RF_TTPMS_2_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_2_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_2_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T1_ro = (uint16_t) HYTECH_RF_TTPMS_T1_ro_toS(_m->RF_TTPMS_T1_phys);
  _m->RF_TTPMS_T2_ro = (uint16_t) HYTECH_RF_TTPMS_T2_ro_toS(_m->RF_TTPMS_T2_phys);
  _m->RF_TTPMS_T3_ro = (uint16_t) HYTECH_RF_TTPMS_T3_ro_toS(_m->RF_TTPMS_T3_phys);
  _m->RF_TTPMS_T4_ro = (uint16_t) HYTECH_RF_TTPMS_T4_ro_toS(_m->RF_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_T1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_T2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_T3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_T4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_2_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_2_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_2_IDE;
  return RF_TTPMS_2_CANID;
}

#else

uint32_t Pack_RF_TTPMS_2_hytech(RF_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_2_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T1_ro = (uint16_t) HYTECH_RF_TTPMS_T1_ro_toS(_m->RF_TTPMS_T1_phys);
  _m->RF_TTPMS_T2_ro = (uint16_t) HYTECH_RF_TTPMS_T2_ro_toS(_m->RF_TTPMS_T2_phys);
  _m->RF_TTPMS_T3_ro = (uint16_t) HYTECH_RF_TTPMS_T3_ro_toS(_m->RF_TTPMS_T3_phys);
  _m->RF_TTPMS_T4_ro = (uint16_t) HYTECH_RF_TTPMS_T4_ro_toS(_m->RF_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_T1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_T2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_T3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_T4_ro & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_2_DLC;
  *_ide = (uint8_t) RF_TTPMS_2_IDE;
  return RF_TTPMS_2_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_T5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T5_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T5_ro_fromS(_m->RF_TTPMS_T5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T6_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T6_ro_fromS(_m->RF_TTPMS_T6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T7_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T7_ro_fromS(_m->RF_TTPMS_T7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T8_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T8_ro_fromS(_m->RF_TTPMS_T8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_3_hytech(&_m->mon1, RF_TTPMS_3_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_3_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_3_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T5_ro = (uint16_t) HYTECH_RF_TTPMS_T5_ro_toS(_m->RF_TTPMS_T5_phys);
  _m->RF_TTPMS_T6_ro = (uint16_t) HYTECH_RF_TTPMS_T6_ro_toS(_m->RF_TTPMS_T6_phys);
  _m->RF_TTPMS_T7_ro = (uint16_t) HYTECH_RF_TTPMS_T7_ro_toS(_m->RF_TTPMS_T7_phys);
  _m->RF_TTPMS_T8_ro = (uint16_t) HYTECH_RF_TTPMS_T8_ro_toS(_m->RF_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_T5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_T6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_T7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_T8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_3_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_3_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_3_IDE;
  return RF_TTPMS_3_CANID;
}

#else

uint32_t Pack_RF_TTPMS_3_hytech(RF_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_3_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T5_ro = (uint16_t) HYTECH_RF_TTPMS_T5_ro_toS(_m->RF_TTPMS_T5_phys);
  _m->RF_TTPMS_T6_ro = (uint16_t) HYTECH_RF_TTPMS_T6_ro_toS(_m->RF_TTPMS_T6_phys);
  _m->RF_TTPMS_T7_ro = (uint16_t) HYTECH_RF_TTPMS_T7_ro_toS(_m->RF_TTPMS_T7_phys);
  _m->RF_TTPMS_T8_ro = (uint16_t) HYTECH_RF_TTPMS_T8_ro_toS(_m->RF_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_T5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_T6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_T7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_T8_ro & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_3_DLC;
  *_ide = (uint8_t) RF_TTPMS_3_IDE;
  return RF_TTPMS_3_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_T9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T9_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T9_ro_fromS(_m->RF_TTPMS_T9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T10_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T10_ro_fromS(_m->RF_TTPMS_T10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T11_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T11_ro_fromS(_m->RF_TTPMS_T11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T12_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T12_ro_fromS(_m->RF_TTPMS_T12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_4_hytech(&_m->mon1, RF_TTPMS_4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T9_ro = (uint16_t) HYTECH_RF_TTPMS_T9_ro_toS(_m->RF_TTPMS_T9_phys);
  _m->RF_TTPMS_T10_ro = (uint16_t) HYTECH_RF_TTPMS_T10_ro_toS(_m->RF_TTPMS_T10_phys);
  _m->RF_TTPMS_T11_ro = (uint16_t) HYTECH_RF_TTPMS_T11_ro_toS(_m->RF_TTPMS_T11_phys);
  _m->RF_TTPMS_T12_ro = (uint16_t) HYTECH_RF_TTPMS_T12_ro_toS(_m->RF_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_T9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_T10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_T11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_T12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_4_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_4_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_4_IDE;
  return RF_TTPMS_4_CANID;
}

#else

uint32_t Pack_RF_TTPMS_4_hytech(RF_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T9_ro = (uint16_t) HYTECH_RF_TTPMS_T9_ro_toS(_m->RF_TTPMS_T9_phys);
  _m->RF_TTPMS_T10_ro = (uint16_t) HYTECH_RF_TTPMS_T10_ro_toS(_m->RF_TTPMS_T10_phys);
  _m->RF_TTPMS_T11_ro = (uint16_t) HYTECH_RF_TTPMS_T11_ro_toS(_m->RF_TTPMS_T11_phys);
  _m->RF_TTPMS_T12_ro = (uint16_t) HYTECH_RF_TTPMS_T12_ro_toS(_m->RF_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_T9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_T10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_T11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_T12_ro & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_4_DLC;
  *_ide = (uint8_t) RF_TTPMS_4_IDE;
  return RF_TTPMS_4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_T13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T13_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T13_ro_fromS(_m->RF_TTPMS_T13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T14_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T14_ro_fromS(_m->RF_TTPMS_T14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T15_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T15_ro_fromS(_m->RF_TTPMS_T15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_T16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T16_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T16_ro_fromS(_m->RF_TTPMS_T16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_5_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_5_hytech(&_m->mon1, RF_TTPMS_5_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_5_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_5_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T13_ro = (uint16_t) HYTECH_RF_TTPMS_T13_ro_toS(_m->RF_TTPMS_T13_phys);
  _m->RF_TTPMS_T14_ro = (uint16_t) HYTECH_RF_TTPMS_T14_ro_toS(_m->RF_TTPMS_T14_phys);
  _m->RF_TTPMS_T15_ro = (uint16_t) HYTECH_RF_TTPMS_T15_ro_toS(_m->RF_TTPMS_T15_phys);
  _m->RF_TTPMS_T16_ro = (uint16_t) HYTECH_RF_TTPMS_T16_ro_toS(_m->RF_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_T13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_T14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_T15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_T16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_5_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_5_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_5_IDE;
  return RF_TTPMS_5_CANID;
}

#else

uint32_t Pack_RF_TTPMS_5_hytech(RF_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_5_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T13_ro = (uint16_t) HYTECH_RF_TTPMS_T13_ro_toS(_m->RF_TTPMS_T13_phys);
  _m->RF_TTPMS_T14_ro = (uint16_t) HYTECH_RF_TTPMS_T14_ro_toS(_m->RF_TTPMS_T14_phys);
  _m->RF_TTPMS_T15_ro = (uint16_t) HYTECH_RF_TTPMS_T15_ro_toS(_m->RF_TTPMS_T15_phys);
  _m->RF_TTPMS_T16_ro = (uint16_t) HYTECH_RF_TTPMS_T16_ro_toS(_m->RF_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_T13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_T14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_T15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_T16_ro & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_5_DLC;
  *_ide = (uint8_t) RF_TTPMS_5_IDE;
  return RF_TTPMS_5_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RF_TTPMS_TC = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->RF_TTPMS_RSSI = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
  _m->RF_TTPMS_T_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T_phys = (sigfloat_t)(HYTECH_RF_TTPMS_T_ro_fromS(_m->RF_TTPMS_T_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RF_TTPMS_NODE_ID = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RF_TTPMS_6_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RF_TTPMS_6_hytech(&_m->mon1, RF_TTPMS_6_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RF_TTPMS_6_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_6_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T_ro = (uint16_t) HYTECH_RF_TTPMS_T_ro_toS(_m->RF_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RF_TTPMS_TC >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RF_TTPMS_TC & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RF_TTPMS_RSSI >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RF_TTPMS_RSSI & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RF_TTPMS_T_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RF_TTPMS_T_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RF_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RF_TTPMS_NODE_ID & (0xFFU)) );

  cframe->MsgId = (uint32_t) RF_TTPMS_6_CANID;
  cframe->DLC = (uint8_t) RF_TTPMS_6_DLC;
  cframe->IDE = (uint8_t) RF_TTPMS_6_IDE;
  return RF_TTPMS_6_CANID;
}

#else

uint32_t Pack_RF_TTPMS_6_hytech(RF_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RF_TTPMS_6_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RF_TTPMS_T_ro = (uint16_t) HYTECH_RF_TTPMS_T_ro_toS(_m->RF_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RF_TTPMS_TC >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RF_TTPMS_TC & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RF_TTPMS_RSSI >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RF_TTPMS_RSSI & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RF_TTPMS_T_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RF_TTPMS_T_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RF_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RF_TTPMS_NODE_ID & (0xFFU)) );

  *_len = (uint8_t) RF_TTPMS_6_DLC;
  *_ide = (uint8_t) RF_TTPMS_6_IDE;
  return RF_TTPMS_6_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_SN = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->RR_TTPMS_BAT_V = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->RR_TTPMS_P = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
  _m->RR_TTPMS_P_GAUGE = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_1_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_1_hytech(&_m->mon1, RR_TTPMS_1_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_1_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_1_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_SN >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_SN & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_BAT_V & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_P >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_P & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_P_GAUGE & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_1_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_1_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_1_IDE;
  return RR_TTPMS_1_CANID;
}

#else

uint32_t Pack_RR_TTPMS_1_hytech(RR_TTPMS_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_1_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_SN >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_SN & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_BAT_V >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_BAT_V & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_P >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_P & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_P_GAUGE >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_P_GAUGE & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_1_DLC;
  *_ide = (uint8_t) RR_TTPMS_1_IDE;
  return RR_TTPMS_1_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_T1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T1_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T1_ro_fromS(_m->RR_TTPMS_T1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T2_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T2_ro_fromS(_m->RR_TTPMS_T2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T3_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T3_ro_fromS(_m->RR_TTPMS_T3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T4_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T4_ro_fromS(_m->RR_TTPMS_T4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_2_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_2_hytech(&_m->mon1, RR_TTPMS_2_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_2_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_2_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T1_ro = (uint16_t) HYTECH_RR_TTPMS_T1_ro_toS(_m->RR_TTPMS_T1_phys);
  _m->RR_TTPMS_T2_ro = (uint16_t) HYTECH_RR_TTPMS_T2_ro_toS(_m->RR_TTPMS_T2_phys);
  _m->RR_TTPMS_T3_ro = (uint16_t) HYTECH_RR_TTPMS_T3_ro_toS(_m->RR_TTPMS_T3_phys);
  _m->RR_TTPMS_T4_ro = (uint16_t) HYTECH_RR_TTPMS_T4_ro_toS(_m->RR_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_T1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_T2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_T3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_T4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_2_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_2_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_2_IDE;
  return RR_TTPMS_2_CANID;
}

#else

uint32_t Pack_RR_TTPMS_2_hytech(RR_TTPMS_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_2_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T1_ro = (uint16_t) HYTECH_RR_TTPMS_T1_ro_toS(_m->RR_TTPMS_T1_phys);
  _m->RR_TTPMS_T2_ro = (uint16_t) HYTECH_RR_TTPMS_T2_ro_toS(_m->RR_TTPMS_T2_phys);
  _m->RR_TTPMS_T3_ro = (uint16_t) HYTECH_RR_TTPMS_T3_ro_toS(_m->RR_TTPMS_T3_phys);
  _m->RR_TTPMS_T4_ro = (uint16_t) HYTECH_RR_TTPMS_T4_ro_toS(_m->RR_TTPMS_T4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_T1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_T1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_T2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_T2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_T3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_T3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_T4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_T4_ro & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_2_DLC;
  *_ide = (uint8_t) RR_TTPMS_2_IDE;
  return RR_TTPMS_2_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_T5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T5_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T5_ro_fromS(_m->RR_TTPMS_T5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T6_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T6_ro_fromS(_m->RR_TTPMS_T6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T7_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T7_ro_fromS(_m->RR_TTPMS_T7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T8_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T8_ro_fromS(_m->RR_TTPMS_T8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_3_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_3_hytech(&_m->mon1, RR_TTPMS_3_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_3_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_3_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T5_ro = (uint16_t) HYTECH_RR_TTPMS_T5_ro_toS(_m->RR_TTPMS_T5_phys);
  _m->RR_TTPMS_T6_ro = (uint16_t) HYTECH_RR_TTPMS_T6_ro_toS(_m->RR_TTPMS_T6_phys);
  _m->RR_TTPMS_T7_ro = (uint16_t) HYTECH_RR_TTPMS_T7_ro_toS(_m->RR_TTPMS_T7_phys);
  _m->RR_TTPMS_T8_ro = (uint16_t) HYTECH_RR_TTPMS_T8_ro_toS(_m->RR_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_T5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_T6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_T7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_T8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_3_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_3_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_3_IDE;
  return RR_TTPMS_3_CANID;
}

#else

uint32_t Pack_RR_TTPMS_3_hytech(RR_TTPMS_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_3_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T5_ro = (uint16_t) HYTECH_RR_TTPMS_T5_ro_toS(_m->RR_TTPMS_T5_phys);
  _m->RR_TTPMS_T6_ro = (uint16_t) HYTECH_RR_TTPMS_T6_ro_toS(_m->RR_TTPMS_T6_phys);
  _m->RR_TTPMS_T7_ro = (uint16_t) HYTECH_RR_TTPMS_T7_ro_toS(_m->RR_TTPMS_T7_phys);
  _m->RR_TTPMS_T8_ro = (uint16_t) HYTECH_RR_TTPMS_T8_ro_toS(_m->RR_TTPMS_T8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_T5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_T5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_T6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_T6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_T7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_T7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_T8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_T8_ro & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_3_DLC;
  *_ide = (uint8_t) RR_TTPMS_3_IDE;
  return RR_TTPMS_3_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_T9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T9_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T9_ro_fromS(_m->RR_TTPMS_T9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T10_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T10_ro_fromS(_m->RR_TTPMS_T10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T11_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T11_ro_fromS(_m->RR_TTPMS_T11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T12_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T12_ro_fromS(_m->RR_TTPMS_T12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_4_hytech(&_m->mon1, RR_TTPMS_4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T9_ro = (uint16_t) HYTECH_RR_TTPMS_T9_ro_toS(_m->RR_TTPMS_T9_phys);
  _m->RR_TTPMS_T10_ro = (uint16_t) HYTECH_RR_TTPMS_T10_ro_toS(_m->RR_TTPMS_T10_phys);
  _m->RR_TTPMS_T11_ro = (uint16_t) HYTECH_RR_TTPMS_T11_ro_toS(_m->RR_TTPMS_T11_phys);
  _m->RR_TTPMS_T12_ro = (uint16_t) HYTECH_RR_TTPMS_T12_ro_toS(_m->RR_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_T9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_T10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_T11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_T12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_4_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_4_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_4_IDE;
  return RR_TTPMS_4_CANID;
}

#else

uint32_t Pack_RR_TTPMS_4_hytech(RR_TTPMS_4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T9_ro = (uint16_t) HYTECH_RR_TTPMS_T9_ro_toS(_m->RR_TTPMS_T9_phys);
  _m->RR_TTPMS_T10_ro = (uint16_t) HYTECH_RR_TTPMS_T10_ro_toS(_m->RR_TTPMS_T10_phys);
  _m->RR_TTPMS_T11_ro = (uint16_t) HYTECH_RR_TTPMS_T11_ro_toS(_m->RR_TTPMS_T11_phys);
  _m->RR_TTPMS_T12_ro = (uint16_t) HYTECH_RR_TTPMS_T12_ro_toS(_m->RR_TTPMS_T12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_T9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_T9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_T10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_T10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_T11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_T11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_T12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_T12_ro & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_4_DLC;
  *_ide = (uint8_t) RR_TTPMS_4_IDE;
  return RR_TTPMS_4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_T13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T13_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T13_ro_fromS(_m->RR_TTPMS_T13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T14_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T14_ro_fromS(_m->RR_TTPMS_T14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T15_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T15_ro_fromS(_m->RR_TTPMS_T15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_T16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T16_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T16_ro_fromS(_m->RR_TTPMS_T16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_5_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_5_hytech(&_m->mon1, RR_TTPMS_5_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_5_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_5_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T13_ro = (uint16_t) HYTECH_RR_TTPMS_T13_ro_toS(_m->RR_TTPMS_T13_phys);
  _m->RR_TTPMS_T14_ro = (uint16_t) HYTECH_RR_TTPMS_T14_ro_toS(_m->RR_TTPMS_T14_phys);
  _m->RR_TTPMS_T15_ro = (uint16_t) HYTECH_RR_TTPMS_T15_ro_toS(_m->RR_TTPMS_T15_phys);
  _m->RR_TTPMS_T16_ro = (uint16_t) HYTECH_RR_TTPMS_T16_ro_toS(_m->RR_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_T13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_T14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_T15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_T16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_5_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_5_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_5_IDE;
  return RR_TTPMS_5_CANID;
}

#else

uint32_t Pack_RR_TTPMS_5_hytech(RR_TTPMS_5_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_5_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T13_ro = (uint16_t) HYTECH_RR_TTPMS_T13_ro_toS(_m->RR_TTPMS_T13_phys);
  _m->RR_TTPMS_T14_ro = (uint16_t) HYTECH_RR_TTPMS_T14_ro_toS(_m->RR_TTPMS_T14_phys);
  _m->RR_TTPMS_T15_ro = (uint16_t) HYTECH_RR_TTPMS_T15_ro_toS(_m->RR_TTPMS_T15_phys);
  _m->RR_TTPMS_T16_ro = (uint16_t) HYTECH_RR_TTPMS_T16_ro_toS(_m->RR_TTPMS_T16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_T13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_T13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_T14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_T14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_T15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_T15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_T16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_T16_ro & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_5_DLC;
  *_ide = (uint8_t) RR_TTPMS_5_IDE;
  return RR_TTPMS_5_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->RR_TTPMS_TC = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->RR_TTPMS_RSSI = (int16_t) __ext_sig__(( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) ), 16);
  _m->RR_TTPMS_T_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T_phys = (sigfloat_t)(HYTECH_RR_TTPMS_T_ro_fromS(_m->RR_TTPMS_T_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->RR_TTPMS_NODE_ID = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < RR_TTPMS_6_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_RR_TTPMS_6_hytech(&_m->mon1, RR_TTPMS_6_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return RR_TTPMS_6_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_6_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T_ro = (uint16_t) HYTECH_RR_TTPMS_T_ro_toS(_m->RR_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->RR_TTPMS_TC >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->RR_TTPMS_TC & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->RR_TTPMS_RSSI >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->RR_TTPMS_RSSI & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->RR_TTPMS_T_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->RR_TTPMS_T_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->RR_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->RR_TTPMS_NODE_ID & (0xFFU)) );

  cframe->MsgId = (uint32_t) RR_TTPMS_6_CANID;
  cframe->DLC = (uint8_t) RR_TTPMS_6_DLC;
  cframe->IDE = (uint8_t) RR_TTPMS_6_IDE;
  return RR_TTPMS_6_CANID;
}

#else

uint32_t Pack_RR_TTPMS_6_hytech(RR_TTPMS_6_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(RR_TTPMS_6_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->RR_TTPMS_T_ro = (uint16_t) HYTECH_RR_TTPMS_T_ro_toS(_m->RR_TTPMS_T_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->RR_TTPMS_TC >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->RR_TTPMS_TC & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->RR_TTPMS_RSSI >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->RR_TTPMS_RSSI & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->RR_TTPMS_T_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->RR_TTPMS_T_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->RR_TTPMS_NODE_ID >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->RR_TTPMS_NODE_ID & (0xFFU)) );

  *_len = (uint8_t) RR_TTPMS_6_DLC;
  *_ide = (uint8_t) RR_TTPMS_6_IDE;
  return RR_TTPMS_6_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_sensor_prog_const = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
  _m->brake_temp_sensor_base_can_id = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
  _m->brake_temp_sensor_emissivity_ro = (uint8_t) ( (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_sensor_emissivity_phys = (sigfloat_t)(HYTECH_brake_temp_sensor_emissivity_ro_fromS(_m->brake_temp_sensor_emissivity_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_sensor_sampling_freq = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->brake_temp_sensor_ch_setting = (uint8_t) ( (_d[6] & (0xFFU)) );
  _m->brake_temp_sensor_can_bitrate = (uint8_t) ( (_d[7] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < BRAKE_ROTOR_SENSOR_COMMAND_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_BRAKE_ROTOR_SENSOR_COMMAND_hytech(&_m->mon1, BRAKE_ROTOR_SENSOR_COMMAND_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return BRAKE_ROTOR_SENSOR_COMMAND_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BRAKE_ROTOR_SENSOR_COMMAND_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_sensor_emissivity_ro = (uint8_t) HYTECH_brake_temp_sensor_emissivity_ro_toS(_m->brake_temp_sensor_emissivity_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_sensor_prog_const >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_sensor_prog_const & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_sensor_base_can_id >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_sensor_base_can_id & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->brake_temp_sensor_emissivity_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_sensor_sampling_freq & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->brake_temp_sensor_ch_setting & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_sensor_can_bitrate & (0xFFU)) );

  cframe->MsgId = (uint32_t) BRAKE_ROTOR_SENSOR_COMMAND_CANID;
  cframe->DLC = (uint8_t) BRAKE_ROTOR_SENSOR_COMMAND_DLC;
  cframe->IDE = (uint8_t) BRAKE_ROTOR_SENSOR_COMMAND_IDE;
  return BRAKE_ROTOR_SENSOR_COMMAND_CANID;
}

#else

uint32_t Pack_BRAKE_ROTOR_SENSOR_COMMAND_hytech(BRAKE_ROTOR_SENSOR_COMMAND_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(BRAKE_ROTOR_SENSOR_COMMAND_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_sensor_emissivity_ro = (uint8_t) HYTECH_brake_temp_sensor_emissivity_ro_toS(_m->brake_temp_sensor_emissivity_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_sensor_prog_const >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_sensor_prog_const & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_sensor_base_can_id >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_sensor_base_can_id & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->brake_temp_sensor_emissivity_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_sensor_sampling_freq & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->brake_temp_sensor_ch_setting & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_sensor_can_bitrate & (0xFFU)) );

  *_len = (uint8_t) BRAKE_ROTOR_SENSOR_COMMAND_DLC;
  *_ide = (uint8_t) BRAKE_ROTOR_SENSOR_COMMAND_IDE;
  return BRAKE_ROTOR_SENSOR_COMMAND_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_phys = (sigfloat_t)(HYTECH_brake_temp_channel_1_ro_fromS(_m->brake_temp_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_2_phys = (sigfloat_t)(HYTECH_brake_temp_channel_2_ro_fromS(_m->brake_temp_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_3_phys = (sigfloat_t)(HYTECH_brake_temp_channel_3_ro_fromS(_m->brake_temp_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_4_phys = (sigfloat_t)(HYTECH_brake_temp_channel_4_ro_fromS(_m->brake_temp_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(&_m->mon1, FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_ro = (uint16_t) HYTECH_brake_temp_channel_1_ro_toS(_m->brake_temp_channel_1_phys);
  _m->brake_temp_channel_2_ro = (uint16_t) HYTECH_brake_temp_channel_2_ro_toS(_m->brake_temp_channel_2_phys);
  _m->brake_temp_channel_3_ro = (uint16_t) HYTECH_brake_temp_channel_3_ro_toS(_m->brake_temp_channel_3_phys);
  _m->brake_temp_channel_4_ro = (uint16_t) HYTECH_brake_temp_channel_4_ro_toS(_m->brake_temp_channel_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
  cframe->DLC = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC;
  cframe->IDE = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH1_CH4_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#else

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FL_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_ro = (uint16_t) HYTECH_brake_temp_channel_1_ro_toS(_m->brake_temp_channel_1_phys);
  _m->brake_temp_channel_2_ro = (uint16_t) HYTECH_brake_temp_channel_2_ro_toS(_m->brake_temp_channel_2_phys);
  _m->brake_temp_channel_3_ro = (uint16_t) HYTECH_brake_temp_channel_3_ro_toS(_m->brake_temp_channel_3_phys);
  _m->brake_temp_channel_4_ro = (uint16_t) HYTECH_brake_temp_channel_4_ro_toS(_m->brake_temp_channel_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_4_ro & (0xFFU)) );

  *_len = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH1_CH4_DLC;
  *_ide = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH1_CH4_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_phys = (sigfloat_t)(HYTECH_brake_temp_channel_5_ro_fromS(_m->brake_temp_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_6_phys = (sigfloat_t)(HYTECH_brake_temp_channel_6_ro_fromS(_m->brake_temp_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_7_phys = (sigfloat_t)(HYTECH_brake_temp_channel_7_ro_fromS(_m->brake_temp_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_8_phys = (sigfloat_t)(HYTECH_brake_temp_channel_8_ro_fromS(_m->brake_temp_channel_8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(&_m->mon1, FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_ro = (uint16_t) HYTECH_brake_temp_channel_5_ro_toS(_m->brake_temp_channel_5_phys);
  _m->brake_temp_channel_6_ro = (uint16_t) HYTECH_brake_temp_channel_6_ro_toS(_m->brake_temp_channel_6_phys);
  _m->brake_temp_channel_7_ro = (uint16_t) HYTECH_brake_temp_channel_7_ro_toS(_m->brake_temp_channel_7_phys);
  _m->brake_temp_channel_8_ro = (uint16_t) HYTECH_brake_temp_channel_8_ro_toS(_m->brake_temp_channel_8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
  cframe->DLC = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC;
  cframe->IDE = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH5_CH8_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#else

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FL_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_ro = (uint16_t) HYTECH_brake_temp_channel_5_ro_toS(_m->brake_temp_channel_5_phys);
  _m->brake_temp_channel_6_ro = (uint16_t) HYTECH_brake_temp_channel_6_ro_toS(_m->brake_temp_channel_6_phys);
  _m->brake_temp_channel_7_ro = (uint16_t) HYTECH_brake_temp_channel_7_ro_toS(_m->brake_temp_channel_7_phys);
  _m->brake_temp_channel_8_ro = (uint16_t) HYTECH_brake_temp_channel_8_ro_toS(_m->brake_temp_channel_8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_8_ro & (0xFFU)) );

  *_len = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH5_CH8_DLC;
  *_ide = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH5_CH8_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_phys = (sigfloat_t)(HYTECH_brake_temp_channel_9_ro_fromS(_m->brake_temp_channel_9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_10_phys = (sigfloat_t)(HYTECH_brake_temp_channel_10_ro_fromS(_m->brake_temp_channel_10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_11_phys = (sigfloat_t)(HYTECH_brake_temp_channel_11_ro_fromS(_m->brake_temp_channel_11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_12_phys = (sigfloat_t)(HYTECH_brake_temp_channel_12_ro_fromS(_m->brake_temp_channel_12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(&_m->mon1, FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_ro = (uint16_t) HYTECH_brake_temp_channel_9_ro_toS(_m->brake_temp_channel_9_phys);
  _m->brake_temp_channel_10_ro = (uint16_t) HYTECH_brake_temp_channel_10_ro_toS(_m->brake_temp_channel_10_phys);
  _m->brake_temp_channel_11_ro = (uint16_t) HYTECH_brake_temp_channel_11_ro_toS(_m->brake_temp_channel_11_phys);
  _m->brake_temp_channel_12_ro = (uint16_t) HYTECH_brake_temp_channel_12_ro_toS(_m->brake_temp_channel_12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
  cframe->DLC = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC;
  cframe->IDE = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH9_CH12_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#else

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FL_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_ro = (uint16_t) HYTECH_brake_temp_channel_9_ro_toS(_m->brake_temp_channel_9_phys);
  _m->brake_temp_channel_10_ro = (uint16_t) HYTECH_brake_temp_channel_10_ro_toS(_m->brake_temp_channel_10_phys);
  _m->brake_temp_channel_11_ro = (uint16_t) HYTECH_brake_temp_channel_11_ro_toS(_m->brake_temp_channel_11_phys);
  _m->brake_temp_channel_12_ro = (uint16_t) HYTECH_brake_temp_channel_12_ro_toS(_m->brake_temp_channel_12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_12_ro & (0xFFU)) );

  *_len = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH9_CH12_DLC;
  *_ide = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH9_CH12_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_phys = (sigfloat_t)(HYTECH_brake_temp_channel_13_ro_fromS(_m->brake_temp_channel_13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_14_phys = (sigfloat_t)(HYTECH_brake_temp_channel_14_ro_fromS(_m->brake_temp_channel_14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_15_phys = (sigfloat_t)(HYTECH_brake_temp_channel_15_ro_fromS(_m->brake_temp_channel_15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_16_phys = (sigfloat_t)(HYTECH_brake_temp_channel_16_ro_fromS(_m->brake_temp_channel_16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(&_m->mon1, FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_ro = (uint16_t) HYTECH_brake_temp_channel_13_ro_toS(_m->brake_temp_channel_13_phys);
  _m->brake_temp_channel_14_ro = (uint16_t) HYTECH_brake_temp_channel_14_ro_toS(_m->brake_temp_channel_14_phys);
  _m->brake_temp_channel_15_ro = (uint16_t) HYTECH_brake_temp_channel_15_ro_toS(_m->brake_temp_channel_15_phys);
  _m->brake_temp_channel_16_ro = (uint16_t) HYTECH_brake_temp_channel_16_ro_toS(_m->brake_temp_channel_16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
  cframe->DLC = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC;
  cframe->IDE = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH13_CH16_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#else

uint32_t Pack_FL_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FL_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_ro = (uint16_t) HYTECH_brake_temp_channel_13_ro_toS(_m->brake_temp_channel_13_phys);
  _m->brake_temp_channel_14_ro = (uint16_t) HYTECH_brake_temp_channel_14_ro_toS(_m->brake_temp_channel_14_phys);
  _m->brake_temp_channel_15_ro = (uint16_t) HYTECH_brake_temp_channel_15_ro_toS(_m->brake_temp_channel_15_phys);
  _m->brake_temp_channel_16_ro = (uint16_t) HYTECH_brake_temp_channel_16_ro_toS(_m->brake_temp_channel_16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_16_ro & (0xFFU)) );

  *_len = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH13_CH16_DLC;
  *_ide = (uint8_t) FL_BRAKE_ROTOR_TEMP_CH13_CH16_IDE;
  return FL_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_rotor_sensor_temp_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_phys = (sigfloat_t)(HYTECH_brake_rotor_sensor_temp_ro_fromS(_m->brake_rotor_sensor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FL_BRAKE_ROTOR_SENSOR_TEMP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(&_m->mon1, FL_BRAKE_ROTOR_SENSOR_TEMP_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FL_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_SENSOR_TEMP_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_ro = (uint16_t) HYTECH_brake_rotor_sensor_temp_ro_toS(_m->brake_rotor_sensor_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_rotor_sensor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_rotor_sensor_temp_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FL_BRAKE_ROTOR_SENSOR_TEMP_CANID;
  cframe->DLC = (uint8_t) FL_BRAKE_ROTOR_SENSOR_TEMP_DLC;
  cframe->IDE = (uint8_t) FL_BRAKE_ROTOR_SENSOR_TEMP_IDE;
  return FL_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#else

uint32_t Pack_FL_BRAKE_ROTOR_SENSOR_TEMP_hytech(FL_BRAKE_ROTOR_SENSOR_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FL_BRAKE_ROTOR_SENSOR_TEMP_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_ro = (uint16_t) HYTECH_brake_rotor_sensor_temp_ro_toS(_m->brake_rotor_sensor_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_rotor_sensor_temp_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_rotor_sensor_temp_ro & (0xFFU)) );

  *_len = (uint8_t) FL_BRAKE_ROTOR_SENSOR_TEMP_DLC;
  *_ide = (uint8_t) FL_BRAKE_ROTOR_SENSOR_TEMP_IDE;
  return FL_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_1_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_phys = (sigfloat_t)(HYTECH_brake_temp_channel_1_ro_fromS(_m->brake_temp_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_2_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_2_phys = (sigfloat_t)(HYTECH_brake_temp_channel_2_ro_fromS(_m->brake_temp_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_3_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_3_phys = (sigfloat_t)(HYTECH_brake_temp_channel_3_ro_fromS(_m->brake_temp_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_4_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_4_phys = (sigfloat_t)(HYTECH_brake_temp_channel_4_ro_fromS(_m->brake_temp_channel_4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(&_m->mon1, FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_ro = (uint16_t) HYTECH_brake_temp_channel_1_ro_toS(_m->brake_temp_channel_1_phys);
  _m->brake_temp_channel_2_ro = (uint16_t) HYTECH_brake_temp_channel_2_ro_toS(_m->brake_temp_channel_2_phys);
  _m->brake_temp_channel_3_ro = (uint16_t) HYTECH_brake_temp_channel_3_ro_toS(_m->brake_temp_channel_3_phys);
  _m->brake_temp_channel_4_ro = (uint16_t) HYTECH_brake_temp_channel_4_ro_toS(_m->brake_temp_channel_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_1_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_2_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_3_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_4_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
  cframe->DLC = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC;
  cframe->IDE = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH1_CH4_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#else

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH1_CH4_hytech(FR_BRAKE_ROTOR_TEMP_CH1_CH4_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_1_ro = (uint16_t) HYTECH_brake_temp_channel_1_ro_toS(_m->brake_temp_channel_1_phys);
  _m->brake_temp_channel_2_ro = (uint16_t) HYTECH_brake_temp_channel_2_ro_toS(_m->brake_temp_channel_2_phys);
  _m->brake_temp_channel_3_ro = (uint16_t) HYTECH_brake_temp_channel_3_ro_toS(_m->brake_temp_channel_3_phys);
  _m->brake_temp_channel_4_ro = (uint16_t) HYTECH_brake_temp_channel_4_ro_toS(_m->brake_temp_channel_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_1_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_1_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_2_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_2_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_3_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_3_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_4_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_4_ro & (0xFFU)) );

  *_len = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH1_CH4_DLC;
  *_ide = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH1_CH4_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH1_CH4_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_5_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_phys = (sigfloat_t)(HYTECH_brake_temp_channel_5_ro_fromS(_m->brake_temp_channel_5_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_6_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_6_phys = (sigfloat_t)(HYTECH_brake_temp_channel_6_ro_fromS(_m->brake_temp_channel_6_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_7_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_7_phys = (sigfloat_t)(HYTECH_brake_temp_channel_7_ro_fromS(_m->brake_temp_channel_7_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_8_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_8_phys = (sigfloat_t)(HYTECH_brake_temp_channel_8_ro_fromS(_m->brake_temp_channel_8_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(&_m->mon1, FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_ro = (uint16_t) HYTECH_brake_temp_channel_5_ro_toS(_m->brake_temp_channel_5_phys);
  _m->brake_temp_channel_6_ro = (uint16_t) HYTECH_brake_temp_channel_6_ro_toS(_m->brake_temp_channel_6_phys);
  _m->brake_temp_channel_7_ro = (uint16_t) HYTECH_brake_temp_channel_7_ro_toS(_m->brake_temp_channel_7_phys);
  _m->brake_temp_channel_8_ro = (uint16_t) HYTECH_brake_temp_channel_8_ro_toS(_m->brake_temp_channel_8_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_5_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_5_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_6_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_6_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_7_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_7_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_8_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_8_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
  cframe->DLC = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC;
  cframe->IDE = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH5_CH8_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#else

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH5_CH8_hytech(FR_BRAKE_ROTOR_TEMP_CH5_CH8_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_5_ro = (uint16_t) HYTECH_brake_temp_channel_5_ro_toS(_m->brake_temp_channel_5_phys);
  _m->brake_temp_channel_6_ro = (uint16_t) HYTECH_brake_temp_channel_6_ro_toS(_m->brake_temp_channel_6_phys);
  _m->brake_temp_channel_7_ro = (uint16_t) HYTECH_brake_temp_channel_7_ro_toS(_m->brake_temp_channel_7_phys);
  _m->brake_temp_channel_8_ro = (uint16_t) HYTECH_brake_temp_channel_8_ro_toS(_m->brake_temp_channel_8_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_5_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_5_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_6_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_6_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_7_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_7_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_8_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_8_ro & (0xFFU)) );

  *_len = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH5_CH8_DLC;
  *_ide = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH5_CH8_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH5_CH8_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_9_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_phys = (sigfloat_t)(HYTECH_brake_temp_channel_9_ro_fromS(_m->brake_temp_channel_9_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_10_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_10_phys = (sigfloat_t)(HYTECH_brake_temp_channel_10_ro_fromS(_m->brake_temp_channel_10_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_11_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_11_phys = (sigfloat_t)(HYTECH_brake_temp_channel_11_ro_fromS(_m->brake_temp_channel_11_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_12_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_12_phys = (sigfloat_t)(HYTECH_brake_temp_channel_12_ro_fromS(_m->brake_temp_channel_12_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(&_m->mon1, FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_ro = (uint16_t) HYTECH_brake_temp_channel_9_ro_toS(_m->brake_temp_channel_9_phys);
  _m->brake_temp_channel_10_ro = (uint16_t) HYTECH_brake_temp_channel_10_ro_toS(_m->brake_temp_channel_10_phys);
  _m->brake_temp_channel_11_ro = (uint16_t) HYTECH_brake_temp_channel_11_ro_toS(_m->brake_temp_channel_11_phys);
  _m->brake_temp_channel_12_ro = (uint16_t) HYTECH_brake_temp_channel_12_ro_toS(_m->brake_temp_channel_12_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_9_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_9_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_10_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_10_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_11_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_11_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_12_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_12_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
  cframe->DLC = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC;
  cframe->IDE = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH9_CH12_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#else

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH9_CH12_hytech(FR_BRAKE_ROTOR_TEMP_CH9_CH12_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_9_ro = (uint16_t) HYTECH_brake_temp_channel_9_ro_toS(_m->brake_temp_channel_9_phys);
  _m->brake_temp_channel_10_ro = (uint16_t) HYTECH_brake_temp_channel_10_ro_toS(_m->brake_temp_channel_10_phys);
  _m->brake_temp_channel_11_ro = (uint16_t) HYTECH_brake_temp_channel_11_ro_toS(_m->brake_temp_channel_11_phys);
  _m->brake_temp_channel_12_ro = (uint16_t) HYTECH_brake_temp_channel_12_ro_toS(_m->brake_temp_channel_12_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_9_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_9_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_10_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_10_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_11_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_11_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_12_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_12_ro & (0xFFU)) );

  *_len = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH9_CH12_DLC;
  *_ide = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH9_CH12_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH9_CH12_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_temp_channel_13_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_phys = (sigfloat_t)(HYTECH_brake_temp_channel_13_ro_fromS(_m->brake_temp_channel_13_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_14_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_14_phys = (sigfloat_t)(HYTECH_brake_temp_channel_14_ro_fromS(_m->brake_temp_channel_14_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_15_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_15_phys = (sigfloat_t)(HYTECH_brake_temp_channel_15_ro_fromS(_m->brake_temp_channel_15_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->brake_temp_channel_16_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_16_phys = (sigfloat_t)(HYTECH_brake_temp_channel_16_ro_fromS(_m->brake_temp_channel_16_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(&_m->mon1, FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_ro = (uint16_t) HYTECH_brake_temp_channel_13_ro_toS(_m->brake_temp_channel_13_phys);
  _m->brake_temp_channel_14_ro = (uint16_t) HYTECH_brake_temp_channel_14_ro_toS(_m->brake_temp_channel_14_phys);
  _m->brake_temp_channel_15_ro = (uint16_t) HYTECH_brake_temp_channel_15_ro_toS(_m->brake_temp_channel_15_phys);
  _m->brake_temp_channel_16_ro = (uint16_t) HYTECH_brake_temp_channel_16_ro_toS(_m->brake_temp_channel_16_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_temp_channel_13_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_temp_channel_13_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->brake_temp_channel_14_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->brake_temp_channel_14_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->brake_temp_channel_15_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->brake_temp_channel_15_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->brake_temp_channel_16_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->brake_temp_channel_16_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
  cframe->DLC = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC;
  cframe->IDE = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH13_CH16_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#else

uint32_t Pack_FR_BRAKE_ROTOR_TEMP_CH13_CH16_hytech(FR_BRAKE_ROTOR_TEMP_CH13_CH16_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_temp_channel_13_ro = (uint16_t) HYTECH_brake_temp_channel_13_ro_toS(_m->brake_temp_channel_13_phys);
  _m->brake_temp_channel_14_ro = (uint16_t) HYTECH_brake_temp_channel_14_ro_toS(_m->brake_temp_channel_14_phys);
  _m->brake_temp_channel_15_ro = (uint16_t) HYTECH_brake_temp_channel_15_ro_toS(_m->brake_temp_channel_15_phys);
  _m->brake_temp_channel_16_ro = (uint16_t) HYTECH_brake_temp_channel_16_ro_toS(_m->brake_temp_channel_16_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_temp_channel_13_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_temp_channel_13_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->brake_temp_channel_14_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->brake_temp_channel_14_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->brake_temp_channel_15_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->brake_temp_channel_15_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->brake_temp_channel_16_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->brake_temp_channel_16_ro & (0xFFU)) );

  *_len = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH13_CH16_DLC;
  *_ide = (uint8_t) FR_BRAKE_ROTOR_TEMP_CH13_CH16_IDE;
  return FR_BRAKE_ROTOR_TEMP_CH13_CH16_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->brake_rotor_sensor_temp_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_phys = (sigfloat_t)(HYTECH_brake_rotor_sensor_temp_ro_fromS(_m->brake_rotor_sensor_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < FR_BRAKE_ROTOR_SENSOR_TEMP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(&_m->mon1, FR_BRAKE_ROTOR_SENSOR_TEMP_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return FR_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_SENSOR_TEMP_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_ro = (uint16_t) HYTECH_brake_rotor_sensor_temp_ro_toS(_m->brake_rotor_sensor_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->brake_rotor_sensor_temp_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->brake_rotor_sensor_temp_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) FR_BRAKE_ROTOR_SENSOR_TEMP_CANID;
  cframe->DLC = (uint8_t) FR_BRAKE_ROTOR_SENSOR_TEMP_DLC;
  cframe->IDE = (uint8_t) FR_BRAKE_ROTOR_SENSOR_TEMP_IDE;
  return FR_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#else

uint32_t Pack_FR_BRAKE_ROTOR_SENSOR_TEMP_hytech(FR_BRAKE_ROTOR_SENSOR_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(FR_BRAKE_ROTOR_SENSOR_TEMP_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->brake_rotor_sensor_temp_ro = (uint16_t) HYTECH_brake_rotor_sensor_temp_ro_toS(_m->brake_rotor_sensor_temp_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->brake_rotor_sensor_temp_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->brake_rotor_sensor_temp_ro & (0xFFU)) );

  *_len = (uint8_t) FR_BRAKE_ROTOR_SENSOR_TEMP_DLC;
  *_ide = (uint8_t) FR_BRAKE_ROTOR_SENSOR_TEMP_IDE;
  return FR_BRAKE_ROTOR_SENSOR_TEMP_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_input_ro = (int32_t) __ext_sig__(( ((_d[2] & (0x0FU)) << 16U) | ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 20);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_input_phys = (sigfloat_t)(HYTECH_controller_input_ro_fromS(_m->controller_input_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_output_ro = (int32_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 12U) | ((_d[3] & (0xFFU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) ), 20);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_output_phys = (sigfloat_t)(HYTECH_controller_output_ro_fromS(_m->controller_output_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_PID_TV_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_PID_TV_DATA_hytech(&_m->mon1, CONTROLLER_PID_TV_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_PID_TV_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_TV_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_input_ro = (int32_t) HYTECH_controller_input_ro_toS(_m->controller_input_phys);
  _m->controller_output_ro = (int32_t) HYTECH_controller_output_ro_toS(_m->controller_output_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_input_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_input_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_input_ro >> 16U) & (0x0FU)) | ((_m->controller_output_ro & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_output_ro >> 4U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_output_ro >> 12U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_PID_TV_DATA_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_PID_TV_DATA_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_PID_TV_DATA_IDE;
  return CONTROLLER_PID_TV_DATA_CANID;
}

#else

uint32_t Pack_CONTROLLER_PID_TV_DATA_hytech(CONTROLLER_PID_TV_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_TV_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_input_ro = (int32_t) HYTECH_controller_input_ro_toS(_m->controller_input_phys);
  _m->controller_output_ro = (int32_t) HYTECH_controller_output_ro_toS(_m->controller_output_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_input_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_input_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->controller_input_ro >> 16U) & (0x0FU)) | ((_m->controller_output_ro & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->controller_output_ro >> 4U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->controller_output_ro >> 12U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_PID_TV_DATA_DLC;
  *_ide = (uint8_t) CONTROLLER_PID_TV_DATA_IDE;
  return CONTROLLER_PID_TV_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->pid_tv_fl_delta_ro = (int16_t) __ext_sig__(( ((_d[1] & (0x03U)) << 8U) | (_d[0] & (0xFFU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_fl_delta_phys = (sigfloat_t)(HYTECH_pid_tv_fl_delta_ro_fromS(_m->pid_tv_fl_delta_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->pid_tv_fr_delta_ro = (int16_t) __ext_sig__(( ((_d[2] & (0x0FU)) << 6U) | ((_d[1] >> 2U) & (0x3FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_fr_delta_phys = (sigfloat_t)(HYTECH_pid_tv_fr_delta_ro_fromS(_m->pid_tv_fr_delta_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->pid_tv_rl_delta_ro = (int16_t) __ext_sig__(( ((_d[3] & (0x3FU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_rl_delta_phys = (sigfloat_t)(HYTECH_pid_tv_rl_delta_ro_fromS(_m->pid_tv_rl_delta_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->pid_tv_rr_delta_ro = (int16_t) __ext_sig__(( ((_d[4] & (0xFFU)) << 2U) | ((_d[3] >> 6U) & (0x03U)) ), 10);
#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_rr_delta_phys = (sigfloat_t)(HYTECH_pid_tv_rr_delta_ro_fromS(_m->pid_tv_rr_delta_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_PID_TV_DELTA_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_PID_TV_DELTA_DATA_hytech(&_m->mon1, CONTROLLER_PID_TV_DELTA_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_PID_TV_DELTA_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_TV_DELTA_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_fl_delta_ro = (int16_t) HYTECH_pid_tv_fl_delta_ro_toS(_m->pid_tv_fl_delta_phys);
  _m->pid_tv_fr_delta_ro = (int16_t) HYTECH_pid_tv_fr_delta_ro_toS(_m->pid_tv_fr_delta_phys);
  _m->pid_tv_rl_delta_ro = (int16_t) HYTECH_pid_tv_rl_delta_ro_toS(_m->pid_tv_rl_delta_phys);
  _m->pid_tv_rr_delta_ro = (int16_t) HYTECH_pid_tv_rr_delta_ro_toS(_m->pid_tv_rr_delta_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->pid_tv_fl_delta_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->pid_tv_fl_delta_ro >> 8U) & (0x03U)) | ((_m->pid_tv_fr_delta_ro & (0x3FU)) << 2U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->pid_tv_fr_delta_ro >> 6U) & (0x0FU)) | ((_m->pid_tv_rl_delta_ro & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->pid_tv_rl_delta_ro >> 4U) & (0x3FU)) | ((_m->pid_tv_rr_delta_ro & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->pid_tv_rr_delta_ro >> 2U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_PID_TV_DELTA_DATA_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_PID_TV_DELTA_DATA_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_PID_TV_DELTA_DATA_IDE;
  return CONTROLLER_PID_TV_DELTA_DATA_CANID;
}

#else

uint32_t Pack_CONTROLLER_PID_TV_DELTA_DATA_hytech(CONTROLLER_PID_TV_DELTA_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_TV_DELTA_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->pid_tv_fl_delta_ro = (int16_t) HYTECH_pid_tv_fl_delta_ro_toS(_m->pid_tv_fl_delta_phys);
  _m->pid_tv_fr_delta_ro = (int16_t) HYTECH_pid_tv_fr_delta_ro_toS(_m->pid_tv_fr_delta_phys);
  _m->pid_tv_rl_delta_ro = (int16_t) HYTECH_pid_tv_rl_delta_ro_toS(_m->pid_tv_rl_delta_phys);
  _m->pid_tv_rr_delta_ro = (int16_t) HYTECH_pid_tv_rr_delta_ro_toS(_m->pid_tv_rr_delta_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->pid_tv_fl_delta_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->pid_tv_fl_delta_ro >> 8U) & (0x03U)) | ((_m->pid_tv_fr_delta_ro & (0x3FU)) << 2U) );
  _d[2] |= (uint8_t) ( ((_m->pid_tv_fr_delta_ro >> 6U) & (0x0FU)) | ((_m->pid_tv_rl_delta_ro & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->pid_tv_rl_delta_ro >> 4U) & (0x3FU)) | ((_m->pid_tv_rr_delta_ro & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( ((_m->pid_tv_rr_delta_ro >> 2U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_PID_TV_DELTA_DATA_DLC;
  *_ide = (uint8_t) CONTROLLER_PID_TV_DELTA_DATA_IDE;
  return CONTROLLER_PID_TV_DELTA_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->speed_above_thresh = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->torque_delta_above_thresh = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->tc_not_ready = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->steering_system_has_err = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->mode_intended = (uint8_t) ( ((_d[1] & (0x0FU)) << 4U) | ((_d[0] >> 4U) & (0x0FU)) );
  _m->mode_actual = (uint8_t) ( ((_d[2] & (0x0FU)) << 4U) | ((_d[1] >> 4U) & (0x0FU)) );
  _m->dash_dial_mode = (uint8_t) ( ((_d[3] & (0x0FU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) );
  _m->torque_mode = (uint8_t) ( ((_d[4] & (0x0FU)) << 4U) | ((_d[3] >> 4U) & (0x0FU)) );
  _m->torque_limit_ro = (uint16_t) ( ((_d[6] & (0x0FU)) << 12U) | ((_d[5] & (0xFFU)) << 4U) | ((_d[4] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_limit_phys = (sigfloat_t)(HYTECH_torque_limit_ro_fromS(_m->torque_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < TCMUX_STATUS_REPORT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_TCMUX_STATUS_REPORT_hytech(&_m->mon1, TCMUX_STATUS_REPORT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return TCMUX_STATUS_REPORT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCMUX_STATUS_REPORT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_limit_ro = (uint16_t) HYTECH_torque_limit_ro_toS(_m->torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->speed_above_thresh & (0x01U)) | ((_m->torque_delta_above_thresh & (0x01U)) << 1U) | ((_m->tc_not_ready & (0x01U)) << 2U) | ((_m->steering_system_has_err & (0x01U)) << 3U) | ((_m->mode_intended & (0x0FU)) << 4U) );
  cframe->Data[1] |= (uint8_t) ( ((_m->mode_intended >> 4U) & (0x0FU)) | ((_m->mode_actual & (0x0FU)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->mode_actual >> 4U) & (0x0FU)) | ((_m->dash_dial_mode & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->dash_dial_mode >> 4U) & (0x0FU)) | ((_m->torque_mode & (0x0FU)) << 4U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->torque_mode >> 4U) & (0x0FU)) | ((_m->torque_limit_ro & (0x0FU)) << 4U) );
  cframe->Data[5] |= (uint8_t) ( ((_m->torque_limit_ro >> 4U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->torque_limit_ro >> 12U) & (0x0FU)) );

  cframe->MsgId = (uint32_t) TCMUX_STATUS_REPORT_CANID;
  cframe->DLC = (uint8_t) TCMUX_STATUS_REPORT_DLC;
  cframe->IDE = (uint8_t) TCMUX_STATUS_REPORT_IDE;
  return TCMUX_STATUS_REPORT_CANID;
}

#else

uint32_t Pack_TCMUX_STATUS_REPORT_hytech(TCMUX_STATUS_REPORT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(TCMUX_STATUS_REPORT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->torque_limit_ro = (uint16_t) HYTECH_torque_limit_ro_toS(_m->torque_limit_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->speed_above_thresh & (0x01U)) | ((_m->torque_delta_above_thresh & (0x01U)) << 1U) | ((_m->tc_not_ready & (0x01U)) << 2U) | ((_m->steering_system_has_err & (0x01U)) << 3U) | ((_m->mode_intended & (0x0FU)) << 4U) );
  _d[1] |= (uint8_t) ( ((_m->mode_intended >> 4U) & (0x0FU)) | ((_m->mode_actual & (0x0FU)) << 4U) );
  _d[2] |= (uint8_t) ( ((_m->mode_actual >> 4U) & (0x0FU)) | ((_m->dash_dial_mode & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->dash_dial_mode >> 4U) & (0x0FU)) | ((_m->torque_mode & (0x0FU)) << 4U) );
  _d[4] |= (uint8_t) ( ((_m->torque_mode >> 4U) & (0x0FU)) | ((_m->torque_limit_ro & (0x0FU)) << 4U) );
  _d[5] |= (uint8_t) ( ((_m->torque_limit_ro >> 4U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->torque_limit_ro >> 12U) & (0x0FU)) );

  *_len = (uint8_t) TCMUX_STATUS_REPORT_DLC;
  *_ide = (uint8_t) TCMUX_STATUS_REPORT_IDE;
  return TCMUX_STATUS_REPORT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ACU_OK_hytech(ACU_OK_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->bms_ok = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->imd_ok = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->latch_ok = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ACU_OK_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ACU_OK_hytech(&_m->mon1, ACU_OK_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return ACU_OK_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_ACU_OK_hytech(ACU_OK_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ACU_OK_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->bms_ok & (0x01U)) | ((_m->imd_ok & (0x01U)) << 1U) | ((_m->latch_ok & (0x01U)) << 2U) );

  cframe->MsgId = (uint32_t) ACU_OK_CANID;
  cframe->DLC = (uint8_t) ACU_OK_DLC;
  cframe->IDE = (uint8_t) ACU_OK_IDE;
  return ACU_OK_CANID;
}

#else

uint32_t Pack_ACU_OK_hytech(ACU_OK_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ACU_OK_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->bms_ok & (0x01U)) | ((_m->imd_ok & (0x01U)) << 1U) | ((_m->latch_ok & (0x01U)) << 2U) );

  *_len = (uint8_t) ACU_OK_DLC;
  *_ide = (uint8_t) ACU_OK_IDE;
  return ACU_OK_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->em_temp_sig_index = (uint8_t) ( (_d[0] & (0x07U)) );
  _m->em_num_sensors = (uint8_t) ( ((_d[0] >> 3U) & (0x1FU)) );
  _m->em_min_temp_ro = (uint8_t) ( (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_min_temp_phys = (sigfloat_t)(HYTECH_em_min_temp_ro_fromS(_m->em_min_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_max_temp_ro = (uint8_t) ( (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_max_temp_phys = (sigfloat_t)(HYTECH_em_max_temp_ro_fromS(_m->em_max_temp_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_temp_0_ro = (uint8_t) ( (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_temp_0_phys = (sigfloat_t)(HYTECH_em_temp_0_ro_fromS(_m->em_temp_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_temp_1_ro = (uint8_t) ( (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_temp_1_phys = (sigfloat_t)(HYTECH_em_temp_1_ro_fromS(_m->em_temp_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_temp_2_ro = (uint8_t) ( (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_temp_2_phys = (sigfloat_t)(HYTECH_em_temp_2_ro_fromS(_m->em_temp_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_temp_3_ro = (uint8_t) ( (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_temp_3_phys = (sigfloat_t)(HYTECH_em_temp_3_ro_fromS(_m->em_temp_3_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->em_temp_4_ro = (uint8_t) ( (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->em_temp_4_phys = (sigfloat_t)(HYTECH_em_temp_4_ro_fromS(_m->em_temp_4_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < ENERGY_METER_TEMP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_ENERGY_METER_TEMP_hytech(&_m->mon1, ENERGY_METER_TEMP_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return ENERGY_METER_TEMP_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_TEMP_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->em_min_temp_ro = (uint8_t) HYTECH_em_min_temp_ro_toS(_m->em_min_temp_phys);
  _m->em_max_temp_ro = (uint8_t) HYTECH_em_max_temp_ro_toS(_m->em_max_temp_phys);
  _m->em_temp_0_ro = (uint8_t) HYTECH_em_temp_0_ro_toS(_m->em_temp_0_phys);
  _m->em_temp_1_ro = (uint8_t) HYTECH_em_temp_1_ro_toS(_m->em_temp_1_phys);
  _m->em_temp_2_ro = (uint8_t) HYTECH_em_temp_2_ro_toS(_m->em_temp_2_phys);
  _m->em_temp_3_ro = (uint8_t) HYTECH_em_temp_3_ro_toS(_m->em_temp_3_phys);
  _m->em_temp_4_ro = (uint8_t) HYTECH_em_temp_4_ro_toS(_m->em_temp_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->em_temp_sig_index & (0x07U)) | ((_m->em_num_sensors & (0x1FU)) << 3U) );
  cframe->Data[1] |= (uint8_t) ( (_m->em_min_temp_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->em_max_temp_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->em_temp_0_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->em_temp_1_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->em_temp_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->em_temp_3_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->em_temp_4_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) ENERGY_METER_TEMP_CANID;
  cframe->DLC = (uint8_t) ENERGY_METER_TEMP_DLC;
  cframe->IDE = (uint8_t) ENERGY_METER_TEMP_IDE;
  return ENERGY_METER_TEMP_CANID;
}

#else

uint32_t Pack_ENERGY_METER_TEMP_hytech(ENERGY_METER_TEMP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(ENERGY_METER_TEMP_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->em_min_temp_ro = (uint8_t) HYTECH_em_min_temp_ro_toS(_m->em_min_temp_phys);
  _m->em_max_temp_ro = (uint8_t) HYTECH_em_max_temp_ro_toS(_m->em_max_temp_phys);
  _m->em_temp_0_ro = (uint8_t) HYTECH_em_temp_0_ro_toS(_m->em_temp_0_phys);
  _m->em_temp_1_ro = (uint8_t) HYTECH_em_temp_1_ro_toS(_m->em_temp_1_phys);
  _m->em_temp_2_ro = (uint8_t) HYTECH_em_temp_2_ro_toS(_m->em_temp_2_phys);
  _m->em_temp_3_ro = (uint8_t) HYTECH_em_temp_3_ro_toS(_m->em_temp_3_phys);
  _m->em_temp_4_ro = (uint8_t) HYTECH_em_temp_4_ro_toS(_m->em_temp_4_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->em_temp_sig_index & (0x07U)) | ((_m->em_num_sensors & (0x1FU)) << 3U) );
  _d[1] |= (uint8_t) ( (_m->em_min_temp_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->em_max_temp_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->em_temp_0_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->em_temp_1_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->em_temp_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->em_temp_3_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->em_temp_4_ro & (0xFFU)) );

  *_len = (uint8_t) ENERGY_METER_TEMP_DLC;
  *_ide = (uint8_t) ENERGY_METER_TEMP_IDE;
  return ENERGY_METER_TEMP_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_0_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_phys = (sigfloat_t)(HYTECH_aero_channel_0_ro_fromS(_m->aero_channel_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_1_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_1_phys = (sigfloat_t)(HYTECH_aero_channel_1_ro_fromS(_m->aero_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_2_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_2_phys = (sigfloat_t)(HYTECH_aero_channel_2_ro_fromS(_m->aero_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_3_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_3_phys = (sigfloat_t)(HYTECH_aero_channel_3_ro_fromS(_m->aero_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_21_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_21_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_21_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_21_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_21_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_21_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_21_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_21_IDE;
  return AERO_PRESSURE_SENSOR_21_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_21_hytech(AERO_PRESSURE_SENSOR_21_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_21_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_21_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_21_IDE;
  return AERO_PRESSURE_SENSOR_21_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_0_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_phys = (sigfloat_t)(HYTECH_aero_channel_0_ro_fromS(_m->aero_channel_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_1_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_1_phys = (sigfloat_t)(HYTECH_aero_channel_1_ro_fromS(_m->aero_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_2_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_2_phys = (sigfloat_t)(HYTECH_aero_channel_2_ro_fromS(_m->aero_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_3_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_3_phys = (sigfloat_t)(HYTECH_aero_channel_3_ro_fromS(_m->aero_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_31_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_31_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_31_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_31_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_31_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_31_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_31_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_31_IDE;
  return AERO_PRESSURE_SENSOR_31_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_31_hytech(AERO_PRESSURE_SENSOR_31_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_31_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_31_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_31_IDE;
  return AERO_PRESSURE_SENSOR_31_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->aero_channel_0_ro = (uint16_t) ( ((_d[0] & (0xFFU)) << 8U) | (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_phys = (sigfloat_t)(HYTECH_aero_channel_0_ro_fromS(_m->aero_channel_0_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_1_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 8U) | (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_1_phys = (sigfloat_t)(HYTECH_aero_channel_1_ro_fromS(_m->aero_channel_1_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_2_ro = (uint16_t) ( ((_d[4] & (0xFFU)) << 8U) | (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_2_phys = (sigfloat_t)(HYTECH_aero_channel_2_ro_fromS(_m->aero_channel_2_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->aero_channel_3_ro = (uint16_t) ( ((_d[6] & (0xFFU)) << 8U) | (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_3_phys = (sigfloat_t)(HYTECH_aero_channel_3_ro_fromS(_m->aero_channel_3_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < AERO_PRESSURE_SENSOR_41_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_AERO_PRESSURE_SENSOR_41_hytech(&_m->mon1, AERO_PRESSURE_SENSOR_41_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return AERO_PRESSURE_SENSOR_41_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_41_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) AERO_PRESSURE_SENSOR_41_CANID;
  cframe->DLC = (uint8_t) AERO_PRESSURE_SENSOR_41_DLC;
  cframe->IDE = (uint8_t) AERO_PRESSURE_SENSOR_41_IDE;
  return AERO_PRESSURE_SENSOR_41_CANID;
}

#else

uint32_t Pack_AERO_PRESSURE_SENSOR_41_hytech(AERO_PRESSURE_SENSOR_41_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(AERO_PRESSURE_SENSOR_41_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->aero_channel_0_ro = (uint16_t) HYTECH_aero_channel_0_ro_toS(_m->aero_channel_0_phys);
  _m->aero_channel_1_ro = (uint16_t) HYTECH_aero_channel_1_ro_toS(_m->aero_channel_1_phys);
  _m->aero_channel_2_ro = (uint16_t) HYTECH_aero_channel_2_ro_toS(_m->aero_channel_2_phys);
  _m->aero_channel_3_ro = (uint16_t) HYTECH_aero_channel_3_ro_toS(_m->aero_channel_3_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( ((_m->aero_channel_0_ro >> 8U) & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->aero_channel_0_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->aero_channel_1_ro >> 8U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->aero_channel_1_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->aero_channel_2_ro >> 8U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->aero_channel_2_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->aero_channel_3_ro >> 8U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->aero_channel_3_ro & (0xFFU)) );

  *_len = (uint8_t) AERO_PRESSURE_SENSOR_41_DLC;
  *_ide = (uint8_t) AERO_PRESSURE_SENSOR_41_IDE;
  return AERO_PRESSURE_SENSOR_41_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_pid_output_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_fl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_output_fl_ro_fromS(_m->controller_tcs_pid_output_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_output_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_output_fr_ro_fromS(_m->controller_tcs_pid_output_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_output_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_rl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_output_rl_ro_fromS(_m->controller_tcs_pid_output_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_output_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_output_rr_ro_fromS(_m->controller_tcs_pid_output_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_PID_OUTPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_PID_OUTPUT_hytech(&_m->mon1, CONTROLLER_TCS_PID_OUTPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_PID_OUTPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_OUTPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_fl_ro = (int16_t) HYTECH_controller_tcs_pid_output_fl_ro_toS(_m->controller_tcs_pid_output_fl_phys);
  _m->controller_tcs_pid_output_fr_ro = (int16_t) HYTECH_controller_tcs_pid_output_fr_ro_toS(_m->controller_tcs_pid_output_fr_phys);
  _m->controller_tcs_pid_output_rl_ro = (int16_t) HYTECH_controller_tcs_pid_output_rl_ro_toS(_m->controller_tcs_pid_output_rl_phys);
  _m->controller_tcs_pid_output_rr_ro = (int16_t) HYTECH_controller_tcs_pid_output_rr_ro_toS(_m->controller_tcs_pid_output_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_pid_output_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_pid_output_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_pid_output_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_pid_output_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_tcs_pid_output_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_pid_output_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_tcs_pid_output_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_pid_output_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_PID_OUTPUT_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_PID_OUTPUT_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_PID_OUTPUT_IDE;
  return CONTROLLER_TCS_PID_OUTPUT_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_PID_OUTPUT_hytech(CONTROLLER_TCS_PID_OUTPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_OUTPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_output_fl_ro = (int16_t) HYTECH_controller_tcs_pid_output_fl_ro_toS(_m->controller_tcs_pid_output_fl_phys);
  _m->controller_tcs_pid_output_fr_ro = (int16_t) HYTECH_controller_tcs_pid_output_fr_ro_toS(_m->controller_tcs_pid_output_fr_phys);
  _m->controller_tcs_pid_output_rl_ro = (int16_t) HYTECH_controller_tcs_pid_output_rl_ro_toS(_m->controller_tcs_pid_output_rl_phys);
  _m->controller_tcs_pid_output_rr_ro = (int16_t) HYTECH_controller_tcs_pid_output_rr_ro_toS(_m->controller_tcs_pid_output_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_pid_output_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_pid_output_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_pid_output_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_pid_output_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_tcs_pid_output_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_pid_output_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_tcs_pid_output_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_pid_output_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_PID_OUTPUT_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_PID_OUTPUT_IDE;
  return CONTROLLER_TCS_PID_OUTPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_wheel_lin_vel_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_fl_phys = (sigfloat_t)(HYTECH_vehm_wheel_lin_vel_fl_ro_fromS(_m->vehm_wheel_lin_vel_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_wheel_lin_vel_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_fr_phys = (sigfloat_t)(HYTECH_vehm_wheel_lin_vel_fr_ro_fromS(_m->vehm_wheel_lin_vel_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_wheel_lin_vel_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_rl_phys = (sigfloat_t)(HYTECH_vehm_wheel_lin_vel_rl_ro_fromS(_m->vehm_wheel_lin_vel_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_wheel_lin_vel_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_rr_phys = (sigfloat_t)(HYTECH_vehm_wheel_lin_vel_rr_ro_fromS(_m->vehm_wheel_lin_vel_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_WHEEL_LIN_VEL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_WHEEL_LIN_VEL_hytech(&_m->mon1, VEHM_WHEEL_LIN_VEL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_WHEEL_LIN_VEL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_WHEEL_LIN_VEL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_fl_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_fl_ro_toS(_m->vehm_wheel_lin_vel_fl_phys);
  _m->vehm_wheel_lin_vel_fr_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_fr_ro_toS(_m->vehm_wheel_lin_vel_fr_phys);
  _m->vehm_wheel_lin_vel_rl_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_rl_ro_toS(_m->vehm_wheel_lin_vel_rl_phys);
  _m->vehm_wheel_lin_vel_rr_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_rr_ro_toS(_m->vehm_wheel_lin_vel_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_WHEEL_LIN_VEL_CANID;
  cframe->DLC = (uint8_t) VEHM_WHEEL_LIN_VEL_DLC;
  cframe->IDE = (uint8_t) VEHM_WHEEL_LIN_VEL_IDE;
  return VEHM_WHEEL_LIN_VEL_CANID;
}

#else

uint32_t Pack_VEHM_WHEEL_LIN_VEL_hytech(VEHM_WHEEL_LIN_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_WHEEL_LIN_VEL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_lin_vel_fl_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_fl_ro_toS(_m->vehm_wheel_lin_vel_fl_phys);
  _m->vehm_wheel_lin_vel_fr_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_fr_ro_toS(_m->vehm_wheel_lin_vel_fr_phys);
  _m->vehm_wheel_lin_vel_rl_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_rl_ro_toS(_m->vehm_wheel_lin_vel_rl_phys);
  _m->vehm_wheel_lin_vel_rr_ro = (int16_t) HYTECH_vehm_wheel_lin_vel_rr_ro_toS(_m->vehm_wheel_lin_vel_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->vehm_wheel_lin_vel_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vehm_wheel_lin_vel_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_WHEEL_LIN_VEL_DLC;
  *_ide = (uint8_t) VEHM_WHEEL_LIN_VEL_IDE;
  return VEHM_WHEEL_LIN_VEL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_bias_torq_avg_front_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_bias_torq_avg_front_phys = (sigfloat_t)(HYTECH_controller_bias_torq_avg_front_ro_fromS(_m->controller_bias_torq_avg_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_bias_torq_avg_rear_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_bias_torq_avg_rear_phys = (sigfloat_t)(HYTECH_controller_bias_torq_avg_rear_ro_fromS(_m->controller_bias_torq_avg_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TORQUE_BIAS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TORQUE_BIAS_hytech(&_m->mon1, CONTROLLER_TORQUE_BIAS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TORQUE_BIAS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TORQUE_BIAS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_bias_torq_avg_front_ro = (int16_t) HYTECH_controller_bias_torq_avg_front_ro_toS(_m->controller_bias_torq_avg_front_phys);
  _m->controller_bias_torq_avg_rear_ro = (int16_t) HYTECH_controller_bias_torq_avg_rear_ro_toS(_m->controller_bias_torq_avg_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_bias_torq_avg_front_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_bias_torq_avg_front_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_bias_torq_avg_rear_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_bias_torq_avg_rear_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TORQUE_BIAS_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TORQUE_BIAS_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TORQUE_BIAS_IDE;
  return CONTROLLER_TORQUE_BIAS_CANID;
}

#else

uint32_t Pack_CONTROLLER_TORQUE_BIAS_hytech(CONTROLLER_TORQUE_BIAS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TORQUE_BIAS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_bias_torq_avg_front_ro = (int16_t) HYTECH_controller_bias_torq_avg_front_ro_toS(_m->controller_bias_torq_avg_front_phys);
  _m->controller_bias_torq_avg_rear_ro = (int16_t) HYTECH_controller_bias_torq_avg_rear_ro_toS(_m->controller_bias_torq_avg_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_bias_torq_avg_front_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_bias_torq_avg_front_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_bias_torq_avg_rear_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_bias_torq_avg_rear_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TORQUE_BIAS_DLC;
  *_ide = (uint8_t) CONTROLLER_TORQUE_BIAS_IDE;
  return CONTROLLER_TORQUE_BIAS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_regen_5kph_torq_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_fl_phys = (sigfloat_t)(HYTECH_controller_regen_5kph_torq_fl_ro_fromS(_m->controller_regen_5kph_torq_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_regen_5kph_torq_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_fr_phys = (sigfloat_t)(HYTECH_controller_regen_5kph_torq_fr_ro_fromS(_m->controller_regen_5kph_torq_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_regen_5kph_torq_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_rl_phys = (sigfloat_t)(HYTECH_controller_regen_5kph_torq_rl_ro_fromS(_m->controller_regen_5kph_torq_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_regen_5kph_torq_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_rr_phys = (sigfloat_t)(HYTECH_controller_regen_5kph_torq_rr_ro_fromS(_m->controller_regen_5kph_torq_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_REGEN_5KPH_TORQUE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_REGEN_5KPH_TORQUE_hytech(&_m->mon1, CONTROLLER_REGEN_5KPH_TORQUE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_REGEN_5KPH_TORQUE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_REGEN_5KPH_TORQUE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_fl_ro = (int16_t) HYTECH_controller_regen_5kph_torq_fl_ro_toS(_m->controller_regen_5kph_torq_fl_phys);
  _m->controller_regen_5kph_torq_fr_ro = (int16_t) HYTECH_controller_regen_5kph_torq_fr_ro_toS(_m->controller_regen_5kph_torq_fr_phys);
  _m->controller_regen_5kph_torq_rl_ro = (int16_t) HYTECH_controller_regen_5kph_torq_rl_ro_toS(_m->controller_regen_5kph_torq_rl_phys);
  _m->controller_regen_5kph_torq_rr_ro = (int16_t) HYTECH_controller_regen_5kph_torq_rr_ro_toS(_m->controller_regen_5kph_torq_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_regen_5kph_torq_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_regen_5kph_torq_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_regen_5kph_torq_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_regen_5kph_torq_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_REGEN_5KPH_TORQUE_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_REGEN_5KPH_TORQUE_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_REGEN_5KPH_TORQUE_IDE;
  return CONTROLLER_REGEN_5KPH_TORQUE_CANID;
}

#else

uint32_t Pack_CONTROLLER_REGEN_5KPH_TORQUE_hytech(CONTROLLER_REGEN_5KPH_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_REGEN_5KPH_TORQUE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_5kph_torq_fl_ro = (int16_t) HYTECH_controller_regen_5kph_torq_fl_ro_toS(_m->controller_regen_5kph_torq_fl_phys);
  _m->controller_regen_5kph_torq_fr_ro = (int16_t) HYTECH_controller_regen_5kph_torq_fr_ro_toS(_m->controller_regen_5kph_torq_fr_phys);
  _m->controller_regen_5kph_torq_rl_ro = (int16_t) HYTECH_controller_regen_5kph_torq_rl_ro_toS(_m->controller_regen_5kph_torq_rl_phys);
  _m->controller_regen_5kph_torq_rr_ro = (int16_t) HYTECH_controller_regen_5kph_torq_rr_ro_toS(_m->controller_regen_5kph_torq_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_regen_5kph_torq_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_regen_5kph_torq_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_regen_5kph_torq_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_regen_5kph_torq_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_regen_5kph_torq_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_REGEN_5KPH_TORQUE_DLC;
  *_ide = (uint8_t) CONTROLLER_REGEN_5KPH_TORQUE_IDE;
  return CONTROLLER_REGEN_5KPH_TORQUE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_regen_5kph_status_fl = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->controller_regen_5kph_status_fr = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->controller_regen_5kph_status_rl = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->controller_regen_5kph_status_rr = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_REGEN_5KPH_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_REGEN_5KPH_STATUS_hytech(&_m->mon1, CONTROLLER_REGEN_5KPH_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_REGEN_5KPH_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_REGEN_5KPH_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->controller_regen_5kph_status_fl & (0x01U)) | ((_m->controller_regen_5kph_status_fr & (0x01U)) << 1U) | ((_m->controller_regen_5kph_status_rl & (0x01U)) << 2U) | ((_m->controller_regen_5kph_status_rr & (0x01U)) << 3U) );

  cframe->MsgId = (uint32_t) CONTROLLER_REGEN_5KPH_STATUS_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_REGEN_5KPH_STATUS_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_REGEN_5KPH_STATUS_IDE;
  return CONTROLLER_REGEN_5KPH_STATUS_CANID;
}

#else

uint32_t Pack_CONTROLLER_REGEN_5KPH_STATUS_hytech(CONTROLLER_REGEN_5KPH_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_REGEN_5KPH_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->controller_regen_5kph_status_fl & (0x01U)) | ((_m->controller_regen_5kph_status_fr & (0x01U)) << 1U) | ((_m->controller_regen_5kph_status_rl & (0x01U)) << 2U) | ((_m->controller_regen_5kph_status_rr & (0x01U)) << 3U) );

  *_len = (uint8_t) CONTROLLER_REGEN_5KPH_STATUS_DLC;
  *_ide = (uint8_t) CONTROLLER_REGEN_5KPH_STATUS_IDE;
  return CONTROLLER_REGEN_5KPH_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_pid_p_fl_ro = (uint16_t) ( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_fl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_p_fl_ro_fromS(_m->controller_tcs_pid_p_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_p_fr_ro = (uint16_t) ( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_p_fr_ro_fromS(_m->controller_tcs_pid_p_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_p_rl_ro = (uint16_t) ( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_rl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_p_rl_ro_fromS(_m->controller_tcs_pid_p_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_p_rr_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_p_rr_ro_fromS(_m->controller_tcs_pid_p_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_PID_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_PID_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_PID_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_PID_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_fl_ro = (uint16_t) HYTECH_controller_tcs_pid_p_fl_ro_toS(_m->controller_tcs_pid_p_fl_phys);
  _m->controller_tcs_pid_p_fr_ro = (uint16_t) HYTECH_controller_tcs_pid_p_fr_ro_toS(_m->controller_tcs_pid_p_fr_phys);
  _m->controller_tcs_pid_p_rl_ro = (uint16_t) HYTECH_controller_tcs_pid_p_rl_ro_toS(_m->controller_tcs_pid_p_rl_phys);
  _m->controller_tcs_pid_p_rr_ro = (uint16_t) HYTECH_controller_tcs_pid_p_rr_ro_toS(_m->controller_tcs_pid_p_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_pid_p_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_pid_p_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_pid_p_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_pid_p_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_tcs_pid_p_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_pid_p_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_tcs_pid_p_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_pid_p_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_PID_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_PID_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_PID_CONFIG_IDE;
  return CONTROLLER_TCS_PID_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_PID_CONFIG_hytech(CONTROLLER_TCS_PID_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_p_fl_ro = (uint16_t) HYTECH_controller_tcs_pid_p_fl_ro_toS(_m->controller_tcs_pid_p_fl_phys);
  _m->controller_tcs_pid_p_fr_ro = (uint16_t) HYTECH_controller_tcs_pid_p_fr_ro_toS(_m->controller_tcs_pid_p_fr_phys);
  _m->controller_tcs_pid_p_rl_ro = (uint16_t) HYTECH_controller_tcs_pid_p_rl_ro_toS(_m->controller_tcs_pid_p_rl_phys);
  _m->controller_tcs_pid_p_rr_ro = (uint16_t) HYTECH_controller_tcs_pid_p_rr_ro_toS(_m->controller_tcs_pid_p_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_pid_p_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_pid_p_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_pid_p_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_pid_p_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_tcs_pid_p_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_pid_p_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_tcs_pid_p_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_pid_p_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_PID_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_PID_CONFIG_IDE;
  return CONTROLLER_TCS_PID_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_yaw_pid_p_ro = (uint16_t) ( ((_d[1] & (0x1FU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_p_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_p_ro_fromS(_m->controller_yaw_pid_p_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_i_ro = (uint16_t) ( ((_d[2] & (0x7FU)) << 3U) | ((_d[1] >> 5U) & (0x07U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_i_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_i_ro_fromS(_m->controller_yaw_pid_i_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_d_ro = (uint16_t) ( ((_d[4] & (0x01U)) << 9U) | ((_d[3] & (0xFFU)) << 1U) | ((_d[2] >> 7U) & (0x01U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_d_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_d_ro_fromS(_m->controller_yaw_pid_d_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_brakes_p_ro = (uint16_t) ( ((_d[5] & (0x0FU)) << 7U) | ((_d[4] >> 1U) & (0x7FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_brakes_p_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_brakes_p_ro_fromS(_m->controller_yaw_pid_brakes_p_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_brakes_i_ro = (uint16_t) ( ((_d[6] & (0x3FU)) << 4U) | ((_d[5] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_brakes_i_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_brakes_i_ro_fromS(_m->controller_yaw_pid_brakes_i_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_brakes_d_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 2U) | ((_d[6] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_brakes_d_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_brakes_d_ro_fromS(_m->controller_yaw_pid_brakes_d_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_YAW_PID_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_YAW_PID_CONFIG_hytech(&_m->mon1, CONTROLLER_YAW_PID_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_YAW_PID_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_YAW_PID_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_p_ro = (uint16_t) HYTECH_controller_yaw_pid_p_ro_toS(_m->controller_yaw_pid_p_phys);
  _m->controller_yaw_pid_i_ro = (uint16_t) HYTECH_controller_yaw_pid_i_ro_toS(_m->controller_yaw_pid_i_phys);
  _m->controller_yaw_pid_d_ro = (uint16_t) HYTECH_controller_yaw_pid_d_ro_toS(_m->controller_yaw_pid_d_phys);
  _m->controller_yaw_pid_brakes_p_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_p_ro_toS(_m->controller_yaw_pid_brakes_p_phys);
  _m->controller_yaw_pid_brakes_i_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_i_ro_toS(_m->controller_yaw_pid_brakes_i_phys);
  _m->controller_yaw_pid_brakes_d_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_d_ro_toS(_m->controller_yaw_pid_brakes_d_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_yaw_pid_p_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_yaw_pid_p_ro >> 8U) & (0x1FU)) | ((_m->controller_yaw_pid_i_ro & (0x07U)) << 5U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_yaw_pid_i_ro >> 3U) & (0x7FU)) | ((_m->controller_yaw_pid_d_ro & (0x01U)) << 7U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_yaw_pid_d_ro >> 1U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_yaw_pid_d_ro >> 9U) & (0x01U)) | ((_m->controller_yaw_pid_brakes_p_ro & (0x7FU)) << 1U) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_p_ro >> 7U) & (0x0FU)) | ((_m->controller_yaw_pid_brakes_i_ro & (0x0FU)) << 4U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_i_ro >> 4U) & (0x3FU)) | ((_m->controller_yaw_pid_brakes_d_ro & (0x03U)) << 6U) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_d_ro >> 2U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_YAW_PID_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_YAW_PID_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_YAW_PID_CONFIG_IDE;
  return CONTROLLER_YAW_PID_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_YAW_PID_CONFIG_hytech(CONTROLLER_YAW_PID_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_YAW_PID_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_p_ro = (uint16_t) HYTECH_controller_yaw_pid_p_ro_toS(_m->controller_yaw_pid_p_phys);
  _m->controller_yaw_pid_i_ro = (uint16_t) HYTECH_controller_yaw_pid_i_ro_toS(_m->controller_yaw_pid_i_phys);
  _m->controller_yaw_pid_d_ro = (uint16_t) HYTECH_controller_yaw_pid_d_ro_toS(_m->controller_yaw_pid_d_phys);
  _m->controller_yaw_pid_brakes_p_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_p_ro_toS(_m->controller_yaw_pid_brakes_p_phys);
  _m->controller_yaw_pid_brakes_i_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_i_ro_toS(_m->controller_yaw_pid_brakes_i_phys);
  _m->controller_yaw_pid_brakes_d_ro = (uint16_t) HYTECH_controller_yaw_pid_brakes_d_ro_toS(_m->controller_yaw_pid_brakes_d_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_yaw_pid_p_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_yaw_pid_p_ro >> 8U) & (0x1FU)) | ((_m->controller_yaw_pid_i_ro & (0x07U)) << 5U) );
  _d[2] |= (uint8_t) ( ((_m->controller_yaw_pid_i_ro >> 3U) & (0x7FU)) | ((_m->controller_yaw_pid_d_ro & (0x01U)) << 7U) );
  _d[3] |= (uint8_t) ( ((_m->controller_yaw_pid_d_ro >> 1U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->controller_yaw_pid_d_ro >> 9U) & (0x01U)) | ((_m->controller_yaw_pid_brakes_p_ro & (0x7FU)) << 1U) );
  _d[5] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_p_ro >> 7U) & (0x0FU)) | ((_m->controller_yaw_pid_brakes_i_ro & (0x0FU)) << 4U) );
  _d[6] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_i_ro >> 4U) & (0x3FU)) | ((_m->controller_yaw_pid_brakes_d_ro & (0x03U)) << 6U) );
  _d[7] |= (uint8_t) ( ((_m->controller_yaw_pid_brakes_d_ro >> 2U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_YAW_PID_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_YAW_PID_CONFIG_IDE;
  return CONTROLLER_YAW_PID_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_saturation_front_ro = (uint16_t) ( ((_d[1] & (0x0FU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_saturation_front_phys = (sigfloat_t)(HYTECH_controller_tcs_saturation_front_ro_fromS(_m->controller_tcs_saturation_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_saturation_rear_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 4U) | ((_d[1] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_saturation_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_saturation_rear_ro_fromS(_m->controller_tcs_saturation_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_SATURATION_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_SATURATION_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_SATURATION_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_SATURATION_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_SATURATION_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_saturation_front_ro = (uint16_t) HYTECH_controller_tcs_saturation_front_ro_toS(_m->controller_tcs_saturation_front_phys);
  _m->controller_tcs_saturation_rear_ro = (uint16_t) HYTECH_controller_tcs_saturation_rear_ro_toS(_m->controller_tcs_saturation_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_saturation_front_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_saturation_front_ro >> 8U) & (0x0FU)) | ((_m->controller_tcs_saturation_rear_ro & (0x0FU)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_tcs_saturation_rear_ro >> 4U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_SATURATION_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_SATURATION_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_SATURATION_CONFIG_IDE;
  return CONTROLLER_TCS_SATURATION_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_SATURATION_CONFIG_hytech(CONTROLLER_TCS_SATURATION_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_SATURATION_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_saturation_front_ro = (uint16_t) HYTECH_controller_tcs_saturation_front_ro_toS(_m->controller_tcs_saturation_front_phys);
  _m->controller_tcs_saturation_rear_ro = (uint16_t) HYTECH_controller_tcs_saturation_rear_ro_toS(_m->controller_tcs_saturation_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_saturation_front_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_saturation_front_ro >> 8U) & (0x0FU)) | ((_m->controller_tcs_saturation_rear_ro & (0x0FU)) << 4U) );
  _d[2] |= (uint8_t) ( ((_m->controller_tcs_saturation_rear_ro >> 4U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_SATURATION_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_SATURATION_CONFIG_IDE;
  return CONTROLLER_TCS_SATURATION_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_lauSL_start_front_ro = (uint8_t) ( (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_start_front_phys = (sigfloat_t)(HYTECH_controller_tcs_lauSL_start_front_ro_fromS(_m->controller_tcs_lauSL_start_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_lauSL_end_front_ro = (uint8_t) ( (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_end_front_phys = (sigfloat_t)(HYTECH_controller_tcs_lauSL_end_front_ro_fromS(_m->controller_tcs_lauSL_end_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_lauSL_start_rear_ro = (uint8_t) ( (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_start_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_lauSL_start_rear_ro_fromS(_m->controller_tcs_lauSL_start_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_lauSL_end_rear_ro = (uint8_t) ( (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_end_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_lauSL_end_rear_ro_fromS(_m->controller_tcs_lauSL_end_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_SL_start_front_ro = (uint8_t) ( (_d[4] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_SL_start_front_phys = (sigfloat_t)(HYTECH_controller_tcs_SL_start_front_ro_fromS(_m->controller_tcs_SL_start_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_SL_end_front_ro = (uint8_t) ( (_d[5] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_SL_end_front_phys = (sigfloat_t)(HYTECH_controller_tcs_SL_end_front_ro_fromS(_m->controller_tcs_SL_end_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_SL_start_rear_ro = (uint8_t) ( (_d[6] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_SL_start_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_SL_start_rear_ro_fromS(_m->controller_tcs_SL_start_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_SL_end_rear_ro = (uint8_t) ( (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_SL_end_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_SL_end_rear_ro_fromS(_m->controller_tcs_SL_end_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_CONFIG_CONT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_CONFIG_CONT_hytech(&_m->mon1, CONTROLLER_TCS_CONFIG_CONT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_CONFIG_CONT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_CONFIG_CONT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_start_front_ro = (uint8_t) HYTECH_controller_tcs_lauSL_start_front_ro_toS(_m->controller_tcs_lauSL_start_front_phys);
  _m->controller_tcs_lauSL_end_front_ro = (uint8_t) HYTECH_controller_tcs_lauSL_end_front_ro_toS(_m->controller_tcs_lauSL_end_front_phys);
  _m->controller_tcs_lauSL_start_rear_ro = (uint8_t) HYTECH_controller_tcs_lauSL_start_rear_ro_toS(_m->controller_tcs_lauSL_start_rear_phys);
  _m->controller_tcs_lauSL_end_rear_ro = (uint8_t) HYTECH_controller_tcs_lauSL_end_rear_ro_toS(_m->controller_tcs_lauSL_end_rear_phys);
  _m->controller_tcs_SL_start_front_ro = (uint8_t) HYTECH_controller_tcs_SL_start_front_ro_toS(_m->controller_tcs_SL_start_front_phys);
  _m->controller_tcs_SL_end_front_ro = (uint8_t) HYTECH_controller_tcs_SL_end_front_ro_toS(_m->controller_tcs_SL_end_front_phys);
  _m->controller_tcs_SL_start_rear_ro = (uint8_t) HYTECH_controller_tcs_SL_start_rear_ro_toS(_m->controller_tcs_SL_start_rear_phys);
  _m->controller_tcs_SL_end_rear_ro = (uint8_t) HYTECH_controller_tcs_SL_end_rear_ro_toS(_m->controller_tcs_SL_end_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_lauSL_start_front_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->controller_tcs_lauSL_end_front_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_lauSL_start_rear_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->controller_tcs_lauSL_end_rear_ro & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_tcs_SL_start_front_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->controller_tcs_SL_end_front_ro & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_tcs_SL_start_rear_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( (_m->controller_tcs_SL_end_rear_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_CONFIG_CONT_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_CONFIG_CONT_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_CONFIG_CONT_IDE;
  return CONTROLLER_TCS_CONFIG_CONT_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_CONFIG_CONT_hytech(CONTROLLER_TCS_CONFIG_CONT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_CONFIG_CONT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_lauSL_start_front_ro = (uint8_t) HYTECH_controller_tcs_lauSL_start_front_ro_toS(_m->controller_tcs_lauSL_start_front_phys);
  _m->controller_tcs_lauSL_end_front_ro = (uint8_t) HYTECH_controller_tcs_lauSL_end_front_ro_toS(_m->controller_tcs_lauSL_end_front_phys);
  _m->controller_tcs_lauSL_start_rear_ro = (uint8_t) HYTECH_controller_tcs_lauSL_start_rear_ro_toS(_m->controller_tcs_lauSL_start_rear_phys);
  _m->controller_tcs_lauSL_end_rear_ro = (uint8_t) HYTECH_controller_tcs_lauSL_end_rear_ro_toS(_m->controller_tcs_lauSL_end_rear_phys);
  _m->controller_tcs_SL_start_front_ro = (uint8_t) HYTECH_controller_tcs_SL_start_front_ro_toS(_m->controller_tcs_SL_start_front_phys);
  _m->controller_tcs_SL_end_front_ro = (uint8_t) HYTECH_controller_tcs_SL_end_front_ro_toS(_m->controller_tcs_SL_end_front_phys);
  _m->controller_tcs_SL_start_rear_ro = (uint8_t) HYTECH_controller_tcs_SL_start_rear_ro_toS(_m->controller_tcs_SL_start_rear_phys);
  _m->controller_tcs_SL_end_rear_ro = (uint8_t) HYTECH_controller_tcs_SL_end_rear_ro_toS(_m->controller_tcs_SL_end_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_lauSL_start_front_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->controller_tcs_lauSL_end_front_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_lauSL_start_rear_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->controller_tcs_lauSL_end_rear_ro & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_tcs_SL_start_front_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->controller_tcs_SL_end_front_ro & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_tcs_SL_start_rear_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( (_m->controller_tcs_SL_end_rear_ro & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_CONFIG_CONT_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_CONFIG_CONT_IDE;
  return CONTROLLER_TCS_CONFIG_CONT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_gen_LRdiff_lowerB_ro = (uint16_t) ( ((_d[1] & (0x1FU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_gen_LRdiff_lowerB_phys = (sigfloat_t)(HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_fromS(_m->controller_tcs_gen_LRdiff_lowerB_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_gen_LRdiff_upperB_ro = (uint16_t) ( ((_d[3] & (0x07U)) << 11U) | ((_d[2] & (0xFFU)) << 3U) | ((_d[1] >> 5U) & (0x07U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_gen_LRdiff_upperB_phys = (sigfloat_t)(HYTECH_controller_tcs_gen_LRdiff_upperB_ro_fromS(_m->controller_tcs_gen_LRdiff_upperB_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_w_steer_lower_b_ro = (uint16_t) ( ((_d[4] & (0x7FU)) << 5U) | ((_d[3] >> 3U) & (0x1FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_w_steer_lower_b_phys = (sigfloat_t)(HYTECH_controller_tcs_w_steer_lower_b_ro_fromS(_m->controller_tcs_w_steer_lower_b_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_w_steer_upper_b_ro = (uint16_t) ( ((_d[6] & (0x07U)) << 9U) | ((_d[5] & (0xFFU)) << 1U) | ((_d[4] >> 7U) & (0x01U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_w_steer_upper_b_phys = (sigfloat_t)(HYTECH_controller_tcs_w_steer_upper_b_ro_fromS(_m->controller_tcs_w_steer_upper_b_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_launch_LRdiff_ro = (uint16_t) ( ((_d[7] & (0xFFU)) << 5U) | ((_d[6] >> 3U) & (0x1FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_launch_LRdiff_phys = (sigfloat_t)(HYTECH_controller_tcs_launch_LRdiff_ro_fromS(_m->controller_tcs_launch_LRdiff_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_DIFF_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_DIFF_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_DIFF_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_DIFF_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_DIFF_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_gen_LRdiff_lowerB_ro = (uint16_t) HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_toS(_m->controller_tcs_gen_LRdiff_lowerB_phys);
  _m->controller_tcs_gen_LRdiff_upperB_ro = (uint16_t) HYTECH_controller_tcs_gen_LRdiff_upperB_ro_toS(_m->controller_tcs_gen_LRdiff_upperB_phys);
  _m->controller_tcs_w_steer_lower_b_ro = (uint16_t) HYTECH_controller_tcs_w_steer_lower_b_ro_toS(_m->controller_tcs_w_steer_lower_b_phys);
  _m->controller_tcs_w_steer_upper_b_ro = (uint16_t) HYTECH_controller_tcs_w_steer_upper_b_ro_toS(_m->controller_tcs_w_steer_upper_b_phys);
  _m->controller_tcs_launch_LRdiff_ro = (uint16_t) HYTECH_controller_tcs_launch_LRdiff_ro_toS(_m->controller_tcs_launch_LRdiff_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_gen_LRdiff_lowerB_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_lowerB_ro >> 8U) & (0x1FU)) | ((_m->controller_tcs_gen_LRdiff_upperB_ro & (0x07U)) << 5U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_upperB_ro >> 3U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_upperB_ro >> 11U) & (0x07U)) | ((_m->controller_tcs_w_steer_lower_b_ro & (0x1FU)) << 3U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_tcs_w_steer_lower_b_ro >> 5U) & (0x7FU)) | ((_m->controller_tcs_w_steer_upper_b_ro & (0x01U)) << 7U) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_w_steer_upper_b_ro >> 1U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( ((_m->controller_tcs_w_steer_upper_b_ro >> 9U) & (0x07U)) | ((_m->controller_tcs_launch_LRdiff_ro & (0x1FU)) << 3U) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_launch_LRdiff_ro >> 5U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_DIFF_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_DIFF_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_DIFF_CONFIG_IDE;
  return CONTROLLER_TCS_DIFF_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_DIFF_CONFIG_hytech(CONTROLLER_TCS_DIFF_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_DIFF_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_gen_LRdiff_lowerB_ro = (uint16_t) HYTECH_controller_tcs_gen_LRdiff_lowerB_ro_toS(_m->controller_tcs_gen_LRdiff_lowerB_phys);
  _m->controller_tcs_gen_LRdiff_upperB_ro = (uint16_t) HYTECH_controller_tcs_gen_LRdiff_upperB_ro_toS(_m->controller_tcs_gen_LRdiff_upperB_phys);
  _m->controller_tcs_w_steer_lower_b_ro = (uint16_t) HYTECH_controller_tcs_w_steer_lower_b_ro_toS(_m->controller_tcs_w_steer_lower_b_phys);
  _m->controller_tcs_w_steer_upper_b_ro = (uint16_t) HYTECH_controller_tcs_w_steer_upper_b_ro_toS(_m->controller_tcs_w_steer_upper_b_phys);
  _m->controller_tcs_launch_LRdiff_ro = (uint16_t) HYTECH_controller_tcs_launch_LRdiff_ro_toS(_m->controller_tcs_launch_LRdiff_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_gen_LRdiff_lowerB_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_lowerB_ro >> 8U) & (0x1FU)) | ((_m->controller_tcs_gen_LRdiff_upperB_ro & (0x07U)) << 5U) );
  _d[2] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_upperB_ro >> 3U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_gen_LRdiff_upperB_ro >> 11U) & (0x07U)) | ((_m->controller_tcs_w_steer_lower_b_ro & (0x1FU)) << 3U) );
  _d[4] |= (uint8_t) ( ((_m->controller_tcs_w_steer_lower_b_ro >> 5U) & (0x7FU)) | ((_m->controller_tcs_w_steer_upper_b_ro & (0x01U)) << 7U) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_w_steer_upper_b_ro >> 1U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( ((_m->controller_tcs_w_steer_upper_b_ro >> 9U) & (0x07U)) | ((_m->controller_tcs_launch_LRdiff_ro & (0x1FU)) << 3U) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_launch_LRdiff_ro >> 5U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_DIFF_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_DIFF_CONFIG_IDE;
  return CONTROLLER_TCS_DIFF_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_lower_rpm_front = (uint16_t) ( ((_d[1] & (0x7FU)) << 8U) | (_d[0] & (0xFFU)) );
  _m->controller_tcs_lower_rpm_rear = (uint16_t) ( ((_d[3] & (0x3FU)) << 9U) | ((_d[2] & (0xFFU)) << 1U) | ((_d[1] >> 7U) & (0x01U)) );
  _m->controller_tcs_upper_rpm_front = (uint16_t) ( ((_d[5] & (0x1FU)) << 10U) | ((_d[4] & (0xFFU)) << 2U) | ((_d[3] >> 6U) & (0x03U)) );
  _m->controller_tcs_upper_rpm_rear = (uint16_t) ( ((_d[7] & (0x0FU)) << 11U) | ((_d[6] & (0xFFU)) << 3U) | ((_d[5] >> 5U) & (0x07U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_lower_rpm_front & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_front >> 8U) & (0x7FU)) | ((_m->controller_tcs_lower_rpm_rear & (0x01U)) << 7U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_rear >> 1U) & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_rear >> 9U) & (0x3FU)) | ((_m->controller_tcs_upper_rpm_front & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_front >> 2U) & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_front >> 10U) & (0x1FU)) | ((_m->controller_tcs_upper_rpm_rear & (0x07U)) << 5U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_rear >> 3U) & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_rear >> 11U) & (0x0FU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_RPM_SCHE_CONFIG_IDE;
  return CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_RPM_SCHE_CONFIG_hytech(CONTROLLER_TCS_RPM_SCHE_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->controller_tcs_lower_rpm_front & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_front >> 8U) & (0x7FU)) | ((_m->controller_tcs_lower_rpm_rear & (0x01U)) << 7U) );
  _d[2] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_rear >> 1U) & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_lower_rpm_rear >> 9U) & (0x3FU)) | ((_m->controller_tcs_upper_rpm_front & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_front >> 2U) & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_front >> 10U) & (0x1FU)) | ((_m->controller_tcs_upper_rpm_rear & (0x07U)) << 5U) );
  _d[6] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_rear >> 3U) & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_upper_rpm_rear >> 11U) & (0x0FU)) );

  *_len = (uint8_t) CONTROLLER_TCS_RPM_SCHE_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_RPM_SCHE_CONFIG_IDE;
  return CONTROLLER_TCS_RPM_SCHE_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_slip_target_front_ro = (uint16_t) ( ((_d[1] & (0x0FU)) << 8U) | (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_slip_target_front_phys = (sigfloat_t)(HYTECH_controller_tcs_slip_target_front_ro_fromS(_m->controller_tcs_slip_target_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_slip_target_rear_ro = (uint16_t) ( ((_d[2] & (0xFFU)) << 4U) | ((_d[1] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_slip_target_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_slip_target_rear_ro_fromS(_m->controller_tcs_slip_target_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_SLIP_TARGETS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_SLIP_TARGETS_hytech(&_m->mon1, CONTROLLER_TCS_SLIP_TARGETS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_SLIP_TARGETS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_SLIP_TARGETS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_slip_target_front_ro = (uint16_t) HYTECH_controller_tcs_slip_target_front_ro_toS(_m->controller_tcs_slip_target_front_phys);
  _m->controller_tcs_slip_target_rear_ro = (uint16_t) HYTECH_controller_tcs_slip_target_rear_ro_toS(_m->controller_tcs_slip_target_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_slip_target_front_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_slip_target_front_ro >> 8U) & (0x0FU)) | ((_m->controller_tcs_slip_target_rear_ro & (0x0FU)) << 4U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_tcs_slip_target_rear_ro >> 4U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_SLIP_TARGETS_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_SLIP_TARGETS_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_SLIP_TARGETS_IDE;
  return CONTROLLER_TCS_SLIP_TARGETS_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_SLIP_TARGETS_hytech(CONTROLLER_TCS_SLIP_TARGETS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_SLIP_TARGETS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_slip_target_front_ro = (uint16_t) HYTECH_controller_tcs_slip_target_front_ro_toS(_m->controller_tcs_slip_target_front_phys);
  _m->controller_tcs_slip_target_rear_ro = (uint16_t) HYTECH_controller_tcs_slip_target_rear_ro_toS(_m->controller_tcs_slip_target_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_slip_target_front_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_slip_target_front_ro >> 8U) & (0x0FU)) | ((_m->controller_tcs_slip_target_rear_ro & (0x0FU)) << 4U) );
  _d[2] |= (uint8_t) ( ((_m->controller_tcs_slip_target_rear_ro >> 4U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_SLIP_TARGETS_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_SLIP_TARGETS_IDE;
  return CONTROLLER_TCS_SLIP_TARGETS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_wheel_steer_avg_deg_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_steer_avg_deg_phys = (sigfloat_t)(HYTECH_vehm_wheel_steer_avg_deg_ro_fromS(_m->vehm_wheel_steer_avg_deg_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(&_m->mon1, VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_steer_avg_deg_ro = (int16_t) HYTECH_vehm_wheel_steer_avg_deg_ro_toS(_m->vehm_wheel_steer_avg_deg_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_wheel_steer_avg_deg_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_wheel_steer_avg_deg_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID;
  cframe->DLC = (uint8_t) VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC;
  cframe->IDE = (uint8_t) VEHM_WHEEL_STEER_AVG_DEG_MSG_IDE;
  return VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID;
}

#else

uint32_t Pack_VEHM_WHEEL_STEER_AVG_DEG_MSG_hytech(VEHM_WHEEL_STEER_AVG_DEG_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_wheel_steer_avg_deg_ro = (int16_t) HYTECH_vehm_wheel_steer_avg_deg_ro_toS(_m->vehm_wheel_steer_avg_deg_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_wheel_steer_avg_deg_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_wheel_steer_avg_deg_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_WHEEL_STEER_AVG_DEG_MSG_DLC;
  *_ide = (uint8_t) VEHM_WHEEL_STEER_AVG_DEG_MSG_IDE;
  return VEHM_WHEEL_STEER_AVG_DEG_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_phys = (sigfloat_t)(HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_fromS(_m->vehm_kin_desired_yaw_rate_rad_s_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(&_m->mon1, VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_toS(_m->vehm_kin_desired_yaw_rate_rad_s_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_kin_desired_yaw_rate_rad_s_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_kin_desired_yaw_rate_rad_s_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID;
  cframe->DLC = (uint8_t) VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC;
  cframe->IDE = (uint8_t) VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_IDE;
  return VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID;
}

#else

uint32_t Pack_VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_hytech(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_toS(_m->vehm_kin_desired_yaw_rate_rad_s_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_kin_desired_yaw_rate_rad_s_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_kin_desired_yaw_rate_rad_s_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_DLC;
  *_ide = (uint8_t) VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_IDE;
  return VEHM_KIN_DES_YAW_RATE_RAD_S_MSG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_endper_nl_front_ro = (uint8_t) ( (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_endper_nl_front_phys = (sigfloat_t)(HYTECH_controller_tcs_endper_nl_front_ro_fromS(_m->controller_tcs_endper_nl_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_endper_nl_rear_ro = (uint8_t) ( (_d[1] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_endper_nl_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_endper_nl_rear_ro_fromS(_m->controller_tcs_endper_nl_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_startper_nl_front_ro = (uint8_t) ( (_d[2] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_startper_nl_front_phys = (sigfloat_t)(HYTECH_controller_tcs_startper_nl_front_ro_fromS(_m->controller_tcs_startper_nl_front_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_startper_nl_rear_ro = (uint8_t) ( (_d[3] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_startper_nl_rear_phys = (sigfloat_t)(HYTECH_controller_tcs_startper_nl_rear_ro_fromS(_m->controller_tcs_startper_nl_rear_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_NL_SCHE_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_NL_SCHE_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_NL_SCHE_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_NL_SCHE_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_endper_nl_front_ro = (uint8_t) HYTECH_controller_tcs_endper_nl_front_ro_toS(_m->controller_tcs_endper_nl_front_phys);
  _m->controller_tcs_endper_nl_rear_ro = (uint8_t) HYTECH_controller_tcs_endper_nl_rear_ro_toS(_m->controller_tcs_endper_nl_rear_phys);
  _m->controller_tcs_startper_nl_front_ro = (uint8_t) HYTECH_controller_tcs_startper_nl_front_ro_toS(_m->controller_tcs_startper_nl_front_phys);
  _m->controller_tcs_startper_nl_rear_ro = (uint8_t) HYTECH_controller_tcs_startper_nl_rear_ro_toS(_m->controller_tcs_startper_nl_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_endper_nl_front_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->controller_tcs_endper_nl_rear_ro & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_startper_nl_front_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->controller_tcs_startper_nl_rear_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_NL_SCHE_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_NL_SCHE_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_NL_SCHE_CONFIG_IDE;
  return CONTROLLER_TCS_NL_SCHE_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_NL_SCHE_CONFIG_hytech(CONTROLLER_TCS_NL_SCHE_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_NL_SCHE_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_endper_nl_front_ro = (uint8_t) HYTECH_controller_tcs_endper_nl_front_ro_toS(_m->controller_tcs_endper_nl_front_phys);
  _m->controller_tcs_endper_nl_rear_ro = (uint8_t) HYTECH_controller_tcs_endper_nl_rear_ro_toS(_m->controller_tcs_endper_nl_rear_phys);
  _m->controller_tcs_startper_nl_front_ro = (uint8_t) HYTECH_controller_tcs_startper_nl_front_ro_toS(_m->controller_tcs_startper_nl_front_phys);
  _m->controller_tcs_startper_nl_rear_ro = (uint8_t) HYTECH_controller_tcs_startper_nl_rear_ro_toS(_m->controller_tcs_startper_nl_rear_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_endper_nl_front_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->controller_tcs_endper_nl_rear_ro & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_startper_nl_front_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->controller_tcs_startper_nl_rear_ro & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_NL_SCHE_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_NL_SCHE_CONFIG_IDE;
  return CONTROLLER_TCS_NL_SCHE_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_phys = (sigfloat_t)(HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_fromS(_m->vehm_kin_desired_yaw_rate_rad_s_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_rate_error_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_rate_error_phys = (sigfloat_t)(HYTECH_controller_yaw_rate_error_ro_fromS(_m->controller_yaw_rate_error_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_pid_output_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_pid_output_phys = (sigfloat_t)(HYTECH_controller_yaw_pid_output_ro_fromS(_m->controller_yaw_pid_output_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_PID_YAW_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_PID_YAW_hytech(&_m->mon1, CONTROLLER_PID_YAW_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_PID_YAW_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_YAW_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_toS(_m->vehm_kin_desired_yaw_rate_rad_s_phys);
  _m->controller_yaw_rate_error_ro = (int16_t) HYTECH_controller_yaw_rate_error_ro_toS(_m->controller_yaw_rate_error_phys);
  _m->controller_yaw_pid_output_ro = (int16_t) HYTECH_controller_yaw_pid_output_ro_toS(_m->controller_yaw_pid_output_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_kin_desired_yaw_rate_rad_s_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_kin_desired_yaw_rate_rad_s_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_yaw_rate_error_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_yaw_rate_error_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_yaw_pid_output_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_yaw_pid_output_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_PID_YAW_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_PID_YAW_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_PID_YAW_IDE;
  return CONTROLLER_PID_YAW_CANID;
}

#else

uint32_t Pack_CONTROLLER_PID_YAW_hytech(CONTROLLER_PID_YAW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_YAW_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_kin_desired_yaw_rate_rad_s_ro = (int16_t) HYTECH_vehm_kin_desired_yaw_rate_rad_s_ro_toS(_m->vehm_kin_desired_yaw_rate_rad_s_phys);
  _m->controller_yaw_rate_error_ro = (int16_t) HYTECH_controller_yaw_rate_error_ro_toS(_m->controller_yaw_rate_error_phys);
  _m->controller_yaw_pid_output_ro = (int16_t) HYTECH_controller_yaw_pid_output_ro_toS(_m->controller_yaw_pid_output_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_kin_desired_yaw_rate_rad_s_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_kin_desired_yaw_rate_rad_s_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_yaw_rate_error_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_yaw_rate_error_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_yaw_pid_output_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_yaw_pid_output_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_PID_YAW_DLC;
  *_ide = (uint8_t) CONTROLLER_PID_YAW_IDE;
  return CONTROLLER_PID_YAW_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_long_corner_vel_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_fl_phys = (sigfloat_t)(HYTECH_vehm_long_corner_vel_fl_ro_fromS(_m->vehm_long_corner_vel_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_long_corner_vel_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_fr_phys = (sigfloat_t)(HYTECH_vehm_long_corner_vel_fr_ro_fromS(_m->vehm_long_corner_vel_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_long_corner_vel_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_rl_phys = (sigfloat_t)(HYTECH_vehm_long_corner_vel_rl_ro_fromS(_m->vehm_long_corner_vel_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_long_corner_vel_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_rr_phys = (sigfloat_t)(HYTECH_vehm_long_corner_vel_rr_ro_fromS(_m->vehm_long_corner_vel_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_LONG_CORNER_VEL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_LONG_CORNER_VEL_hytech(&_m->mon1, VEHM_LONG_CORNER_VEL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_LONG_CORNER_VEL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_LONG_CORNER_VEL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_fl_ro = (int16_t) HYTECH_vehm_long_corner_vel_fl_ro_toS(_m->vehm_long_corner_vel_fl_phys);
  _m->vehm_long_corner_vel_fr_ro = (int16_t) HYTECH_vehm_long_corner_vel_fr_ro_toS(_m->vehm_long_corner_vel_fr_phys);
  _m->vehm_long_corner_vel_rl_ro = (int16_t) HYTECH_vehm_long_corner_vel_rl_ro_toS(_m->vehm_long_corner_vel_rl_phys);
  _m->vehm_long_corner_vel_rr_ro = (int16_t) HYTECH_vehm_long_corner_vel_rr_ro_toS(_m->vehm_long_corner_vel_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_long_corner_vel_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_long_corner_vel_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vehm_long_corner_vel_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vehm_long_corner_vel_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vehm_long_corner_vel_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vehm_long_corner_vel_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->vehm_long_corner_vel_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vehm_long_corner_vel_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_LONG_CORNER_VEL_CANID;
  cframe->DLC = (uint8_t) VEHM_LONG_CORNER_VEL_DLC;
  cframe->IDE = (uint8_t) VEHM_LONG_CORNER_VEL_IDE;
  return VEHM_LONG_CORNER_VEL_CANID;
}

#else

uint32_t Pack_VEHM_LONG_CORNER_VEL_hytech(VEHM_LONG_CORNER_VEL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_LONG_CORNER_VEL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_long_corner_vel_fl_ro = (int16_t) HYTECH_vehm_long_corner_vel_fl_ro_toS(_m->vehm_long_corner_vel_fl_phys);
  _m->vehm_long_corner_vel_fr_ro = (int16_t) HYTECH_vehm_long_corner_vel_fr_ro_toS(_m->vehm_long_corner_vel_fr_phys);
  _m->vehm_long_corner_vel_rl_ro = (int16_t) HYTECH_vehm_long_corner_vel_rl_ro_toS(_m->vehm_long_corner_vel_rl_phys);
  _m->vehm_long_corner_vel_rr_ro = (int16_t) HYTECH_vehm_long_corner_vel_rr_ro_toS(_m->vehm_long_corner_vel_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_long_corner_vel_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_long_corner_vel_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vehm_long_corner_vel_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vehm_long_corner_vel_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vehm_long_corner_vel_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vehm_long_corner_vel_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->vehm_long_corner_vel_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vehm_long_corner_vel_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_LONG_CORNER_VEL_DLC;
  *_ide = (uint8_t) VEHM_LONG_CORNER_VEL_IDE;
  return VEHM_LONG_CORNER_VEL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_SL_hytech(VEHM_SL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_sl_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_fl_phys = (sigfloat_t)(HYTECH_vehm_sl_fl_ro_fromS(_m->vehm_sl_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_sl_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_fr_phys = (sigfloat_t)(HYTECH_vehm_sl_fr_ro_fromS(_m->vehm_sl_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_sl_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_rl_phys = (sigfloat_t)(HYTECH_vehm_sl_rl_ro_fromS(_m->vehm_sl_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_sl_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_rr_phys = (sigfloat_t)(HYTECH_vehm_sl_rr_ro_fromS(_m->vehm_sl_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_SL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_SL_hytech(&_m->mon1, VEHM_SL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_SL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_SL_hytech(VEHM_SL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_SL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_fl_ro = (int16_t) HYTECH_vehm_sl_fl_ro_toS(_m->vehm_sl_fl_phys);
  _m->vehm_sl_fr_ro = (int16_t) HYTECH_vehm_sl_fr_ro_toS(_m->vehm_sl_fr_phys);
  _m->vehm_sl_rl_ro = (int16_t) HYTECH_vehm_sl_rl_ro_toS(_m->vehm_sl_rl_phys);
  _m->vehm_sl_rr_ro = (int16_t) HYTECH_vehm_sl_rr_ro_toS(_m->vehm_sl_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_sl_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_sl_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vehm_sl_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vehm_sl_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vehm_sl_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vehm_sl_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->vehm_sl_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vehm_sl_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_SL_CANID;
  cframe->DLC = (uint8_t) VEHM_SL_DLC;
  cframe->IDE = (uint8_t) VEHM_SL_IDE;
  return VEHM_SL_CANID;
}

#else

uint32_t Pack_VEHM_SL_hytech(VEHM_SL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_SL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_sl_fl_ro = (int16_t) HYTECH_vehm_sl_fl_ro_toS(_m->vehm_sl_fl_phys);
  _m->vehm_sl_fr_ro = (int16_t) HYTECH_vehm_sl_fr_ro_toS(_m->vehm_sl_fr_phys);
  _m->vehm_sl_rl_ro = (int16_t) HYTECH_vehm_sl_rl_ro_toS(_m->vehm_sl_rl_phys);
  _m->vehm_sl_rr_ro = (int16_t) HYTECH_vehm_sl_rr_ro_toS(_m->vehm_sl_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_sl_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_sl_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vehm_sl_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vehm_sl_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vehm_sl_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vehm_sl_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->vehm_sl_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vehm_sl_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_SL_DLC;
  *_ide = (uint8_t) VEHM_SL_IDE;
  return VEHM_SL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_yaw_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_fl_phys = (sigfloat_t)(HYTECH_controller_yaw_torque_fl_ro_fromS(_m->controller_yaw_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_fr_phys = (sigfloat_t)(HYTECH_controller_yaw_torque_fr_ro_fromS(_m->controller_yaw_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_rl_phys = (sigfloat_t)(HYTECH_controller_yaw_torque_rl_ro_fromS(_m->controller_yaw_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_yaw_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_rr_phys = (sigfloat_t)(HYTECH_controller_yaw_torque_rr_ro_fromS(_m->controller_yaw_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_PID_YAW_TORQUE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_PID_YAW_TORQUE_hytech(&_m->mon1, CONTROLLER_PID_YAW_TORQUE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_PID_YAW_TORQUE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_YAW_TORQUE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_fl_ro = (int16_t) HYTECH_controller_yaw_torque_fl_ro_toS(_m->controller_yaw_torque_fl_phys);
  _m->controller_yaw_torque_fr_ro = (int16_t) HYTECH_controller_yaw_torque_fr_ro_toS(_m->controller_yaw_torque_fr_phys);
  _m->controller_yaw_torque_rl_ro = (int16_t) HYTECH_controller_yaw_torque_rl_ro_toS(_m->controller_yaw_torque_rl_phys);
  _m->controller_yaw_torque_rr_ro = (int16_t) HYTECH_controller_yaw_torque_rr_ro_toS(_m->controller_yaw_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_yaw_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_yaw_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_yaw_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_yaw_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_yaw_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_yaw_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_yaw_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_yaw_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_PID_YAW_TORQUE_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_PID_YAW_TORQUE_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_PID_YAW_TORQUE_IDE;
  return CONTROLLER_PID_YAW_TORQUE_CANID;
}

#else

uint32_t Pack_CONTROLLER_PID_YAW_TORQUE_hytech(CONTROLLER_PID_YAW_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_PID_YAW_TORQUE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_yaw_torque_fl_ro = (int16_t) HYTECH_controller_yaw_torque_fl_ro_toS(_m->controller_yaw_torque_fl_phys);
  _m->controller_yaw_torque_fr_ro = (int16_t) HYTECH_controller_yaw_torque_fr_ro_toS(_m->controller_yaw_torque_fr_phys);
  _m->controller_yaw_torque_rl_ro = (int16_t) HYTECH_controller_yaw_torque_rl_ro_toS(_m->controller_yaw_torque_rl_phys);
  _m->controller_yaw_torque_rr_ro = (int16_t) HYTECH_controller_yaw_torque_rr_ro_toS(_m->controller_yaw_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_yaw_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_yaw_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_yaw_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_yaw_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_yaw_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_yaw_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_yaw_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_yaw_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_PID_YAW_TORQUE_DLC;
  *_ide = (uint8_t) CONTROLLER_PID_YAW_TORQUE_IDE;
  return CONTROLLER_PID_YAW_TORQUE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_normal_percent_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_fl_phys = (sigfloat_t)(HYTECH_controller_normal_percent_fl_ro_fromS(_m->controller_normal_percent_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_percent_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_fr_phys = (sigfloat_t)(HYTECH_controller_normal_percent_fr_ro_fromS(_m->controller_normal_percent_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_percent_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_rl_phys = (sigfloat_t)(HYTECH_controller_normal_percent_rl_ro_fromS(_m->controller_normal_percent_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_percent_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_rr_phys = (sigfloat_t)(HYTECH_controller_normal_percent_rr_ro_fromS(_m->controller_normal_percent_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_NORMAL_DIST_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_NORMAL_DIST_hytech(&_m->mon1, CONTROLLER_NORMAL_DIST_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_NORMAL_DIST_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_NORMAL_DIST_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_fl_ro = (int16_t) HYTECH_controller_normal_percent_fl_ro_toS(_m->controller_normal_percent_fl_phys);
  _m->controller_normal_percent_fr_ro = (int16_t) HYTECH_controller_normal_percent_fr_ro_toS(_m->controller_normal_percent_fr_phys);
  _m->controller_normal_percent_rl_ro = (int16_t) HYTECH_controller_normal_percent_rl_ro_toS(_m->controller_normal_percent_rl_phys);
  _m->controller_normal_percent_rr_ro = (int16_t) HYTECH_controller_normal_percent_rr_ro_toS(_m->controller_normal_percent_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_normal_percent_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_normal_percent_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_normal_percent_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_normal_percent_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_normal_percent_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_normal_percent_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_normal_percent_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_normal_percent_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_NORMAL_DIST_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_NORMAL_DIST_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_NORMAL_DIST_IDE;
  return CONTROLLER_NORMAL_DIST_CANID;
}

#else

uint32_t Pack_CONTROLLER_NORMAL_DIST_hytech(CONTROLLER_NORMAL_DIST_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_NORMAL_DIST_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_percent_fl_ro = (int16_t) HYTECH_controller_normal_percent_fl_ro_toS(_m->controller_normal_percent_fl_phys);
  _m->controller_normal_percent_fr_ro = (int16_t) HYTECH_controller_normal_percent_fr_ro_toS(_m->controller_normal_percent_fr_phys);
  _m->controller_normal_percent_rl_ro = (int16_t) HYTECH_controller_normal_percent_rl_ro_toS(_m->controller_normal_percent_rl_phys);
  _m->controller_normal_percent_rr_ro = (int16_t) HYTECH_controller_normal_percent_rr_ro_toS(_m->controller_normal_percent_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_normal_percent_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_normal_percent_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_normal_percent_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_normal_percent_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_normal_percent_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_normal_percent_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_normal_percent_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_normal_percent_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_NORMAL_DIST_DLC;
  *_ide = (uint8_t) CONTROLLER_NORMAL_DIST_IDE;
  return CONTROLLER_NORMAL_DIST_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_normal_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_fl_phys = (sigfloat_t)(HYTECH_controller_normal_torque_fl_ro_fromS(_m->controller_normal_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_fr_phys = (sigfloat_t)(HYTECH_controller_normal_torque_fr_ro_fromS(_m->controller_normal_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_rl_phys = (sigfloat_t)(HYTECH_controller_normal_torque_rl_ro_fromS(_m->controller_normal_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_normal_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_rr_phys = (sigfloat_t)(HYTECH_controller_normal_torque_rr_ro_fromS(_m->controller_normal_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_NORMAL_TORQUE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_NORMAL_TORQUE_hytech(&_m->mon1, CONTROLLER_NORMAL_TORQUE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_NORMAL_TORQUE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_NORMAL_TORQUE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_fl_ro = (int16_t) HYTECH_controller_normal_torque_fl_ro_toS(_m->controller_normal_torque_fl_phys);
  _m->controller_normal_torque_fr_ro = (int16_t) HYTECH_controller_normal_torque_fr_ro_toS(_m->controller_normal_torque_fr_phys);
  _m->controller_normal_torque_rl_ro = (int16_t) HYTECH_controller_normal_torque_rl_ro_toS(_m->controller_normal_torque_rl_phys);
  _m->controller_normal_torque_rr_ro = (int16_t) HYTECH_controller_normal_torque_rr_ro_toS(_m->controller_normal_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_normal_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_normal_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_normal_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_normal_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_normal_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_normal_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_normal_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_normal_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_NORMAL_TORQUE_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_NORMAL_TORQUE_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_NORMAL_TORQUE_IDE;
  return CONTROLLER_NORMAL_TORQUE_CANID;
}

#else

uint32_t Pack_CONTROLLER_NORMAL_TORQUE_hytech(CONTROLLER_NORMAL_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_NORMAL_TORQUE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_normal_torque_fl_ro = (int16_t) HYTECH_controller_normal_torque_fl_ro_toS(_m->controller_normal_torque_fl_phys);
  _m->controller_normal_torque_fr_ro = (int16_t) HYTECH_controller_normal_torque_fr_ro_toS(_m->controller_normal_torque_fr_phys);
  _m->controller_normal_torque_rl_ro = (int16_t) HYTECH_controller_normal_torque_rl_ro_toS(_m->controller_normal_torque_rl_phys);
  _m->controller_normal_torque_rr_ro = (int16_t) HYTECH_controller_normal_torque_rr_ro_toS(_m->controller_normal_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_normal_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_normal_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_normal_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_normal_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_normal_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_normal_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_normal_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_normal_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_NORMAL_TORQUE_DLC;
  *_ide = (uint8_t) CONTROLLER_NORMAL_TORQUE_IDE;
  return CONTROLLER_NORMAL_TORQUE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_power_lim_status = (uint8_t) ( (_d[0] & (0x0FU)) );
  _m->controller_power_lim_error_ro = (int16_t) __ext_sig__(( ((_d[2] & (0x0FU)) << 12U) | ((_d[1] & (0xFFU)) << 4U) | ((_d[0] >> 4U) & (0x0FU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_error_phys = (sigfloat_t)(HYTECH_controller_power_lim_error_ro_fromS(_m->controller_power_lim_error_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_torque_adj_ro = (int16_t) __ext_sig__(( ((_d[4] & (0x0FU)) << 12U) | ((_d[3] & (0xFFU)) << 4U) | ((_d[2] >> 4U) & (0x0FU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_adj_phys = (sigfloat_t)(HYTECH_controller_power_lim_torque_adj_ro_fromS(_m->controller_power_lim_torque_adj_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_POWER_LIM_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_POWER_LIM_hytech(&_m->mon1, CONTROLLER_POWER_LIM_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_POWER_LIM_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_error_ro = (int16_t) HYTECH_controller_power_lim_error_ro_toS(_m->controller_power_lim_error_phys);
  _m->controller_power_lim_torque_adj_ro = (int16_t) HYTECH_controller_power_lim_torque_adj_ro_toS(_m->controller_power_lim_torque_adj_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_power_lim_status & (0x0FU)) | ((_m->controller_power_lim_error_ro & (0x0FU)) << 4U) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_power_lim_error_ro >> 4U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_power_lim_error_ro >> 12U) & (0x0FU)) | ((_m->controller_power_lim_torque_adj_ro & (0x0FU)) << 4U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_power_lim_torque_adj_ro >> 4U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_power_lim_torque_adj_ro >> 12U) & (0x0FU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_POWER_LIM_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_POWER_LIM_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_POWER_LIM_IDE;
  return CONTROLLER_POWER_LIM_CANID;
}

#else

uint32_t Pack_CONTROLLER_POWER_LIM_hytech(CONTROLLER_POWER_LIM_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_error_ro = (int16_t) HYTECH_controller_power_lim_error_ro_toS(_m->controller_power_lim_error_phys);
  _m->controller_power_lim_torque_adj_ro = (int16_t) HYTECH_controller_power_lim_torque_adj_ro_toS(_m->controller_power_lim_torque_adj_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_power_lim_status & (0x0FU)) | ((_m->controller_power_lim_error_ro & (0x0FU)) << 4U) );
  _d[1] |= (uint8_t) ( ((_m->controller_power_lim_error_ro >> 4U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( ((_m->controller_power_lim_error_ro >> 12U) & (0x0FU)) | ((_m->controller_power_lim_torque_adj_ro & (0x0FU)) << 4U) );
  _d[3] |= (uint8_t) ( ((_m->controller_power_lim_torque_adj_ro >> 4U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( ((_m->controller_power_lim_torque_adj_ro >> 12U) & (0x0FU)) );

  *_len = (uint8_t) CONTROLLER_POWER_LIM_DLC;
  *_ide = (uint8_t) CONTROLLER_POWER_LIM_IDE;
  return CONTROLLER_POWER_LIM_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_power_lim_cornerp_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_fl_phys = (sigfloat_t)(HYTECH_controller_power_lim_cornerp_fl_ro_fromS(_m->controller_power_lim_cornerp_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_cornerp_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_fr_phys = (sigfloat_t)(HYTECH_controller_power_lim_cornerp_fr_ro_fromS(_m->controller_power_lim_cornerp_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_cornerp_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_rl_phys = (sigfloat_t)(HYTECH_controller_power_lim_cornerp_rl_ro_fromS(_m->controller_power_lim_cornerp_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_cornerp_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_rr_phys = (sigfloat_t)(HYTECH_controller_power_lim_cornerp_rr_ro_fromS(_m->controller_power_lim_cornerp_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_POWER_LIM_CORNER_POW_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_POWER_LIM_CORNER_POW_hytech(&_m->mon1, CONTROLLER_POWER_LIM_CORNER_POW_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_POWER_LIM_CORNER_POW_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_CORNER_POW_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_fl_ro = (int16_t) HYTECH_controller_power_lim_cornerp_fl_ro_toS(_m->controller_power_lim_cornerp_fl_phys);
  _m->controller_power_lim_cornerp_fr_ro = (int16_t) HYTECH_controller_power_lim_cornerp_fr_ro_toS(_m->controller_power_lim_cornerp_fr_phys);
  _m->controller_power_lim_cornerp_rl_ro = (int16_t) HYTECH_controller_power_lim_cornerp_rl_ro_toS(_m->controller_power_lim_cornerp_rl_phys);
  _m->controller_power_lim_cornerp_rr_ro = (int16_t) HYTECH_controller_power_lim_cornerp_rr_ro_toS(_m->controller_power_lim_cornerp_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_power_lim_cornerp_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_power_lim_cornerp_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_power_lim_cornerp_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_power_lim_cornerp_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_POWER_LIM_CORNER_POW_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_POWER_LIM_CORNER_POW_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_POWER_LIM_CORNER_POW_IDE;
  return CONTROLLER_POWER_LIM_CORNER_POW_CANID;
}

#else

uint32_t Pack_CONTROLLER_POWER_LIM_CORNER_POW_hytech(CONTROLLER_POWER_LIM_CORNER_POW_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_CORNER_POW_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_cornerp_fl_ro = (int16_t) HYTECH_controller_power_lim_cornerp_fl_ro_toS(_m->controller_power_lim_cornerp_fl_phys);
  _m->controller_power_lim_cornerp_fr_ro = (int16_t) HYTECH_controller_power_lim_cornerp_fr_ro_toS(_m->controller_power_lim_cornerp_fr_phys);
  _m->controller_power_lim_cornerp_rl_ro = (int16_t) HYTECH_controller_power_lim_cornerp_rl_ro_toS(_m->controller_power_lim_cornerp_rl_phys);
  _m->controller_power_lim_cornerp_rr_ro = (int16_t) HYTECH_controller_power_lim_cornerp_rr_ro_toS(_m->controller_power_lim_cornerp_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_power_lim_cornerp_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_power_lim_cornerp_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_power_lim_cornerp_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_power_lim_cornerp_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_power_lim_cornerp_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_POWER_LIM_CORNER_POW_DLC;
  *_ide = (uint8_t) CONTROLLER_POWER_LIM_CORNER_POW_IDE;
  return CONTROLLER_POWER_LIM_CORNER_POW_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_power_lim_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_fl_phys = (sigfloat_t)(HYTECH_controller_power_lim_torque_fl_ro_fromS(_m->controller_power_lim_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_fr_phys = (sigfloat_t)(HYTECH_controller_power_lim_torque_fr_ro_fromS(_m->controller_power_lim_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_rl_phys = (sigfloat_t)(HYTECH_controller_power_lim_torque_rl_ro_fromS(_m->controller_power_lim_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_power_lim_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_rr_phys = (sigfloat_t)(HYTECH_controller_power_lim_torque_rr_ro_fromS(_m->controller_power_lim_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_POWER_LIM_TORQUE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_POWER_LIM_TORQUE_hytech(&_m->mon1, CONTROLLER_POWER_LIM_TORQUE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_POWER_LIM_TORQUE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_TORQUE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_fl_ro = (int16_t) HYTECH_controller_power_lim_torque_fl_ro_toS(_m->controller_power_lim_torque_fl_phys);
  _m->controller_power_lim_torque_fr_ro = (int16_t) HYTECH_controller_power_lim_torque_fr_ro_toS(_m->controller_power_lim_torque_fr_phys);
  _m->controller_power_lim_torque_rl_ro = (int16_t) HYTECH_controller_power_lim_torque_rl_ro_toS(_m->controller_power_lim_torque_rl_phys);
  _m->controller_power_lim_torque_rr_ro = (int16_t) HYTECH_controller_power_lim_torque_rr_ro_toS(_m->controller_power_lim_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_power_lim_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_power_lim_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_power_lim_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_power_lim_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_power_lim_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_power_lim_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_power_lim_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_power_lim_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_POWER_LIM_TORQUE_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_POWER_LIM_TORQUE_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_POWER_LIM_TORQUE_IDE;
  return CONTROLLER_POWER_LIM_TORQUE_CANID;
}

#else

uint32_t Pack_CONTROLLER_POWER_LIM_TORQUE_hytech(CONTROLLER_POWER_LIM_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_POWER_LIM_TORQUE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_power_lim_torque_fl_ro = (int16_t) HYTECH_controller_power_lim_torque_fl_ro_toS(_m->controller_power_lim_torque_fl_phys);
  _m->controller_power_lim_torque_fr_ro = (int16_t) HYTECH_controller_power_lim_torque_fr_ro_toS(_m->controller_power_lim_torque_fr_phys);
  _m->controller_power_lim_torque_rl_ro = (int16_t) HYTECH_controller_power_lim_torque_rl_ro_toS(_m->controller_power_lim_torque_rl_phys);
  _m->controller_power_lim_torque_rr_ro = (int16_t) HYTECH_controller_power_lim_torque_rr_ro_toS(_m->controller_power_lim_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_power_lim_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_power_lim_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_power_lim_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_power_lim_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_power_lim_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_power_lim_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_power_lim_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_power_lim_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_POWER_LIM_TORQUE_DLC;
  *_ide = (uint8_t) CONTROLLER_POWER_LIM_TORQUE_IDE;
  return CONTROLLER_POWER_LIM_TORQUE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_use_launch = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->controller_use_pid_tv = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->controller_use_normal_force = (uint8_t) ( ((_d[0] >> 2U) & (0x01U)) );
  _m->controller_use_pid_power_limit = (uint8_t) ( ((_d[0] >> 3U) & (0x01U)) );
  _m->controller_use_power_limit = (uint8_t) ( ((_d[0] >> 4U) & (0x01U)) );
  _m->controller_use_tcs = (uint8_t) ( ((_d[0] >> 5U) & (0x01U)) );
  _m->controller_use_tcs_lim_yaw_pid = (uint8_t) ( ((_d[0] >> 6U) & (0x01U)) );
  _m->controller_use_dec_yaw_pid_brake = (uint8_t) ( ((_d[0] >> 7U) & (0x01U)) );
  _m->controller_use_discontin_brakes = (uint8_t) ( (_d[1] & (0x01U)) );
  _m->controller_use_no_regen_5kph = (uint8_t) ( ((_d[1] >> 1U) & (0x01U)) );
  _m->controller_use_torque_bias = (uint8_t) ( ((_d[1] >> 2U) & (0x01U)) );
  _m->controller_use_nl_tcs_gain_sche = (uint8_t) ( ((_d[1] >> 3U) & (0x01U)) );
  _m->controller_use_rpm_tcs_gain_sche = (uint8_t) ( ((_d[1] >> 4U) & (0x01U)) );
  _m->controller_use_nl_tcs_slipschedu = (uint8_t) ( ((_d[1] >> 5U) & (0x01U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_BOOLEAN_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_BOOLEAN_hytech(&_m->mon1, CONTROLLER_BOOLEAN_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_BOOLEAN_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_BOOLEAN_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->controller_use_launch & (0x01U)) | ((_m->controller_use_pid_tv & (0x01U)) << 1U) | ((_m->controller_use_normal_force & (0x01U)) << 2U) | ((_m->controller_use_pid_power_limit & (0x01U)) << 3U) | ((_m->controller_use_power_limit & (0x01U)) << 4U) | ((_m->controller_use_tcs & (0x01U)) << 5U) | ((_m->controller_use_tcs_lim_yaw_pid & (0x01U)) << 6U) | ((_m->controller_use_dec_yaw_pid_brake & (0x01U)) << 7U) );
  cframe->Data[1] |= (uint8_t) ( (_m->controller_use_discontin_brakes & (0x01U)) | ((_m->controller_use_no_regen_5kph & (0x01U)) << 1U) | ((_m->controller_use_torque_bias & (0x01U)) << 2U) | ((_m->controller_use_nl_tcs_gain_sche & (0x01U)) << 3U) | ((_m->controller_use_rpm_tcs_gain_sche & (0x01U)) << 4U) | ((_m->controller_use_nl_tcs_slipschedu & (0x01U)) << 5U) );

  cframe->MsgId = (uint32_t) CONTROLLER_BOOLEAN_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_BOOLEAN_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_BOOLEAN_IDE;
  return CONTROLLER_BOOLEAN_CANID;
}

#else

uint32_t Pack_CONTROLLER_BOOLEAN_hytech(CONTROLLER_BOOLEAN_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_BOOLEAN_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->controller_use_launch & (0x01U)) | ((_m->controller_use_pid_tv & (0x01U)) << 1U) | ((_m->controller_use_normal_force & (0x01U)) << 2U) | ((_m->controller_use_pid_power_limit & (0x01U)) << 3U) | ((_m->controller_use_power_limit & (0x01U)) << 4U) | ((_m->controller_use_tcs & (0x01U)) << 5U) | ((_m->controller_use_tcs_lim_yaw_pid & (0x01U)) << 6U) | ((_m->controller_use_dec_yaw_pid_brake & (0x01U)) << 7U) );
  _d[1] |= (uint8_t) ( (_m->controller_use_discontin_brakes & (0x01U)) | ((_m->controller_use_no_regen_5kph & (0x01U)) << 1U) | ((_m->controller_use_torque_bias & (0x01U)) << 2U) | ((_m->controller_use_nl_tcs_gain_sche & (0x01U)) << 3U) | ((_m->controller_use_rpm_tcs_gain_sche & (0x01U)) << 4U) | ((_m->controller_use_nl_tcs_slipschedu & (0x01U)) << 5U) );

  *_len = (uint8_t) CONTROLLER_BOOLEAN_DLC;
  *_ide = (uint8_t) CONTROLLER_BOOLEAN_IDE;
  return CONTROLLER_BOOLEAN_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_BETA_hytech(VEHM_BETA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_beta_deg_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_beta_deg_phys = (sigfloat_t)(HYTECH_vehm_beta_deg_ro_fromS(_m->vehm_beta_deg_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_BETA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_BETA_hytech(&_m->mon1, VEHM_BETA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_BETA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_BETA_hytech(VEHM_BETA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_BETA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_beta_deg_ro = (int16_t) HYTECH_vehm_beta_deg_ro_toS(_m->vehm_beta_deg_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_beta_deg_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_beta_deg_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_BETA_CANID;
  cframe->DLC = (uint8_t) VEHM_BETA_DLC;
  cframe->IDE = (uint8_t) VEHM_BETA_IDE;
  return VEHM_BETA_CANID;
}

#else

uint32_t Pack_VEHM_BETA_hytech(VEHM_BETA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_BETA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_beta_deg_ro = (int16_t) HYTECH_vehm_beta_deg_ro_toS(_m->vehm_beta_deg_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_beta_deg_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_beta_deg_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_BETA_DLC;
  *_ide = (uint8_t) VEHM_BETA_IDE;
  return VEHM_BETA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->dash_buzzer_flag = (uint8_t) ( (_d[0] & (0x01U)) );
  _m->in_pedal_calibration_state = (uint8_t) ( ((_d[0] >> 1U) & (0x01U)) );
  _m->torque_limit_enum_value = (uint8_t) ( (_d[1] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < DASHBOARD_BUZZER_CONTROL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_DASHBOARD_BUZZER_CONTROL_hytech(&_m->mon1, DASHBOARD_BUZZER_CONTROL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return DASHBOARD_BUZZER_CONTROL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_BUZZER_CONTROL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->dash_buzzer_flag & (0x01U)) | ((_m->in_pedal_calibration_state & (0x01U)) << 1U) );
  cframe->Data[1] |= (uint8_t) ( (_m->torque_limit_enum_value & (0xFFU)) );

  cframe->MsgId = (uint32_t) DASHBOARD_BUZZER_CONTROL_CANID;
  cframe->DLC = (uint8_t) DASHBOARD_BUZZER_CONTROL_DLC;
  cframe->IDE = (uint8_t) DASHBOARD_BUZZER_CONTROL_IDE;
  return DASHBOARD_BUZZER_CONTROL_CANID;
}

#else

uint32_t Pack_DASHBOARD_BUZZER_CONTROL_hytech(DASHBOARD_BUZZER_CONTROL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(DASHBOARD_BUZZER_CONTROL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->dash_buzzer_flag & (0x01U)) | ((_m->in_pedal_calibration_state & (0x01U)) << 1U) );
  _d[1] |= (uint8_t) ( (_m->torque_limit_enum_value & (0xFFU)) );

  *_len = (uint8_t) DASHBOARD_BUZZER_CONTROL_DLC;
  *_ide = (uint8_t) DASHBOARD_BUZZER_CONTROL_IDE;
  return DASHBOARD_BUZZER_CONTROL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_initial_torque_req_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_initial_torque_req_phys = (sigfloat_t)(HYTECH_controller_initial_torque_req_ro_fromS(_m->controller_initial_torque_req_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_constrained_torq_req_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_constrained_torq_req_phys = (sigfloat_t)(HYTECH_controller_constrained_torq_req_ro_fromS(_m->controller_constrained_torq_req_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_regen_limit_ro = (int16_t) __ext_sig__(( ((_d[5] & (0x0FU)) << 8U) | (_d[4] & (0xFFU)) ), 12);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_regen_limit_phys = (sigfloat_t)(HYTECH_controller_regen_limit_ro_fromS(_m->controller_regen_limit_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_torque_mode_ro = (uint8_t) ( ((_d[6] & (0x0FU)) << 4U) | ((_d[5] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_torque_mode_phys = (sigfloat_t)(HYTECH_controller_torque_mode_ro_fromS(_m->controller_torque_mode_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_max_yaw_nl_accel_perc_ro = (uint8_t) ( ((_d[6] >> 4U) & (0x0FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_max_yaw_nl_accel_perc_phys = (sigfloat_t)(HYTECH_controller_max_yaw_nl_accel_perc_ro_fromS(_m->controller_max_yaw_nl_accel_perc_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_max_nl_brake_sc_front_ro = (uint8_t) ( (_d[7] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_max_nl_brake_sc_front_phys = (sigfloat_t)(HYTECH_controller_max_nl_brake_sc_front_ro_fromS(_m->controller_max_nl_brake_sc_front_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TORQUE_SETUP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TORQUE_SETUP_hytech(&_m->mon1, CONTROLLER_TORQUE_SETUP_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TORQUE_SETUP_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TORQUE_SETUP_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_initial_torque_req_ro = (int16_t) HYTECH_controller_initial_torque_req_ro_toS(_m->controller_initial_torque_req_phys);
  _m->controller_constrained_torq_req_ro = (int16_t) HYTECH_controller_constrained_torq_req_ro_toS(_m->controller_constrained_torq_req_phys);
  _m->controller_regen_limit_ro = (int16_t) HYTECH_controller_regen_limit_ro_toS(_m->controller_regen_limit_phys);
  _m->controller_torque_mode_ro = (uint8_t) HYTECH_controller_torque_mode_ro_toS(_m->controller_torque_mode_phys);
  _m->controller_max_yaw_nl_accel_perc_ro = (uint8_t) HYTECH_controller_max_yaw_nl_accel_perc_ro_toS(_m->controller_max_yaw_nl_accel_perc_phys);
  _m->controller_max_nl_brake_sc_front_ro = (uint8_t) HYTECH_controller_max_nl_brake_sc_front_ro_toS(_m->controller_max_nl_brake_sc_front_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_initial_torque_req_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_initial_torque_req_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_constrained_torq_req_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_constrained_torq_req_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_regen_limit_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_regen_limit_ro >> 8U) & (0x0FU)) | ((_m->controller_torque_mode_ro & (0x0FU)) << 4U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->controller_torque_mode_ro >> 4U) & (0x0FU)) | ((_m->controller_max_yaw_nl_accel_perc_ro & (0x0FU)) << 4U) );
  cframe->Data[7] |= (uint8_t) ( (_m->controller_max_nl_brake_sc_front_ro & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TORQUE_SETUP_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TORQUE_SETUP_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TORQUE_SETUP_IDE;
  return CONTROLLER_TORQUE_SETUP_CANID;
}

#else

uint32_t Pack_CONTROLLER_TORQUE_SETUP_hytech(CONTROLLER_TORQUE_SETUP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TORQUE_SETUP_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_initial_torque_req_ro = (int16_t) HYTECH_controller_initial_torque_req_ro_toS(_m->controller_initial_torque_req_phys);
  _m->controller_constrained_torq_req_ro = (int16_t) HYTECH_controller_constrained_torq_req_ro_toS(_m->controller_constrained_torq_req_phys);
  _m->controller_regen_limit_ro = (int16_t) HYTECH_controller_regen_limit_ro_toS(_m->controller_regen_limit_phys);
  _m->controller_torque_mode_ro = (uint8_t) HYTECH_controller_torque_mode_ro_toS(_m->controller_torque_mode_phys);
  _m->controller_max_yaw_nl_accel_perc_ro = (uint8_t) HYTECH_controller_max_yaw_nl_accel_perc_ro_toS(_m->controller_max_yaw_nl_accel_perc_phys);
  _m->controller_max_nl_brake_sc_front_ro = (uint8_t) HYTECH_controller_max_nl_brake_sc_front_ro_toS(_m->controller_max_nl_brake_sc_front_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_initial_torque_req_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_initial_torque_req_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_constrained_torq_req_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_constrained_torq_req_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_regen_limit_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_regen_limit_ro >> 8U) & (0x0FU)) | ((_m->controller_torque_mode_ro & (0x0FU)) << 4U) );
  _d[6] |= (uint8_t) ( ((_m->controller_torque_mode_ro >> 4U) & (0x0FU)) | ((_m->controller_max_yaw_nl_accel_perc_ro & (0x0FU)) << 4U) );
  _d[7] |= (uint8_t) ( (_m->controller_max_nl_brake_sc_front_ro & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TORQUE_SETUP_DLC;
  *_ide = (uint8_t) CONTROLLER_TORQUE_SETUP_IDE;
  return CONTROLLER_TORQUE_SETUP_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_launch_dead_zone_ro = (uint8_t) ( (_d[0] & (0xFFU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_launch_dead_zone_phys = (sigfloat_t)(HYTECH_controller_tcs_launch_dead_zone_ro_fromS(_m->controller_tcs_launch_dead_zone_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_launch_vel_thresh_ro = (uint8_t) ( (_d[1] & (0x3FU)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_launch_vel_thresh_phys = (sigfloat_t)(HYTECH_controller_tcs_launch_vel_thresh_ro_fromS(_m->controller_tcs_launch_vel_thresh_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_vel_thresh_ro = (uint8_t) ( ((_d[2] & (0x3FU)) << 2U) | ((_d[1] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_vel_thresh_phys = (sigfloat_t)(HYTECH_controller_tcs_vel_thresh_ro_fromS(_m->controller_tcs_vel_thresh_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_sl_nlperc_star_fr_ro = (uint8_t) ( ((_d[3] & (0x3FU)) << 2U) | ((_d[2] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_sl_nlperc_star_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_sl_nlperc_star_fr_ro_fromS(_m->controller_tcs_sl_nlperc_star_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_sl_nlperc_end_fr_ro = (uint8_t) ( ((_d[4] & (0x3FU)) << 2U) | ((_d[3] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_sl_nlperc_end_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_sl_nlperc_end_fr_ro_fromS(_m->controller_tcs_sl_nlperc_end_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_sl_nlperc_end_rr_ro = (uint8_t) ( ((_d[5] & (0x3FU)) << 2U) | ((_d[4] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_sl_nlperc_end_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_sl_nlperc_end_rr_ro_fromS(_m->controller_tcs_sl_nlperc_end_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_sl_nlperc_star_rr_ro = (uint8_t) ( ((_d[6] & (0x3FU)) << 2U) | ((_d[5] >> 6U) & (0x03U)) );
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_sl_nlperc_star_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_sl_nlperc_star_rr_ro_fromS(_m->controller_tcs_sl_nlperc_star_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_CONFIG_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_CONFIG_hytech(&_m->mon1, CONTROLLER_TCS_CONFIG_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_CONFIG_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_CONFIG_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_launch_dead_zone_ro = (uint8_t) HYTECH_controller_tcs_launch_dead_zone_ro_toS(_m->controller_tcs_launch_dead_zone_phys);
  _m->controller_tcs_launch_vel_thresh_ro = (uint8_t) HYTECH_controller_tcs_launch_vel_thresh_ro_toS(_m->controller_tcs_launch_vel_thresh_phys);
  _m->controller_tcs_vel_thresh_ro = (uint8_t) HYTECH_controller_tcs_vel_thresh_ro_toS(_m->controller_tcs_vel_thresh_phys);
  _m->controller_tcs_sl_nlperc_star_fr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_star_fr_ro_toS(_m->controller_tcs_sl_nlperc_star_fr_phys);
  _m->controller_tcs_sl_nlperc_end_fr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_end_fr_ro_toS(_m->controller_tcs_sl_nlperc_end_fr_phys);
  _m->controller_tcs_sl_nlperc_end_rr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_end_rr_ro_toS(_m->controller_tcs_sl_nlperc_end_rr_phys);
  _m->controller_tcs_sl_nlperc_star_rr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_star_rr_ro_toS(_m->controller_tcs_sl_nlperc_star_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_launch_dead_zone_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->controller_tcs_launch_vel_thresh_ro & (0x3FU)) | ((_m->controller_tcs_vel_thresh_ro & (0x03U)) << 6U) );
  cframe->Data[2] |= (uint8_t) ( ((_m->controller_tcs_vel_thresh_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_star_fr_ro & (0x03U)) << 6U) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_star_fr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_end_fr_ro & (0x03U)) << 6U) );
  cframe->Data[4] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_end_fr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_end_rr_ro & (0x03U)) << 6U) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_end_rr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_star_rr_ro & (0x03U)) << 6U) );
  cframe->Data[6] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_star_rr_ro >> 2U) & (0x3FU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_CONFIG_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_CONFIG_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_CONFIG_IDE;
  return CONTROLLER_TCS_CONFIG_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_CONFIG_hytech(CONTROLLER_TCS_CONFIG_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_CONFIG_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_launch_dead_zone_ro = (uint8_t) HYTECH_controller_tcs_launch_dead_zone_ro_toS(_m->controller_tcs_launch_dead_zone_phys);
  _m->controller_tcs_launch_vel_thresh_ro = (uint8_t) HYTECH_controller_tcs_launch_vel_thresh_ro_toS(_m->controller_tcs_launch_vel_thresh_phys);
  _m->controller_tcs_vel_thresh_ro = (uint8_t) HYTECH_controller_tcs_vel_thresh_ro_toS(_m->controller_tcs_vel_thresh_phys);
  _m->controller_tcs_sl_nlperc_star_fr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_star_fr_ro_toS(_m->controller_tcs_sl_nlperc_star_fr_phys);
  _m->controller_tcs_sl_nlperc_end_fr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_end_fr_ro_toS(_m->controller_tcs_sl_nlperc_end_fr_phys);
  _m->controller_tcs_sl_nlperc_end_rr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_end_rr_ro_toS(_m->controller_tcs_sl_nlperc_end_rr_phys);
  _m->controller_tcs_sl_nlperc_star_rr_ro = (uint8_t) HYTECH_controller_tcs_sl_nlperc_star_rr_ro_toS(_m->controller_tcs_sl_nlperc_star_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_launch_dead_zone_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->controller_tcs_launch_vel_thresh_ro & (0x3FU)) | ((_m->controller_tcs_vel_thresh_ro & (0x03U)) << 6U) );
  _d[2] |= (uint8_t) ( ((_m->controller_tcs_vel_thresh_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_star_fr_ro & (0x03U)) << 6U) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_star_fr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_end_fr_ro & (0x03U)) << 6U) );
  _d[4] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_end_fr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_end_rr_ro & (0x03U)) << 6U) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_end_rr_ro >> 2U) & (0x3FU)) | ((_m->controller_tcs_sl_nlperc_star_rr_ro & (0x03U)) << 6U) );
  _d[6] |= (uint8_t) ( ((_m->controller_tcs_sl_nlperc_star_rr_ro >> 2U) & (0x3FU)) );

  *_len = (uint8_t) CONTROLLER_TCS_CONFIG_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_CONFIG_IDE;
  return CONTROLLER_TCS_CONFIG_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_torque_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_fl_phys = (sigfloat_t)(HYTECH_controller_tcs_torque_fl_ro_fromS(_m->controller_tcs_torque_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_torque_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_torque_fr_ro_fromS(_m->controller_tcs_torque_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_torque_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_rl_phys = (sigfloat_t)(HYTECH_controller_tcs_torque_rl_ro_fromS(_m->controller_tcs_torque_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_torque_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_torque_rr_ro_fromS(_m->controller_tcs_torque_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_TORQUE_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_TORQUE_hytech(&_m->mon1, CONTROLLER_TCS_TORQUE_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_TORQUE_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_TORQUE_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_fl_ro = (int16_t) HYTECH_controller_tcs_torque_fl_ro_toS(_m->controller_tcs_torque_fl_phys);
  _m->controller_tcs_torque_fr_ro = (int16_t) HYTECH_controller_tcs_torque_fr_ro_toS(_m->controller_tcs_torque_fr_phys);
  _m->controller_tcs_torque_rl_ro = (int16_t) HYTECH_controller_tcs_torque_rl_ro_toS(_m->controller_tcs_torque_rl_phys);
  _m->controller_tcs_torque_rr_ro = (int16_t) HYTECH_controller_tcs_torque_rr_ro_toS(_m->controller_tcs_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_torque_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_torque_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_torque_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_torque_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_tcs_torque_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_torque_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_tcs_torque_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_torque_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_TORQUE_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_TORQUE_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_TORQUE_IDE;
  return CONTROLLER_TCS_TORQUE_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_TORQUE_hytech(CONTROLLER_TCS_TORQUE_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_TORQUE_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_torque_fl_ro = (int16_t) HYTECH_controller_tcs_torque_fl_ro_toS(_m->controller_tcs_torque_fl_phys);
  _m->controller_tcs_torque_fr_ro = (int16_t) HYTECH_controller_tcs_torque_fr_ro_toS(_m->controller_tcs_torque_fr_phys);
  _m->controller_tcs_torque_rl_ro = (int16_t) HYTECH_controller_tcs_torque_rl_ro_toS(_m->controller_tcs_torque_rl_phys);
  _m->controller_tcs_torque_rr_ro = (int16_t) HYTECH_controller_tcs_torque_rr_ro_toS(_m->controller_tcs_torque_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_torque_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_torque_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_torque_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_torque_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_tcs_torque_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_torque_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_tcs_torque_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_torque_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_TORQUE_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_TORQUE_IDE;
  return CONTROLLER_TCS_TORQUE_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_status_fl = (uint8_t) ( (_d[0] & (0x03U)) );
  _m->controller_tcs_status_fr = (uint8_t) ( ((_d[0] >> 2U) & (0x03U)) );
  _m->controller_tcs_status_rl = (uint8_t) ( ((_d[0] >> 4U) & (0x03U)) );
  _m->controller_tcs_status_rr = (uint8_t) ( ((_d[0] >> 6U) & (0x03U)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_STATUS_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_STATUS_hytech(&_m->mon1, CONTROLLER_TCS_STATUS_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_STATUS_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_STATUS_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_status_fl & (0x03U)) | ((_m->controller_tcs_status_fr & (0x03U)) << 2U) | ((_m->controller_tcs_status_rl & (0x03U)) << 4U) | ((_m->controller_tcs_status_rr & (0x03U)) << 6U) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_STATUS_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_STATUS_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_STATUS_IDE;
  return CONTROLLER_TCS_STATUS_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_STATUS_hytech(CONTROLLER_TCS_STATUS_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_STATUS_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->controller_tcs_status_fl & (0x03U)) | ((_m->controller_tcs_status_fr & (0x03U)) << 2U) | ((_m->controller_tcs_status_rl & (0x03U)) << 4U) | ((_m->controller_tcs_status_rr & (0x03U)) << 6U) );

  *_len = (uint8_t) CONTROLLER_TCS_STATUS_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_STATUS_IDE;
  return CONTROLLER_TCS_STATUS_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->controller_tcs_pid_input_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_fl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_input_fl_ro_fromS(_m->controller_tcs_pid_input_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_input_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_fr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_input_fr_ro_fromS(_m->controller_tcs_pid_input_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_input_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_rl_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_input_rl_ro_fromS(_m->controller_tcs_pid_input_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->controller_tcs_pid_input_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_rr_phys = (sigfloat_t)(HYTECH_controller_tcs_pid_input_rr_ro_fromS(_m->controller_tcs_pid_input_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CONTROLLER_TCS_PID_INPUT_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CONTROLLER_TCS_PID_INPUT_hytech(&_m->mon1, CONTROLLER_TCS_PID_INPUT_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CONTROLLER_TCS_PID_INPUT_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_INPUT_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_fl_ro = (int16_t) HYTECH_controller_tcs_pid_input_fl_ro_toS(_m->controller_tcs_pid_input_fl_phys);
  _m->controller_tcs_pid_input_fr_ro = (int16_t) HYTECH_controller_tcs_pid_input_fr_ro_toS(_m->controller_tcs_pid_input_fr_phys);
  _m->controller_tcs_pid_input_rl_ro = (int16_t) HYTECH_controller_tcs_pid_input_rl_ro_toS(_m->controller_tcs_pid_input_rl_phys);
  _m->controller_tcs_pid_input_rr_ro = (int16_t) HYTECH_controller_tcs_pid_input_rr_ro_toS(_m->controller_tcs_pid_input_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->controller_tcs_pid_input_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->controller_tcs_pid_input_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->controller_tcs_pid_input_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->controller_tcs_pid_input_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->controller_tcs_pid_input_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->controller_tcs_pid_input_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->controller_tcs_pid_input_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->controller_tcs_pid_input_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) CONTROLLER_TCS_PID_INPUT_CANID;
  cframe->DLC = (uint8_t) CONTROLLER_TCS_PID_INPUT_DLC;
  cframe->IDE = (uint8_t) CONTROLLER_TCS_PID_INPUT_IDE;
  return CONTROLLER_TCS_PID_INPUT_CANID;
}

#else

uint32_t Pack_CONTROLLER_TCS_PID_INPUT_hytech(CONTROLLER_TCS_PID_INPUT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CONTROLLER_TCS_PID_INPUT_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->controller_tcs_pid_input_fl_ro = (int16_t) HYTECH_controller_tcs_pid_input_fl_ro_toS(_m->controller_tcs_pid_input_fl_phys);
  _m->controller_tcs_pid_input_fr_ro = (int16_t) HYTECH_controller_tcs_pid_input_fr_ro_toS(_m->controller_tcs_pid_input_fr_phys);
  _m->controller_tcs_pid_input_rl_ro = (int16_t) HYTECH_controller_tcs_pid_input_rl_ro_toS(_m->controller_tcs_pid_input_rl_phys);
  _m->controller_tcs_pid_input_rr_ro = (int16_t) HYTECH_controller_tcs_pid_input_rr_ro_toS(_m->controller_tcs_pid_input_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->controller_tcs_pid_input_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->controller_tcs_pid_input_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->controller_tcs_pid_input_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->controller_tcs_pid_input_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->controller_tcs_pid_input_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->controller_tcs_pid_input_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->controller_tcs_pid_input_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->controller_tcs_pid_input_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) CONTROLLER_TCS_PID_INPUT_DLC;
  *_ide = (uint8_t) CONTROLLER_TCS_PID_INPUT_IDE;
  return CONTROLLER_TCS_PID_INPUT_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->vehm_alpha_deg_fl_ro = (int16_t) __ext_sig__(( ((_d[1] & (0xFFU)) << 8U) | (_d[0] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_fl_phys = (sigfloat_t)(HYTECH_vehm_alpha_deg_fl_ro_fromS(_m->vehm_alpha_deg_fl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_alpha_deg_fr_ro = (int16_t) __ext_sig__(( ((_d[3] & (0xFFU)) << 8U) | (_d[2] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_fr_phys = (sigfloat_t)(HYTECH_vehm_alpha_deg_fr_ro_fromS(_m->vehm_alpha_deg_fr_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_alpha_deg_rl_ro = (int16_t) __ext_sig__(( ((_d[5] & (0xFFU)) << 8U) | (_d[4] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_rl_phys = (sigfloat_t)(HYTECH_vehm_alpha_deg_rl_ro_fromS(_m->vehm_alpha_deg_rl_ro));
#endif // HYTECH_USE_SIGFLOAT

  _m->vehm_alpha_deg_rr_ro = (int16_t) __ext_sig__(( ((_d[7] & (0xFFU)) << 8U) | (_d[6] & (0xFFU)) ), 16);
#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_rr_phys = (sigfloat_t)(HYTECH_vehm_alpha_deg_rr_ro_fromS(_m->vehm_alpha_deg_rr_ro));
#endif // HYTECH_USE_SIGFLOAT

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < VEHM_ALPHA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_VEHM_ALPHA_hytech(&_m->mon1, VEHM_ALPHA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return VEHM_ALPHA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_ALPHA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_fl_ro = (int16_t) HYTECH_vehm_alpha_deg_fl_ro_toS(_m->vehm_alpha_deg_fl_phys);
  _m->vehm_alpha_deg_fr_ro = (int16_t) HYTECH_vehm_alpha_deg_fr_ro_toS(_m->vehm_alpha_deg_fr_phys);
  _m->vehm_alpha_deg_rl_ro = (int16_t) HYTECH_vehm_alpha_deg_rl_ro_toS(_m->vehm_alpha_deg_rl_phys);
  _m->vehm_alpha_deg_rr_ro = (int16_t) HYTECH_vehm_alpha_deg_rr_ro_toS(_m->vehm_alpha_deg_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  cframe->Data[0] |= (uint8_t) ( (_m->vehm_alpha_deg_fl_ro & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( ((_m->vehm_alpha_deg_fl_ro >> 8U) & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->vehm_alpha_deg_fr_ro & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( ((_m->vehm_alpha_deg_fr_ro >> 8U) & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->vehm_alpha_deg_rl_ro & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( ((_m->vehm_alpha_deg_rl_ro >> 8U) & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->vehm_alpha_deg_rr_ro & (0xFFU)) );
  cframe->Data[7] |= (uint8_t) ( ((_m->vehm_alpha_deg_rr_ro >> 8U) & (0xFFU)) );

  cframe->MsgId = (uint32_t) VEHM_ALPHA_CANID;
  cframe->DLC = (uint8_t) VEHM_ALPHA_DLC;
  cframe->IDE = (uint8_t) VEHM_ALPHA_IDE;
  return VEHM_ALPHA_CANID;
}

#else

uint32_t Pack_VEHM_ALPHA_hytech(VEHM_ALPHA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(VEHM_ALPHA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

#ifdef HYTECH_USE_SIGFLOAT
  _m->vehm_alpha_deg_fl_ro = (int16_t) HYTECH_vehm_alpha_deg_fl_ro_toS(_m->vehm_alpha_deg_fl_phys);
  _m->vehm_alpha_deg_fr_ro = (int16_t) HYTECH_vehm_alpha_deg_fr_ro_toS(_m->vehm_alpha_deg_fr_phys);
  _m->vehm_alpha_deg_rl_ro = (int16_t) HYTECH_vehm_alpha_deg_rl_ro_toS(_m->vehm_alpha_deg_rl_phys);
  _m->vehm_alpha_deg_rr_ro = (int16_t) HYTECH_vehm_alpha_deg_rr_ro_toS(_m->vehm_alpha_deg_rr_phys);
#endif // HYTECH_USE_SIGFLOAT

  _d[0] |= (uint8_t) ( (_m->vehm_alpha_deg_fl_ro & (0xFFU)) );
  _d[1] |= (uint8_t) ( ((_m->vehm_alpha_deg_fl_ro >> 8U) & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->vehm_alpha_deg_fr_ro & (0xFFU)) );
  _d[3] |= (uint8_t) ( ((_m->vehm_alpha_deg_fr_ro >> 8U) & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->vehm_alpha_deg_rl_ro & (0xFFU)) );
  _d[5] |= (uint8_t) ( ((_m->vehm_alpha_deg_rl_ro >> 8U) & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->vehm_alpha_deg_rr_ro & (0xFFU)) );
  _d[7] |= (uint8_t) ( ((_m->vehm_alpha_deg_rr_ro >> 8U) & (0xFFU)) );

  *_len = (uint8_t) VEHM_ALPHA_DLC;
  *_ide = (uint8_t) VEHM_ALPHA_IDE;
  return VEHM_ALPHA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->max_charging_voltage_high = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->max_charging_voltage_low = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->max_charging_current_high = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->max_charging_current_low = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->control = (uint8_t) ( (_d[4] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CHARGER_CONTROL_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CHARGER_CONTROL_hytech(&_m->mon1, CHARGER_CONTROL_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CHARGER_CONTROL_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CHARGER_CONTROL_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->max_charging_voltage_high & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->max_charging_voltage_low & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->max_charging_current_high & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->max_charging_current_low & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->control & (0xFFU)) );

  cframe->MsgId = (uint32_t) CHARGER_CONTROL_CANID;
  cframe->DLC = (uint8_t) CHARGER_CONTROL_DLC;
  cframe->IDE = (uint8_t) CHARGER_CONTROL_IDE;
  return CHARGER_CONTROL_CANID;
}

#else

uint32_t Pack_CHARGER_CONTROL_hytech(CHARGER_CONTROL_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CHARGER_CONTROL_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->max_charging_voltage_high & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->max_charging_voltage_low & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->max_charging_current_high & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->max_charging_current_low & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->control & (0xFFU)) );

  *_len = (uint8_t) CHARGER_CONTROL_DLC;
  *_ide = (uint8_t) CHARGER_CONTROL_IDE;
  return CHARGER_CONTROL_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

uint32_t Unpack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->output_dc_voltage_high = (uint8_t) ( (_d[0] & (0xFFU)) );
  _m->output_dc_voltage_low = (uint8_t) ( (_d[1] & (0xFFU)) );
  _m->output_current_high = (uint8_t) ( (_d[2] & (0xFFU)) );
  _m->output_current_low = (uint8_t) ( (_d[3] & (0xFFU)) );
  _m->flags = (uint8_t) ( (_d[4] & (0xFFU)) );
  _m->input_ac_voltage_high = (uint8_t) ( (_d[5] & (0xFFU)) );
  _m->input_ac_voltage_low = (uint8_t) ( (_d[6] & (0xFFU)) );

#ifdef HYTECH_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < CHARGER_DATA_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_CHARGER_DATA_hytech(&_m->mon1, CHARGER_DATA_CANID);
#endif // HYTECH_USE_DIAG_MONITORS

  return CHARGER_DATA_CANID;
}

#ifdef HYTECH_USE_CANSTRUCT

uint32_t Pack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CHARGER_DATA_DLC); cframe->Data[i++] = HYTECH_INITIAL_BYTE_VALUE);

  cframe->Data[0] |= (uint8_t) ( (_m->output_dc_voltage_high & (0xFFU)) );
  cframe->Data[1] |= (uint8_t) ( (_m->output_dc_voltage_low & (0xFFU)) );
  cframe->Data[2] |= (uint8_t) ( (_m->output_current_high & (0xFFU)) );
  cframe->Data[3] |= (uint8_t) ( (_m->output_current_low & (0xFFU)) );
  cframe->Data[4] |= (uint8_t) ( (_m->flags & (0xFFU)) );
  cframe->Data[5] |= (uint8_t) ( (_m->input_ac_voltage_high & (0xFFU)) );
  cframe->Data[6] |= (uint8_t) ( (_m->input_ac_voltage_low & (0xFFU)) );

  cframe->MsgId = (uint32_t) CHARGER_DATA_CANID;
  cframe->DLC = (uint8_t) CHARGER_DATA_DLC;
  cframe->IDE = (uint8_t) CHARGER_DATA_IDE;
  return CHARGER_DATA_CANID;
}

#else

uint32_t Pack_CHARGER_DATA_hytech(CHARGER_DATA_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0u; i < HYTECH_VALIDATE_DLC(CHARGER_DATA_DLC); _d[i++] = HYTECH_INITIAL_BYTE_VALUE);

  _d[0] |= (uint8_t) ( (_m->output_dc_voltage_high & (0xFFU)) );
  _d[1] |= (uint8_t) ( (_m->output_dc_voltage_low & (0xFFU)) );
  _d[2] |= (uint8_t) ( (_m->output_current_high & (0xFFU)) );
  _d[3] |= (uint8_t) ( (_m->output_current_low & (0xFFU)) );
  _d[4] |= (uint8_t) ( (_m->flags & (0xFFU)) );
  _d[5] |= (uint8_t) ( (_m->input_ac_voltage_high & (0xFFU)) );
  _d[6] |= (uint8_t) ( (_m->input_ac_voltage_low & (0xFFU)) );

  *_len = (uint8_t) CHARGER_DATA_DLC;
  *_ide = (uint8_t) CHARGER_DATA_IDE;
  return CHARGER_DATA_CANID;
}

#endif // HYTECH_USE_CANSTRUCT

