#ifndef __CMD_H
#define __CMD_H


#include "mac_packet.h"
#include "cmd_const.h"
#include "stdint.h"

//// Includes here should be to provide TYPES and ENUMS only
#include "pid-ip2.5.h"

#define CMD_TEST_RADIO              0x00
#define CMD_TEST_MPU                0x06
#define CMD_SET_THRUST_OPEN_LOOP    0x80
#define CMD_PID_START_MOTORS        0x81
#define CMD_SET_PID_GAINS           0x82
#define CMD_GET_AMS_POS             0x84
#define CMD_ERASE_SECTORS           0x8A 
#define CMD_FLASH_READBACK          0x8B 
#define CMD_SET_VEL_PROFILE         0x8D
#define CMD_WHO_AM_I                0x8E
#define CMD_START_TELEMETRY         0x8F
#define CMD_ZERO_POS                0x90
#define CMD_START_TIMED_RUN         0x91 
#define CMD_PID_STOP_MOTORS         0x92         
#define CMD_SET_PHASE               0x93         
#define CMD_SET_MOTOR_MODE          0x94
#define CMD_SET_SERVO               0x95 // JY edits adding servo angle command
#define CMD_CALIB_INIT              0x96 // JY edits adding initial calibration
#define CMD_SALTO_VICON1            0x97 // JY edits adding Salto Vicon jumps
// Redefine

void cmdSetup(void);
void cmdHandleRadioRxBuffer(void);
//void cmdPushFunc(MacPacket rx_packet);


/////// Argument structures

//cmdSetThrustOpenLoop
typedef struct{
	int channel, dc;
} _args_cmdSetThrustOpenLoop;


//cmdSetMotorMode
typedef struct{
	int thrust1, thrust2;
} _args_cmdSetMotorMode;

//cmdSetPIDGains
typedef struct{
	int Kp1, Ki1, Kd1, Kaw1, Kff1;
	int Kp2, Ki2, Kd2, Kaw2, Kff2;
} _args_cmdSetPIDGains;

//cmdSoftwareReset
//no arguments

//cmdcmdStartTimedRun
typedef struct{
    uint16_t run_time;
} _args_cmdStartTimedRun;

//cmdStartTelemetry
typedef struct{
    uint32_t numSamples;
} _args_cmdStartTelemetry;

//cmdEraseSector
typedef struct{
    uint32_t samples;
} _args_cmdEraseSector;

//cmdFlashReadback
typedef struct{
    uint32_t samples;
} _args_cmdFlashReadback;

//cmdSetVelProfile
typedef struct{
    int16_t periodLeft;
    int16_t deltaL[NUM_VELS];
    int16_t flagLeft;
    int16_t periodRight;
    int16_t deltaR[NUM_VELS];
    int16_t flagRight;
} _args_cmdSetVelProfile;

//cmdSetPhase
typedef struct{
    int32_t offset;
} _args_cmdSetPhase;

//cmdSetServo // JY edits
typedef struct{
    float angle;
} _args_cmdSetServo;

//cmdSaltoVicon1 // JY edits
typedef struct{
    unsigned char rol;
    unsigned char pit;
    unsigned char yaw;
    unsigned char thb;
    unsigned char thl;
    unsigned char Il;
} _args_cmdSaltoVicon1;

//cmdCalibInit // JY edits
//no arguments

#endif // __CMD_H
