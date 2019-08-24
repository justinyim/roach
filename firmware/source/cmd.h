#ifndef __CMD_H
#define __CMD_H


#include "mac_packet.h"
#include "cmd_const.h"

#define CMD_TEST_RADIO				0x00
#define CMD_TEST_MPU				0x06
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

#define CMD_SET_PITCH_SET           0x96
#define CMD_RESET_BODY_ANG          0x97
#define CMD_SET_CURRENT_LIMITS      0x98
#define CMD_SET_MOTOR_POS           0x99
#define CMD_START_EXP               0x9A
#define CMD_SET_EXP_PARAMS          0x9B

#define CMD_STOP_EXP                0xA0
#define CMD_INTEGRATED_VICON        0xA1
#define CMD_CALIBRATE_MOTOR         0xA2
#define CMD_ONBOARD_MODE            0xA3
#define CMD_GYRO_BIAS               0xA4
#define CMD_G_VECT_ATT              0xA5
#define CMD_SET_VELOCITY            0xA6
#define CMD_ADJUST_BODY_ANG         0xA7
#define CMD_TILT                    0xA8
#define CMD_STANCE                  0xA9
#define CMD_SET_MOCAP_VEL           0xAA
// Redefine

void cmdSetup(void);
void cmdPushFunc(MacPacket rx_packet);


#endif // __CMD_H
