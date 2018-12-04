#ifndef __SALTO_CTRL_H
#define __SALTO_CTRL_H

#include <stdint.h>
#include "protocol.h"

#define MJ_IDLE         0
#define MJ_START        1
#define MJ_STOP         2
#define MJ_AIR          3
#define MJ_GND          4
#define MJ_STOPPED      5
#define MJ_LAUNCH       6

#define UART_PERIOD     10 // minimum UART period in ms (must be positive integer)

// Parameters for cosApprox
#define PI          2949120 // 180(deg) * 2^15(ticks)/2000(deg/s) * 1000(Hz)
#define PISQUARED   32400 // bit shifted 28 bits down
#define COS_PREC    8 // bits of precision in output of cosine

#define ATT_CORRECTION_GAIN_X 12   // 8 is about equal to PI/180/2000 for 1 degree correction per 1 meter per second
#define ATT_CORRECTION_GAIN_Y 8

//#define FULL_POWER


void SetupTimer5();
void tailCtrlSetup();

void setControlFlag(char state);
void resetBodyAngle();
void calibGyroBias();
void accZeroAtt();
void setOnboardMode(char mode, char flags); 
void updateViconAngle(long* new_vicon_angle);
void adjustBodyAngle(long* body_adjust);

void setAttitudeSetpoint(long yaw, long roll, long pitch);
void setLegSetpoint(long length);
void setPushoffCmd(long cmd);
void setVelocitySetpoint(int16_t* new_vel_des, long new_yaw);

void expStart(uint8_t startSignal);
void expStop(uint8_t stopSignal);

void multiJumpFlow();
char footContact(void);
char footTakeoff(void);
 
long calibPos(char idx);
void femurLUTs(void);
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);

void orientImageProc(long* body_frame, int* ImageProc_frame);
void updateEuler(long* angs, long* vels, long time);
long cosApprox(long x);

void updateVelocity(long time);
void raibert();
long deadbeat(int16_t* vi, int16_t* vo, long* ctrl);

#endif // __SALTO_CTRL_H
