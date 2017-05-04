#ifndef __SALTO_CTRL_H
#define __SALTO_CTRL_H

#include <stdint.h>
#include "protocol.h"

#define MJ_IDLE         0
#define MJ_START        1
#define MJ_STOP         2
#define MJ_AIR          3
#define MJ_GND          4

#define UART_PERIOD     10 // minimum UART period in ms (must be positive integer)
#define FULL_EXTENSION  14500 // TODO: put in settings.h


void SetupTimer5();
void tailCtrlSetup();


void setPitchControlFlag(char state);
void resetBodyAngle();
void updateViconAngle(long* new_vicon_angle);

void setAttitudeSetpoint(long yaw, long roll, long pitch);
void setLegSetpoint(long length);
void setPushoffCmd(long cmd);

void expStart(uint8_t startSignal);
void expStop(uint8_t stopSignal);

void multiJumpFlow();
char footContact(void);
char footTakeoff(void);
 
long calibPos(char idx);
unsigned int crankFromFemur(void);
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);

#endif // __SALTO_CTRL_H
