#ifndef __SALTO1P_H
#define __SALTO1P_H

#include <stdint.h>
#include "protocol.h"

// States for mj_state
#define MJ_IDLE         0
#define MJ_START        1
#define MJ_STOP         2
#define MJ_AIR          3
#define MJ_GND          4
#define MJ_STOPPED      5
#define MJ_LAUNCH       6
#define MJ_STAND        7

#define UART_PERIOD     8000 // minimum UART period in us (must be positive integer)

// Parameters for cosApprox
#define PI          2949120 // 180(deg) * 2^15(ticks)/2000(deg/s) * 1000(Hz)
#define PI_SQUARE   32400   // bit shifted down by 28 bits
#define COS_PREC    8       // bits of precision in output of cosine

#define GRAV_ACC 39         // -9.81 m/s^2 * (2^2 ticks/(m/s^2))

#define MAX_THROT 3800
#define MAX_TAIL 3960

// Setup
void salto1pSetup(void);
void SetupTimer5(void);
void salto1p_functions(void);

// Jump and attitude estimation and control
void eulerUpdate(int32_t* angs, int32_t* vels, int8_t time);
void kinematicUpdate(void);
void jumpModes(void);
void attitudeCtrl(void);
void swingUpCtrl(void);

// Additional onboard estimation and control
void modeEstimation(void);
void stanceUpdate(void);
void flightUpdate(void);
void flightStanceTrans(void);
void stanceFlightTrans(void);
void takeoffEstimation(void);
void legCtrl(void);
void balanceCtrl(void);
void balanceOffsetEstimator(void);
int32_t deadbeatVelCtrl(int16_t* vi, int16_t* vo, int32_t* ctrl);
void attitudeActuators(int32_t roll, int32_t pitch, int32_t yaw);
int32_t tailLinearization(int32_t* tail);
int32_t thrusterLinearization(int32_t* thruster, int16_t velocity);

// Radio commands
void setGains(int16_t* gains);
void setAttitudeSetpoint(long yaw, long roll, long pitch);
void setLegSetpoint(long length);
void setPushoffCmd(long cmd);
void setMotorPos(uint32_t gain, long pos);
void setBodyAngle(long* qSet) ;
void adjustBodyAngle(long* qAdjust);
void updateBodyAngle(long* qUpdate);
void accZeroAtt(void);
void calibGyroBias(void);
void expStart(uint8_t startSignal);
void expStop(uint8_t stopSignal);
void setOnboardMode(uint8_t flags, uint8_t mode);
void setVelocitySetpoint(int16_t* newCmd, int32_t newYaw);
void setTilt(int16_t u_in, int16_t ud_in, int16_t udd_in, int16_t uddd_in);
void setLeg(int16_t rdes_in, int16_t rddes_in, int16_t rdddes_in, int16_t k1des_in, int16_t k2des_in);

// Other
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);

// Utility functions
void orientImageproc(int32_t* v_b, int16_t* v_ip);
int32_t calibPos(uint8_t idx);
int32_t forceControl(int16_t length, int16_t p, int16_t d, int16_t f, int16_t addon);
int32_t forceSetpoint(int16_t r_des, int16_t rd_des, int16_t rdd_des, int16_t k1, int16_t k2);
int32_t cmdLegLen(int16_t leg);
int32_t cosApprox(int32_t x);
int32_t sqrtApprox(int32_t n);
uint16_t med3(uint16_t* arr);



#endif // __SALTO1P_H