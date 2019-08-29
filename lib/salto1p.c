
/*
 * Name: salto1p.c
 * Desc: Integrated Salto-1P control
 * Date: 2018-12-04
 * Author: JKY (Justin Yim)
 */

#include <stdlib.h> // for malloc

// Salto-specific
#include "salto1p.h"
#include "settings.h"
#include "lut.h"

// ImageProc2.5
#include "init.h"  // for Timer1
#include "sclock.h"
#include "timer.h"
#include "led.h"
#include "ports.h"
#include "adc.h"
#include "adc_pid.h"
#include "pwm.h"
#include "p33Fxxxx.h"
#include "ppool.h"
#include "dfmem.h"
#include "telem.h"

#include "tih.h"
#include "mpu6000.h"
#include "as5047.h"
#include "ams-enc.h"
#include "protocol.h"
#include "uart_driver.h"


// Initialization =============================================================
// Constants ------------------------------------------------------------------
// Control and estimation gains
int32_t gainsPD[10];      // PD controller gains (yaw, rol, pit) (P, D, other)

#define TAIL_ALPHA 25 // Low pass tail velocity out of 128
#define W_ALPHA 1  // Low pass body angular velocity out of 4 (RC = 0.006 s)
#define TAIL_CMD_ALPHA 1  // Low pass tail PWM out of 8

#define P_AIR ((3*65536)/100) // leg proportional gain in the air (duty cycle/rad * 65536)
#define D_AIR ((0*65536)/1000) // leg derivative gain in the air (duty cycle/[rad/s] * 65536)
#define P_GND ((3*65536)/10) // leg proportional gain on the ground
#define D_GND ((2*65536)/1000)
#define P_STAND ((2*65536)/100) //((1*65536)/10) // leg proportional gain for standing
#define D_STAND ((5*65536)/10000) //((1*65536)/1000)

uint32_t GAINS_AIR = (P_AIR<<16)+D_AIR;
uint32_t GAINS_GND = (P_GND<<16)+D_GND;
uint32_t GAINS_STAND = (P_STAND<<16)+D_STAND;
uint32_t GAINS_ENERGY = 5*655*65536 + 20*7; // P/100, D/10,000

#define TELEM_DECIMATE 2
int32_t telemDecimateCount = 0;
#define T1_MAX 0xffffff

#define LAG_MS 20 // Vicon lag in milliseconds

// Global Variables -----------------------------------------------------------
// Miscellaneous important variables
uint16_t procFlags = 0;     // Which functions to process
    // 1: Run takeoff processes immediately after takeoff
    // 2: Takeoff correction complete: apply it to attitude estimate
uint8_t modeFlags = 0;     // Running modes
    // 1: stance balance control enabled (1) or default aerial balance (0)
    // 2: Takeoff vel attitude correction (SHOVE) enabled (1) or disabled (0)
    // 4: use onboard velocity control (1) or accept offboard attitude cmd (0)
    // 8: use onboard trajectory (1) or accept offboard horz. velocity cmd (0)
    // 16: static standing balance (1) or usual operation (0)
    // 32: don't turn off when the robot flips over
    // 1 << 6: swing-up controller
    // 2 << 6: 
    // 3 << 6: 
uint32_t t1_ticks = 0;      // Time (1 tick per ms)
uint16_t t5_count = 0;
uint8_t interrupt_count = 0;// How many processing cycles have passed

// Continuous dynamics state variables
int32_t q[3];               // ZXY Body Euler angles (x, y, z) [2*PI ticks/rev]
int32_t w[3];               // Body ang vel (x, y, z) [2^15 ticks/(2000 deg/s)]
int32_t p[3];               // Robot position (x, y, z) [100,000 ticks/m]
int16_t v[3];               // Robot body vel (x, y, z) [2000 ticks/(m/s)]

int32_t tail_pos = 0;       // Tail angle [25/48*2^16 ticks/(2*pi rad)]
int32_t tail_prev = 0;      // Previous tail angle for velocity estimation
int32_t tail_vel = 0;       // Tail vel [(25/48*2^16 ticks)/(2*pi*1000 rad/s)]

// Control output
int32_t foreCmd;       // Control output (linear in torque)
int32_t aftCmd;        // Control output (linear in torque)
int32_t tailCmd;       // Control output (linear in torque)

int32_t foreThruster;       // Control output PWM (after linearization)
int32_t aftThruster;        // Control output PWM (after linearization)
int32_t tailMotor;          // Control output PWM (after linearization)

// Commands
int32_t qCmd[3];            // Attitude setpoint
int32_t legSetpoint = 0;    // Aerial leg motor angle command
int32_t pushoffCmd = 0;     // Ground leg motor angle command

int16_t vCmd[3] = {0,0,5000};// Velocity command for takeoff [2000 ticks/(m/s)]
int32_t pCmd[2] = {0,0};    // Position command [100,000 ticks/m]

// Discrete mode variables
uint8_t mj_state = MJ_IDLE; // Jump mode
uint8_t last_state = MJ_IDLE;// Jump mode at last estimation step
int32_t transition_time = 0;// Time of last mode transition

// Estimation State and Intermediate Variables
//int32_t q0[3];              // Ang offset (rol, pit, yaw) [PI ticks/(pi rad)]

//int32_t qLagLog[LAG_MS][3]; // Attitude circular buffer for lag compensation
//int32_t qLagSum[3];         // Attitude circular buffer sum for lag comp.
//uint8_t qLagInd;            // Attitude circular buffer index

int32_t w500[3];            // Attitude sum for 500Hz Euler update
int32_t wLast[3];           // Last angular velocity for computing q500

int32_t gdataBody[3];       // Raw gyro data in the body frame
int32_t xldataBody[3];      // Raw accelerometer data in body frame
int16_t vB[3];              // CG vel in world aligned to the body-fixed frame

int32_t mot;                // Motor angle [2^14 ticks/rad at the gear]
int32_t last_mot;           // Last motor angle for rejecting bad samples
int32_t motw;               // Motor angVel. [2^15 ticks/(2000 deg/s) at motor]
int32_t femur;              // Femur angle [2^16 ticks/rot]
int32_t crank;              // Crank angle [2^14 ticks/rad]
int32_t foot;               // Foot distance [2^14 ticks/m]
int32_t ma;                 // Mech. adv. [2^9 ticks/(N/Nm)]

int32_t spring;             // Spring deflection [2^14 ticks/rad]
int32_t sTorque;            // Spring torque [2^14 ticks/(Nm)]
int32_t force;              // Foot force [2^10 ticks/N]
int16_t leg = 6000;         // Stance leg length [2^16 ticks/m]
int16_t legVel;             // Stance leg velocity [2000 ticks/(m/s)]

int32_t sin_theta = 0;      // pitch angle in COS_PREC bits
int32_t cos_theta = 1<<COS_PREC;
int32_t sin_phi = 0;        // roll angle
int32_t cos_phi = 1<<COS_PREC;
int32_t sin_psi = 0;        // yaw angle
int32_t cos_psi = 1<<COS_PREC;

int32_t returnable; // TODO delete only for testing purposes


// Attitude actuator notch filters
//int16_t rolI[3];
//int16_t rolO[3];
//int16_t yawI[3];
//int16_t yawO[3];
int32_t pitI[3];
int32_t pitO[3];
int32_t wyI[3];  // Angular velocity notch filter
int32_t wyO[3];

int16_t foreVel = 0;        // Thruster velocity from -4096 to 4088
int16_t aftVel = 0;         // Thruster velocity from -4096 to 4088
uint8_t tbInd = 0;          // Thruster velocity buffer index
uint8_t tbIndPrev = 2;      // Thruster velocity buffer index at previous time
uint16_t foreBuff[3];       // Thruster velocity buffer
uint16_t aftBuff[3];        // Thruster velocity buffer

// Balance control and estimation
int32_t q0offset;           // balance offset estimator
int32_t q1offset;           // balance offset estimator
int32_t wLP[3];             // Angular velocity low-pass filter
int32_t foreLP = 0;         // Thruster low-pass
int32_t aftLP = 0;          // Thruster low-pass
int32_t tauX = 0;           // Torque in stance (see balanceOffsetEstimator)
int32_t tauY = 0;           // Torque in stance (see balanceOffsetEstimator)
#define N_MBUFF 10          // circular buffer length
#define BOE_DEC 4           // Balance off. est. decimation
int32_t MxBuff[N_MBUFF];    // x angular momentum circular buffer
int32_t MyBuff[N_MBUFF];    // y angular momentum circular buffer
int32_t tauXBuff[N_MBUFF];  // x torque circular buffer
int32_t tauYBuff[N_MBUFF];  // y torque circular buffer
int32_t tauXSum = 0;        // sum of x torque buffer
int32_t tauYSum = 0;        // sum of y torque buffer
uint8_t Mind = 0;           // index circular buffers
int32_t u;                  // Tilt control
int32_t ud;                 // Tilt control
int32_t udd;                // Tilt control
int32_t uddd;               // Tilt control
int16_t rdes;               // Force control
int16_t rddes;              // Force control
int16_t rdddes;             // Force control
int16_t k1des;              // Force control
int16_t k2des;              // Force control
uint32_t u_time = 0;        // Time tilt command was set

int32_t I_cg = 734; // 2^20 ticks/(kg m^2)
int32_t Iy = 860; // 2^20 ticks/(kg m^2)
int32_t Ix = 832; // 2^20 ticks/(kg m^2)
int32_t mgc = 90521; // in 2^20 ticks/(N m)
int32_t uCmd;
        // 10 // 8 // 7 // 6+-3i // 6 
/*
#define K0 -1000//-343//-312//-216// in 1 tick/(rad/s^3)
#define K1 -307//-197//-151//-127//-111// in 1024/1000 tick/(rad/s^2)
#define K2 -30//-24//-21//-18//-18// in 1 tick/(rad/s)
#define CK0 100 // command filter 1/(z*z)
#define CK1 5 // command filter 1/((z+z)/(z*z))
*/
/*
// poles -14, -14, -10
#define K0 -1960// in 1 tick/(rad/s^3)
#define K1 -487// in 1024/1000 tick/(rad/s^2)
#define K2 -38// in 1 tick/(rad/s)
#define CK0 196 // command filter 1/(z*z)
#define CK1 7 // command filter 1/((z+z)/(z*z))
*/
/*
// poles -8, -8, -8
#define K0_UP -343 // in 1 tick/(rad/s^3)
#define K1_UP -197 // in 1024/1000 tick/(rad/s^2)
#define K2_UP -24 // in 1 tick/(rad/s)
#define CK0_UP 64 // command filter 1/(z*z)
#define CK1_UP 4 // command filter 1/((z+z)/(z*z))
*/

//*
#if ROBOT_NAME == SALTO_1P_DASHER
// poles -12, -12, -12
#define K0 -1728// in 1 tick/(rad/s^3) prod(poles)
#define K1 -442// in 1024/1000 tick/(rad/s^2) sum(prod(nchoosek(poles,2),2))
#define K2 -36// in 1 tick/(rad/s) sum(poles)
#define CK3 1728 // command filter (z*z*z)
#define CK2 48 // command filter 1/(3*(1/(z*z)))
#define CK1 4 // command filter 1/(1/z+1/z+1/z)
// poles -9, -9, -9
#define K0_UP -729 // in 1 tick/(rad/s^3)
#define K1_UP -249 // in 1024/1000 tick/(rad/s^2)
#define K2_UP -27 // in 1 tick/(rad/s)
#define CK3_UP 729 // command filter 1/(z*z)
#define CK2_UP 27 // command filter 1/((z+z)/(z*z))
#define CK1_UP 3 // command filter 1/(1/z+1/z+1/z)

#define THE_LIMIT 187750
#define PHI_LIMIT 93873

#elif ROBOT_NAME == SALTO_1P_RUDOLPH
// poles -9, -9, -9
#define K0 -729// in 1 tick/(rad/s^3)
#define K1 -249// in 1024/1000 tick/(rad/s^2)
#define K2 -27// in 1 tick/(rad/s)
#define CK3 729 // command filter (z*z*z)
#define CK2 27 // command filter 1/(3/(z*z))
#define CK1 3 // command filter 1/(1/z+1/z+1/z)
// poles -6, -6, -6
#define K0_UP -216 // in 1 tick/(rad/s^3)
#define K1_UP -111 // in 1024/1000 tick/(rad/s^2) 
#define K2_UP -18 // in 1 tick/(rad/s)
#define CK3_UP 216 // command filter (z*z*z)
#define CK2_UP 12 // command filter 1/(3/(z*z))
#define CK1_UP 2 // command filter 1/(1/z+1/z+1/z)

#define THE_LIMIT 187750
#define PHI_LIMIT 93873

#else
// poles -9, -9, -9
#define K0 -729// in 1 tick/(rad/s^3)
#define K1 -249// in 1024/1000 tick/(rad/s^2)
#define K2 -27// in 1 tick/(rad/s)
#define CK3 729 // command filter (z*z*z)
#define CK2 27 // command filter 1/(3/(z*z))
#define CK1 3 // command filter 1/(1/z+1/z+1/z)
// poles -9, -9, -9
#define K0_UP -729 // in 1 tick/(rad/s^3)
#define K1_UP -249 // in 1024/1000 tick/(rad/s^2)
#define K2_UP -27 // in 1 tick/(rad/s)
#define CK3_UP 729 // command filter (z*z*z)
#define CK2_UP 27 // command filter 1/(3/(z*z))
#define CK1_UP 3 // command filter 1/(1/z+1/z+1/z)

#define THE_LIMIT 187750
#define PHI_LIMIT 93873

#endif
//*/

uint8_t swingMode = 1;      // swing-up mode

int32_t ctrl_vect[3];       // deadbeat controller commands
int16_t g_accumulator;      // count steps to integrate v change due to gravity
int32_t TOleg;              // Liftoff leg length
int32_t TOlegVel;           // Liftoff leg velocity
int32_t TOq[3];             // Liftoff attitude
int32_t TOw[3];             // Liftoff angular velocity
int32_t TOt;                // Liftoff time
int16_t TOvz = 8000;        // Liftoff vertical velocity
int32_t TDq[3];             // Touchdown attitude
int32_t TDqCmd[3];          // Touchdown attitude command
int16_t TDvCmd[3];          // Touchdown velocity command
int16_t TDv[3];             // Touchdown velocity
int32_t TDt;                // Touchdown time

int32_t start_time;

int32_t att_correction[2];

#define VEL_BUF_LEN 10  // Buffer to find peak velocity at takeoff
uint8_t vel_ind = 0;
int16_t legBuf[VEL_BUF_LEN*2];
int32_t angBuf[VEL_BUF_LEN*6];
int32_t last_mot;

// TODO move this somewhere else?
#define STEP_MS 2

#define ATT_CORRECTION_GAIN_X 12
//#define ATT_CORRECTION_GAIN_Y 8
#define ATT_CORRECTION_GAIN_Y 10

#define TAIL_BRAKE 20
#define TAIL_REVERSE 7

int32_t originalLegSetpoint;
int32_t originalPushoffCmd;
uint8_t keepLanding = 1;


// Communications variables ---------------------------------------------------
int16_t gdata[3];                       // Rate gyro data array
int16_t xldata[3];                      // Accelerometer data array

packet_union_t uart_tx_packet_global;   // BLDC motor driver packet
extern EncObj encPos[NUM_ENC];          // Encoder angle objects

extern packet_union_t* last_bldc_packet;// BLDC motor driver packet in
extern uint8_t last_bldc_packet_is_new; // BLDC motor driver packets

// Last BLDC packet sent information: TODO, slightly hacky
volatile uint32_t t_cmd_last = 0;
volatile int32_t position_last = 0;
volatile uint32_t current_last = 0;
volatile uint8_t flags_last =0;


// Interrupt running loop at 1 kHz ============================================
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    uint8_t i;

    //mpuGetGyro(gdata); // This should be the only call to mpuGetGyro(gdata)
    //mpuGetXl(xldata); // Similarly, this should only be called once

    // Attitude
    //orientImageproc(gdataBody, gdata); // orient gyro readings to body

    // Previously in _T1Interrupt
    //mpuBeginUpdate(); // Start IMU and encoder reads
    //amsEncoderStartAsyncRead();

    if ((modeFlags>>6) == 1 || // swing-up
        mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND) { // on ground

        int32_t ml = FULL_MASS*(int32_t)leg; // in 2^24 ticks/(kg m)

        w[1] += (ml * 
            (GRAV_ACC*sin_theta/17453 // in 1000*2^10 ticks/(m/s) to 2^11/(2000*pi/180) conversion ~= 17453
            )//- 2*(int32_t)legVel*(int32_t)w[1]/32000000) // in 1000*2000*2^15/(2000*pi/180) to 2^11/(2000*pi/180) conversion = 32000
            - (tailCmd - 6*tail_vel)*TAIL_STALL // in 1000*4000*256 to 2^35/(2000*pi/180) conversion ~= 1
            // MANUAL TUNING 8 instead of 15
            ) / Iy // in 2^20 ticks/(kg m^2)
            + ((gdataBody[1]-w[1]) >> 3);

        // w[0] = ((4-W_ALPHA)*w[0] + W_ALPHA*gdataBody[0])>>2; // Low pass the gyro signal
        // w[2] = ((4-W_ALPHA)*w[2] + W_ALPHA*gdataBody[2])>>2; // Low pass the gyro signal

        w[0] += (ml*GRAV_ACC*sin_phi/17453
            - (int32_t)(foreLP+aftLP)*((int32_t)leg + 5243)/5424 // 1000*4000*2^16/0.0491 to 2^35/(2000*pi/180) conversion ~= 1/5424.0
            ) / Ix                                    // Thruster arm 0.08m ~= 5242.9 in 2^16 ticks/m
            + ((gdataBody[0]-w[0]) >> 3);
        w[2] += ((foreLP-aftLP)>>1) / IZ_CG // 1000*4000/0.0020 to 2^35/(2000*pi/180) ~= 0.4922 ~= >>1
            + ((gdataBody[2]-w[2]) >> 2);

        for (i=0; i<3; i++) {
            // 2-step moving sum for attitude integration at half frequency
            w500[i] = wLast[i] + gdataBody[i];
            wLast[i] = gdataBody[i];
        }

    } else { // Normal attitude estimation
        for (i=0; i<3; i++) {
            // Low pass the gyro signal
            w[i] = ((4-W_ALPHA)*w[i] + W_ALPHA*gdataBody[i])>>2;

            // 2-step moving sum for attitude integration at half frequency
            w500[i] = wLast[i] + gdataBody[i];
            wLast[i] = gdataBody[i];

            // Keep a circular buffer of readings for compensating mocap lag
            //qLagSum[i] += (-qLagLog[qLagInd][i] + gdataBody[i]);
            //qLagLog[qLagInd][i] = gdataBody[i];
        }
    }

    if (modeFlags & 0b1 &&
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)) {
        // Notch filter for pitch
        wyI[2] = wyI[1];
        wyI[1] = wyI[0];
        wyI[0] = w[1];
        wyO[2] = wyO[1];
        wyO[1] = wyO[0];
        #if ROBOT_NAME == SALTO_1P_DASHER
        // Period of 16 cycles, 0.1 bandwidth
        wyO[0] = (102*(int32_t)wyO[1] - 46*(int32_t)wyO[2]
            + 55*(int32_t)wyI[0] - 102*(int32_t)wyI[1] + 55*(int32_t)wyI[2])>>6;
        w[1] = wyO[0];
        #elif ROBOT_NAME == SALTO_1P_RUDOLPH
        // Period of 20 cycles, 0.1 bandwidth
        wyO[0] = (105*(int32_t)wyO[1] - 46*(int32_t)wyO[2]
            + 55*(int32_t)wyI[0] - 105*(int32_t)wyI[1] + 55*(int32_t)wyI[2])>>6;
        w[1] = wyO[0];
        #else
        #endif
    }

    //qLagInd = (qLagInd+1)%LAG_MS; // Circular buffer index

    if (t1_ticks%2) {
        // Attitude integration at 500 Hz
        eulerUpdate(q,w500,1);
    } else {
        // Estimation and control at 500 Hz
        if (modeFlags < (1<<6)) {
            kinematicUpdate();
            modeEstimation();
            jumpModes();
            if (modeFlags & 0b1 && 
                (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)
                && !gainsPD[9]){ // gainsPD[9] is used to select new or old balance
                balanceCtrl();
            } else {
                attitudeCtrl();
            }
        } else if ((modeFlags>>6) == 1) {
            swingUpCtrl();
        }
    }

    
    if (gainsPD[9]) {
        if (!(t1_ticks%2)) {
            // Old estimation using control values
            // Should NOT use the linearizing models
            q0offset = ((foreCmd+aftCmd)>>7)*STEP_MS;
            q1offset = (tailCmd>>4)*STEP_MS;
            q[0] += ((foreCmd+aftCmd)>>7)*STEP_MS;
            q[1] += (tailCmd>>4)*STEP_MS;
        }
    } else {
        // Balance offset Estimator
        if (!(t1_ticks%2)) {
            // Low-pass filters for balance offset estimation
            foreLP = ((31*foreLP)>>5) + (foreCmd>>5);
            aftLP = ((31*aftLP)>>5) + (aftCmd>>5);
            for (i=0; i<3; i++) {
                wLP[i] = ((3*wLP[i])>>2) + (w[i]>>2);
            }
        }
        if (!(t1_ticks%BOE_DEC)) {
            // Toe balance estimation update
            if (modeFlags & 0b1 && 
                (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)) {
                balanceOffsetEstimator();
            }
        }
    }

    /*
    if (ROBOT_NAME == SALTO_1P_SANTA) {
        if (mj_state == MJ_AIR) { // in the air
            tiHChangeMode(1, TIH_MODE_COAST);
        } else { // brake on the ground
            tiHChangeMode(1, TIH_MODE_BRAKE);
        }
    }
    */

    // Previously in _T1Interrupt
    //t1_ticks++;
    //if (t1_ticks == T1_MAX) t1_ticks = 0;


    // Previously in _T1Interrupt
    //if (t1_ticks%TELEM_DECIMATE == 0) {
    //    telemSaveNow();
    //}

    t5_count++;
        
    _T5IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

    interrupt_count++;

    if (interrupt_count == 1) {
        // previously in _T5Interrupt
        mpuGetGyro(gdata); // This should be the only call to mpuGetGyro(gdata)
        mpuGetXl(xldata); // Similarly, this should only be called once
        // Attitude
        orientImageproc(gdataBody, gdata); // orient gyro readings to body
        orientImageproc(xldataBody, xldata); // orient gyro readings to body
    }

    if (interrupt_count == 3) {
        if (!telemDecimateCount){
            telemSaveNow();
        }
        telemDecimateCount = (telemDecimateCount+1)%TELEM_DECIMATE;
    } else if (interrupt_count == 4) {
        mpuBeginUpdate(); // Start IMU and encoder reads
        amsEncoderStartAsyncRead();
    } else if (interrupt_count == 5) {
        interrupt_count = 0;

        if (t1_ticks == T1_MAX) t1_ticks = 0;
            t1_ticks++;
    }

    _T1IF = 0;
}


void salto1p_functions(void) {
    // Low priority processing to be run in the main loop

    int32_t tempPushoffCmd;

    // Update yaw angle approximations
    cos_psi = cosApprox(q[2]);
    sin_psi = cosApprox(q[2]-PI/2);

    // Body velocity
    vB[0] = ((int32_t)v[0]*cos_psi + (int32_t)v[1]*sin_psi)>>COS_PREC;//vi[0];
    vB[1] = (-(int32_t)v[0]*sin_psi + (int32_t)v[1]*cos_psi)>>COS_PREC;//vi[1];
    vB[2] = v[2];

    legCtrl();
    
    // Onboard trajectory
    if (modeFlags & 0b1000) {

    }

    // Onboard velocity control using flight phase attitude
    if (modeFlags & 0b10000) { // orient leg for landing (approximate hack)
        if (mj_state == MJ_AIR) {
            /*
            vCmd[0] = 0;
            vCmd[1] = 0;
            if (0 && (vB[0] > 500 || vB[0] < -500 || // DSIABLED
                vB[1] > 500 || vB[1] < -500)) {
                vCmd[2] = 5000;
                pushoffCmd = deadbeatVelCtrl(vB, vCmd, ctrl_vect);
                legSetpoint = ctrl_vect[2];
            } else {
                vCmd[2] = 4000;
                tempPushoffCmd = deadbeatVelCtrl(vB, vCmd, ctrl_vect);
                if (0 && (v[2] < -6000)) { // DISABLED
                    legSetpoint = ctrl_vect[2];
                    pushoffCmd = tempPushoffCmd;
                } else {
                    legSetpoint = originalLegSetpoint;
                    pushoffCmd = originalPushoffCmd;
                }
            }
            // MANUAL TUNING
            qCmd[1] = ctrl_vect[0]; // offset back by 0 deg (1<<13 ticks/deg), no scale fudge
            qCmd[0] = ctrl_vect[1]-8192; // offset by 1/2 deg, no scale fudge factor
            */
            //int32_t vzLand = -TOvz; // hack for flat ground
            int32_t vzLand = v[2];
            if (vzLand > -3000) {
                vzLand = -3000;
            }

            int32_t landingLeg = 5898; // leg; // leg - 1311
            int32_t Tt = sqrtApprox(landingLeg/GRAV_ACC << 4); // output in 2^10 ticks/s
            // vz*Tt 2000*2^10 to 2^16 ~= >> 5
            // vx*Tt 2000*2^10 to 2^25/(2*pi/180) ~= 469.
            // 1311 = 2cm in 2^16 ticks/m: use (int32_t)leg - 1311
            // 5898 = 9cm in 2^16 ticks/m
            qCmd[1] = -((int32_t)vB[0]*Tt*469)/((landingLeg - (vzLand>>5)*Tt) >> 6)
                -(int32_t)27853; // Motor effect offset: 1.7 degree offset pitch; 16384 ticks/deg
            qCmd[0] = ((int32_t)vB[1]*Tt*469)/((landingLeg - (vzLand>>5)*Tt) >> 6)
                ;//-(int32_t)1*(int32_t)16384; // MANUAL TUNING 1 degree offset roll
        } else if (mj_state == MJ_GND || mj_state == MJ_STAND) {
            qCmd[1] = 0;
            qCmd[0] = 0;
        }
    } else if (modeFlags & 0b100) {
        pushoffCmd = deadbeatVelCtrl(vB, vCmd, ctrl_vect); // onboard velocity control
        qCmd[1] = ctrl_vect[0];
        qCmd[0] = ctrl_vect[1];
        legSetpoint = ctrl_vect[2];
    }

    if (procFlags & 0b1) {
        takeoffEstimation();
    }
}


// Other functions ============================================================
void salto1pSetup(void) {

    // BLDC driver setup
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16); // set BLDC angle offset
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16);
    delay_ms(10);
#ifdef FULL_POWER
    send_command_packet(&uart_tx_packet_global, 0, (65536), 17); // set BLDC max PWM (65536=100%)
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, (65536), 17);
#else
    send_command_packet(&uart_tx_packet_global, 0, (3*65536/4), 17); // set BLDC max PWM (65536=100%)
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, (3*65536/4), 17);
#endif
    
    // Timer setup
    SetupTimer1();
    EnableIntT1;
    SetupTimer5(); // turn on main interrupt
    EnableIntT5;

}

void SetupTimer5(void) {
    ///// Timer 5 setup /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:8
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_SOURCE_INT;
    T5PERvalue = 5000; //1Khz
    OpenTimer5(T5CON1value, T5PERvalue);
}


// Running functions ==========================================================
void eulerUpdate(int32_t* angs, int32_t* vels, int8_t time) {
    // Update the Euler angle estimates (usually body_angle).
    //
    // INPUTS:
    // int32_t angs[3] are the Euler angles to be updated
    // int32_t vels[3] is the body-fixed angular velocities from the gyro
    // time: time in milliseconds (usually 1 to 20)

    int32_t temp_angle[3];
    int i;
    for (i=0; i<3; i++) {
        temp_angle[i] = angs[i];
    }

    sin_theta = cosApprox(temp_angle[1]-PI/2);
    cos_theta = cosApprox(temp_angle[1]);
    sin_phi = cosApprox(temp_angle[0]-PI/2);
    cos_phi = cosApprox(temp_angle[0]);

    // Prevent divide by zero
    if (cos_phi == 0) { cos_phi = 1; }

    // Update Euler angles
    temp_angle[1] += (((sin_phi*sin_theta/cos_phi)*vels[0]
            + (vels[1] << COS_PREC)
            - (cos_theta*sin_phi/cos_phi)*vels[2])*time) >> COS_PREC;
    temp_angle[0] += (cos_theta*vels[0] + sin_theta*vels[2])*time >> COS_PREC;
    temp_angle[2] += ((-sin_theta*vels[0])/cos_phi + (cos_theta*vels[2])/cos_phi)*time;

    // Wrap Euler angles around at +/-180 degrees
    for (i=0; i<3; i++) {
        if (temp_angle[i] > PI) {
            temp_angle[i] -= 2*PI;
        } else if(temp_angle[i] < -PI) {
            temp_angle[i] += 2*PI;
        }
    }

    for (i=0; i<3; i++) {
        angs[i] = temp_angle[i];
    }
}

void kinematicUpdate(void) {
    // Basic update to be done in any mode (robot internal kinematics)

    #define FOOT_ADJUST 262 // MANUAL TUNING 4mm

    // Read femur angle and lookup tables indexed by the femur angle
    femur = calibPos(1);
    uint16_t femur_index;
    if (femur > 0) {
        femur_index = femur/64; // Scale position to 8 bits
    } else{
        femur_index = 0;
    }
    if(femur_index<255 || femur_index > 0){
        crank = crank_femur_256lut[femur_index];
        foot = leg_femur_256lut[femur_index]+FOOT_ADJUST;
        ma = MA_femur_256lut[femur_index];
    } // Reject bad femur readings outside of physical range

    // Read the motor
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = ((sensor_data->position - BLDC_MOTOR_OFFSET)*motPos_to_femur_crank_units); //UNITFIX
    if (mot - last_mot > 1<<14 || last_mot - mot > 1<<14) {
        mot = last_mot; // reject bad samples
    }
    last_mot = mot;
    motw = sensor_data->velocity/70; // velocity in 2^16 and motw in 938.7 ticks/(rad/s)

    spring = mot - crank;
    if(spring < 0){spring=0;}

    sTorque = SPRING_LINEAR*spring -
        (SPRING_QUADRATIC*((spring*spring) >> 14));
    force = ((ma*sTorque) >> 13);

    force -= ((legVel>0?1:-1)*LEG_FRICTION*force/1000); // LEG_FRICTION/1000
    // sensor_data->position is 1 rad / 2^16 ticks (through a 25 to 1 gear ratio)
    // crank and spring are 4 rad / 2^16 tick, or 1 rad / 2^14 ticks
    // sTorque is 1 Nm / 2^14 ticks
    // MA is 1 N/Nm / 2^9 ticks
    // force is 1 N / (2^10 ticks)

    // Tail estimation
    int16_t tail_err = calibPos(0) - tail_pos;
    tail_err = tail_err > 2000 ? 2000 :
               tail_err < -2000 ? -2000 :
               tail_err;
    tail_pos += (tail_vel - w[1]/173) * STEP_MS + 3*(tail_err >> 2);
    tail_vel += ((22*TAIL_STALL/IY_TAIL*(tailCmd - 6*tail_vel)*STEP_MS)>>12) + (tail_err >> 5);
    // tail_pos is in [25/48*2^16 ticks/(2*pi rad)] ~= 5432.5 ticks/rad
    // tail_vel is in [25/48*2^16 ticks/(2000*pi rad/s)] ~= 5.43 ticks/(rad/s)
    // tau_stall/I_tail 2^8/2^20*1000 to 5.4325ticks/(rad/s)) ~= 22
    // free running speed ~= 110 rad/s ~= 600 ticks; 4000/600 ~= 6
    // 1/4000 ~= 1>>12

    // Old tail low-pass velocity estimation
    //tail_pos = calibPos(0);
    //tail_vel = (((128-TAIL_ALPHA)*tail_vel) >> 7) + ((TAIL_ALPHA*(tail_pos - tail_prev)/STEP_MS) >> 7);
    //tail_prev = tail_pos;

    // Thruster velocity estimation
    tbIndPrev = tbInd;
    tbInd = (tbInd+1)%3;
    uint16_t newForeVel = adcGetMotorC(); // From 0 to 1023; stall is about 512
    uint16_t newaftVel = adcGetMotorD();
    if (newForeVel > 973 || newForeVel < 50) {
        foreBuff[tbInd] = foreBuff[tbIndPrev];
    } else {
        foreBuff[tbInd] = newForeVel;
    }
    if (newaftVel > 973 || newaftVel < 50) {
        aftBuff[tbInd] = aftBuff[tbIndPrev];
    } else {
        aftBuff[tbInd] = newaftVel;
    }
    foreVel = -((med3(foreBuff)-512) << 3); // Median filter
    aftVel = -((med3(aftBuff)-512) << 3);

}

void jumpModes(void) {
    // Estimation based on mode
    // DESCRIPTION TODO

    // Hacky e-stop if the robot falls over
    if ((mj_state != MJ_STOPPED) 
            && ((q[0] > PI/3 || q[0] < -PI/3)
            || ((q[1] > PI/3 || q[1] < -PI/3) && !(modeFlags & 0b100000)) )) {
        mj_state = MJ_STOP;
    }

    switch(mj_state) {
        case MJ_START:
            mj_state = MJ_LAUNCH;
            tiHChangeMode(1, TIH_MODE_COAST);
            break;

        case MJ_STOP:
            mj_state = MJ_STOPPED;
            send_command_packet(&uart_tx_packet_global, 0, 0, 0);
            break;

        case MJ_AIR:
            // Ground contact transition out of air to ground
            if (t1_ticks - transition_time > 300
                    && (spring > 1500)) {
                if (modeFlags & 0b10000
                        && legSetpoint == originalLegSetpoint) {
                    mj_state = MJ_STAND;
                    keepLanding = 1;
                } else {
                    mj_state = MJ_GND;
                    keepLanding = 1;//0;
                }
                transition_time = t1_ticks;
            }
            break;

        case MJ_GND:
            // Liftoff transition from ground to air
            if (t1_ticks - transition_time > 30
                    && (spring < 500 || femur > FULL_EXTENSION)
                    && crank > 8192) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            }
            break;

        case MJ_LAUNCH:
            // Liftoff transition from launch to air
            if ((spring < 500 || femur > FULL_EXTENSION)
                    && crank > 8192) {
                if (legVel < 2000 && modeFlags & 0b10000) {
                    // In this case it's not jumping, just standing up
                    mj_state = MJ_STAND;
                } else {
                    // usual transition to aerial phase
                    mj_state = MJ_AIR;
                    transition_time = t1_ticks;
                }
            }
            break;

        case MJ_STAND:
            // No longer standing mode: jump!
            if (!(modeFlags & 0b10000)) {
                mj_state = MJ_LAUNCH;
            }
            int32_t the_eff = q[1] + (int32_t)w[1]*100; // ~= qy + wy*Tt = qy + wy*0.1
            int32_t phi_eff = q[0] + (int32_t)w[0]*100; // ~= qx + qx*Tt
            if (0 && (!keepLanding || // DISABLED
                    the_eff > THE_LIMIT || the_eff < -THE_LIMIT || // 0.2*PI
                    phi_eff > PHI_LIMIT || phi_eff < -PHI_LIMIT)) {
                keepLanding = 0;
                if (t1_ticks - transition_time > 30
                        && (spring < 500 || femur > FULL_EXTENSION)
                        && crank > 8192) {
                    mj_state = MJ_AIR;
                    transition_time = t1_ticks;
                }
            }
            if (t1_ticks - transition_time > 30
                    && (spring < 500 || femur > FULL_EXTENSION)
                    && crank > 8192
                    && legVel > 3000) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            }
            break;

        case MJ_STOPPED:
            break;

        default:
            mj_state = MJ_IDLE;
            break;
    }
}

void modeEstimation(void) {
    // Description TODO
    uint8_t j;

    if (mj_state == MJ_GND || mj_state == MJ_LAUNCH || mj_state == MJ_STAND) {
        if (last_state == MJ_AIR) { // The robot just touched down
            flightStanceTrans();
        }
        stanceUpdate();
    } else if (mj_state == MJ_AIR) { // Flight phase estimation
        if (last_state == MJ_GND || last_state == MJ_LAUNCH || mj_state == MJ_STAND) { // robot thinks it just took off
            stanceFlightTrans();
        }
        flightUpdate();
    } else {
        leg = foot >> 2;
        legVel = 0;
    }
    last_state = mj_state;

    if (v[2] > 20000) { v[2] = 20000;}
    if (v[2] < -20000) { v[2] = -20000;}

    // Save the recent velocities to check takeoff
    legBuf[vel_ind*2] = leg;
    legBuf[vel_ind*2+1] = legVel;
    for (j=0; j<3; j++){
        angBuf[vel_ind*6+j] = q[j];
        angBuf[vel_ind*6+3+j] = w[j];
    }
    vel_ind = (vel_ind+1)%VEL_BUF_LEN;

    // Update attitude with output of takeoff estimator
    if ((modeFlags & 0b100) && (modeFlags & 0b10) && (procFlags & 0b10)) {
        q[0] -= att_correction[1];
        q[1] -= att_correction[0];
        procFlags &= ~0b10;
    }
}

void stanceUpdate(void) {
    // Description TODO
    // Update the onboard leg velocity estiamtes
    // Update leg and leg velocity estimates
    int32_t legErr;
    legErr = (foot>>2) - leg;
    if (legErr > 1000) {legErr = 1000;}
    if (legErr < -1000) {legErr = -1000;}
    leg += legVel/31 * STEP_MS + 3*(legErr >> 2);
    // leg is in 1 m / 2^16 ticks
    // conversion from legVel*STEP_MS to leg is 1000000*2/2^16 or about 31
    legVel += (((force/FULL_MASS
        - (GRAV_ACC*cos_theta*cos_phi>>(2*COS_PREC)) // gravity
        //+ ((((int32_t)w[1]*(int32_t)w[1]>>10) + ((int32_t)w[0]*(int32_t)w[0]>>10))*(int32_t)leg >> 24) // Centrifugal
        ) * STEP_MS) >> 1) + (legErr << 1);
    // legVel is in 1 m/s / (1000*2 ticks)
    // acceleration is in m/s^2 / (2^2 ticks)

    #if ROBOT_NAME == SALTO_1P_DASHER
    if (leg < 5964 && legVel < 0) { // 9cm+1mm
        legVel = 0;
    }
    #elif ROBOT_NAME == SALTO_1P_RUDOLPH
    if (leg < 7000 && legVel < 0) { // 10.7cm
        legVel = 0;
    }
    #endif

    v[0] = 0; // zero velocities; not really necessary
    v[1] = 0;
    v[2] = legVel; // also zeroing vertical velocity
}

void flightUpdate(void) {
    // Description TODO
    uint8_t j;
    leg = foot >> 2;

    g_accumulator+=STEP_MS;
    if (!(procFlags & 0b1)) {
        v[2] -= (GRAV_ACC*g_accumulator) >> 1; // gravitational acceleration
        g_accumulator = 0;
    }

    // Integrate robot position
    for (j=0; j<3; j++){
        p[j] += v[j]*STEP_MS/20;
    }
}

void flightStanceTrans(void) {
    // Description TODO
    uint8_t j;

    legVel = v[2]; // simplification for vertical hopping

    // Impact
    // 2000*2^24/2^20 ticks/(m/s) to 938.7 ticks/(rad/s) ~= >> 5
    // ignoring cos(phi) coefficient in z direction; it should be small
    w[0] += (-((int32_t)v[2]*(int32_t)leg*sin_phi >> (COS_PREC+5))
        -((int32_t)vB[1]*(int32_t)leg*cos_theta >> (COS_PREC+5)))*FULL_MASS/Ix;
    w[1] += (-((int32_t)v[2]*(int32_t)leg*sin_theta >> (COS_PREC+5))
        +((int32_t)vB[0]*(int32_t)leg*cos_theta >> (COS_PREC+5)))*FULL_MASS/Iy;
    for (j=0; j<3; j++) {
        TDvCmd[j] = vCmd[j]; // Save the desired velocities
        TDq[j] = q[j]; // Save the touchdown body angles
        TDv[j] = v[j]; // Save the touchdown body velocities
        TDqCmd[j] = qCmd[j];
        TDt = t1_ticks;
    }
}

void stanceFlightTrans(void) {
    // Description TODO
    int16_t i, j;
    int32_t cntr;
    // save the pose and velocity states for calculating takeoff velocities
    // first, check if takeoff actually happened earlier
    cntr = vel_ind;
    TOlegVel = legVel;
    legVel = 0;
    //*
    for (i=0; i<VEL_BUF_LEN; i++) {
        if (legBuf[cntr*2+1] > TOlegVel) {
            TOlegVel = legBuf[cntr*2+1];
            g_accumulator = i*STEP_MS; // set the accumulator to the number of steps elapsed

            TOleg = legBuf[cntr*2];
            for (j=0; j<3; j++) {
                TOq[j] = angBuf[cntr*6+j];
                TOw[j] = angBuf[cntr*6+3+j];
                TOt = t1_ticks - i;
            }
        }
        cntr -= 1;
        if (cntr < 0) {cntr = VEL_BUF_LEN - 1;}
    }
    //*/
    /*
    TOleg = leg;
    for (j=0; j<3; j++) {
        TOq[j] = q[j];
        TOw[j] = w[j];
    }
    */

    // MANUAL TUNING
    TOlegVel = TOlegVel + 400; // takeoff boost of 0.2 m/s

    //velocity[2] = -velocity[2]; // velocity estimate until real calculation is done
    procFlags |= 0b1; // tell the main loop to calculate the takeoff velocities
}

void takeoffEstimation(void) {
    //Calculate estimated velocities on takeoff

    //*
    int32_t TOcos_theta = cosApprox(TOq[1]);
    int32_t TOsin_theta = cosApprox(TOq[1]-PI/2);
    int32_t TOcos_phi = cosApprox(TOq[0]);
    int32_t TOsin_phi = cosApprox(TOq[0]-PI/2);
    int32_t TOcos_psi = cosApprox(TOq[2]);
    int32_t TOsin_psi = cosApprox(TOq[2]-PI/2);

    // Hack for smoothing TOw
    /*
#if ROBOT_NAME == SALTO_1P_DASHER
    TOw[0] = (TOq[0] - TDq[0])/(TOt-TDt); // 16384 per degree
    TOw[1] = (TOq[1] - TDq[1])/(TOt-TDt);
#else
    TOw[0] = (TOq[0] - TDq[0])/(TOt-TDt);
    TOw[1] = (TOq[1] - TDq[1])/(TOt-TDt);
#endif
    */

    // Compensate for CG offset
    // MANUAL TUNING
#if ROBOT_NAME == SALTO_1P_DASHER
    TOw[1] += 20*TOlegVel/213; // in (centi rad/s)/(m/s). (2^15/2000*180/pi)/2000 = 0.4694: 100/0.4694 = 213
    TOw[0] += 20*TOlegVel/213;
#elif ROBOT_NAME == SALTO_1P_RUDOLPH
    TOw[1] += 00*TOlegVel/213;
    TOw[0] += 00*TOlegVel/213;
#else
    TOw[1] += 0.2*0.469*TOlegVel/; // in (rad/s)/(m/s). (2^15/2000*180/pi)/2000 = 0.4694
    TOw[0] += 0.2*0.469*TOlegVel/;
#endif

    // Body velocity rotation matrix
    int32_t vxw = 14418*(int32_t)TOw[1]/30760; // locking TOleg at 0.22m
    int32_t vyw = -14418*(int32_t)TOw[0]/30760;
    //int32_t vxw = (int32_t)TOleg*(int32_t)TOw[1]/30760;
    //int32_t vyw = -(int32_t)TOleg*(int32_t)TOw[0]/30760;
    // native units (body_vel_LP*leg: 2000/2^15 (deg/s)/tick * 1/2^16 m/ticks * pi/180 rad/deg
    // final units (legVel): 1/2000 (m/s)/tick
    // unit conversion: 1/30760.437 tick/tick

    // MANUAL TUNING
    //vyw = 3*vyw/4;
    // Lateral oscillations complete 1.5 periods during stance and cause a gyro
    // measurement overshoot of somewhere around 50% at takeoff time.
    //vxw = 7*vxw/8;
    
    int32_t velX = (((((TOcos_psi*TOcos_theta) >> COS_PREC)
            - ((((TOsin_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        - ((((TOcos_phi*TOsin_psi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOcos_psi*TOsin_theta) >> COS_PREC) 
            + ((((TOcos_theta*TOsin_psi) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
   int32_t velY = (((((TOcos_theta*TOsin_psi) >> COS_PREC)
            + ((((TOcos_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        + ((((TOcos_psi*TOcos_phi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOsin_psi*TOsin_theta) >> COS_PREC)
            - ((((TOcos_psi*TOcos_theta) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
    int32_t velZ = - ((((TOcos_phi*TOsin_theta) >> COS_PREC)*vxw) >> COS_PREC)
        + ((TOsin_phi*vyw) >> COS_PREC)
        + ((((TOcos_theta*TOcos_phi) >> COS_PREC)*TOlegVel) >> COS_PREC);
    //Z1X2Y3 https://en.wikipedia.org/wiki/Euler_angles

    //velY = 18*velY/20;

    v[0] = velX;
    v[1] = velY;
    v[2] = velZ;
    TOvz = velZ;

    //*
    int i;
    for (i=1; i<3; i++) {
        TDq[i] -= TDqCmd[i]; // calculate angle error
    }

    // Deadbeat-based correction
    //int32_t predicted_angles[3];
    //deadbeat(TDvelocity, velocity, predicted_angles);
    //att_correction[0] = (TDbody_angle[2] - predicted_angles[0])>>2;
    //att_correction[1] = (TDbody_angle[1] - predicted_angles[1])>>2;

    // Velocity-based correction
    int32_t velocity_x = (v[0]*TOcos_psi + v[1]*TOsin_psi)>>COS_PREC;
    int32_t velocity_y = (-v[0]*TOsin_psi + v[1]*TOcos_psi)>>COS_PREC;

    att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - TDvCmd[0]) - (TDq[2]>>1));
    att_correction[1] = ATT_CORRECTION_GAIN_Y*(velocity_y - TDvCmd[1]) + (TDq[1]>>1);

    //att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - stance_vel_des[0]) * 188) /
    //    ((int32_t)(-velocity[2] + TDvelocity[2]) >> 6);
    //att_correction[1] = (ATT_CORRECTION_GAIN_Y*(velocity_y - stance_vel_des[1]) * 188) /
    //    ((int32_t)(-velocity[2] + TDvelocity[2]) >> 6); // (3*2000 >> 6)*2 is about 188

    att_correction[0] = att_correction[0] * 5000/velZ;
    att_correction[1] = att_correction[1] * 5000/velZ;

    att_correction[0] = att_correction[0] > 15000 ? 15000 :
                        att_correction[0] < -15000 ? -15000 :
                        att_correction[0];
    att_correction[1] = att_correction[1] > 15000 ? 15000 :
                        att_correction[1] < -15000 ? -15000 :
                        att_correction[1];

    procFlags |= 0b10;
    //*/
    procFlags &= ~0b1;
}

void legCtrl(void) {
    if (mj_state == MJ_GND) {
        send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_GND, 2);
    } else if (mj_state == MJ_STAND) {
        if (!keepLanding) {
            // Unrecoverable
            if (v[2] < 6000) {
                #ifdef FULL_POWER
                send_command_packet(&uart_tx_packet_global, 90*65536+BLDC_CMD_OFFSET, GAINS_GND, 2);
                #else
                send_command_packet(&uart_tx_packet_global, 80*65536+BLDC_CMD_OFFSET, GAINS_GND, 2);
                #endif
            } else {
                send_command_packet(&uart_tx_packet_global, 50*65536+BLDC_CMD_OFFSET, GAINS_GND, 2);
            }
        } else {
            //send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_STAND, 2);
            send_command_packet(&uart_tx_packet_global, forceSetpoint(rdes, rddes, rdddes, k1des, k2des), GAINS_ENERGY, 2);
        }
    } else if (mj_state == MJ_LAUNCH) {
        if (modeFlags & 0b10000) {
            //send_command_packet(&uart_tx_packet_global, launchVelocity(), GAINS_GND, 2);
            //*
            if (crank > 4096) {
                // slow down the jump
                send_command_packet(&uart_tx_packet_global, legSetpoint+BLDC_CMD_OFFSET, GAINS_GND, 2);//GAINS_STAND, 2);
            } else {
                // usual jump
                send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_GND, 2);
            }
            //*/
        } else {
            send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_GND, 2);
        }
    } else if (mj_state == MJ_AIR) {
        if (modeFlags & 0b10000) {
            // MANUAL TUNING how much to retract leg for landing
            int32_t vzLand = v[2];
            if (vzLand > -3000) {
                vzLand = -3000;
            }
            int32_t legRet = -(vzLand<<16)/200 + ((int32_t)25<<16);
            if (legRet < 65536*(int32_t)45) {
                legRet = 65536*(int32_t)45;
            }
            send_command_packet(&uart_tx_packet_global, legRet+BLDC_CMD_OFFSET, GAINS_AIR, 2);
        } else {
            send_command_packet(&uart_tx_packet_global, legSetpoint+BLDC_CMD_OFFSET, GAINS_AIR, 2);
        }
    } else if (mj_state == MJ_STOPPED) {
        sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
        if (sensor_data->current != 0) {
            send_command_packet(&uart_tx_packet_global, 0, 0, 0);
        }
    }
}

void balanceCtrl(void) {
    // TODO description
    uint8_t i;

    int32_t balanceLeg = leg;

    // Constant parameters
    I_cg = (FULL_MASS*(balanceLeg*balanceLeg >> 8)) >> 12; // 2^20 ticks/(kg m^2)
    Iy = IY_CG + I_cg; // 2^20 ticks/(kg m^2)
    mgc = (balanceLeg*FULL_MASS*GRAV_ACC) >> 6; // in 2^20 ticks/(N m)

    // Controller gain scheduling
    int32_t k0, k1, k2, ck1, ck2, ck3;
    if (leg < 7864) {// 0.12 m extension: around the middle of the soft spot
        k0 = K0;
        k1 = K1;
        k2 = K2;
        ck1 = CK1;
        ck2 = CK2;
        ck3 = CK3;
    } else {
        k0 = K0_UP;
        k1 = K1_UP;
        k2 = K2_UP;
        ck1 = CK1_UP;
        ck2 = CK2_UP;
        ck3 = CK3_UP;
    }

    // Variables that vary with leg length (0.08 to 0.3m):
    //int32_t TcSquared = (Iy<<8)/(mgc>>8); // in 2^16 ticks/s^2
    //int32_t Gw = -(IY_TAIL*(int32_t)1024)/Iy; // in 21.10 fixed point (unitless)
    //int32_t H11 = Iy >> 5; // (TcSquared*mgc)>>21; // in 2^15 ticks/(kg m^2)
    //int32_t H22 = IY_TAIL; // (-H11*Gw)>>5; // in 2^20 ticks/(kg m^2)
    /*
    For leg fully crouched (0.08m):
        mgc = 90521;
        TcSquared = 655;
        Gw = 57;
        H11 = 28;
        H22 = -58;
    For leg fully extended (0.25m):
        mgc = 282880;
        TcSqured = 1700;
        Gw = 7;
        H11 = 229;
        H22 = -50;
    */

    #define U_OFF -3 // MANUAL TUNING momentum bias

    if (t1_ticks - u_time > 100) {
        u = 0 + U_OFF;
        ud = 0;
        udd = 0;
        uddd = 0;
        uCmd = u + (ud/ck1) + (udd/ck2) + (uddd/ck3);
    } else {
        // uCmd is in 2^15/(2000*pi/180)~=938.7 ticks/(rad s)
        uCmd = u + U_OFF + (ud/ck1) + (udd/ck2) + (uddd/ck3);
        // u is in 2^15/(2000*pi/180)~=938.7 ticks/(rad s)
        // ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
        // udd is in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s)
        // uddd is in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s^2)
    }

    // M is in 2^15/(2000*pi/180)~=938.7 ticks/s
    int32_t M = (Iy*w[1] + IY_TAIL*(w[1] + 173*tail_vel))/mgc;// + IY_MOT*motw)/mgc;
    // For w[1]=2^15, l=0.25m: M ~= 1700*2^15/2^16 ~= 2^26/2^16 = 2^10

    int32_t Md = q[1];
    int32_t Mdd = w[1];
    // 1000*2000/2^16 conversion ~= 30.52 ~= >>5

    // Mddd is in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s^2)
    int32_t Mddd = (k2*Mdd) + ((k1*Md)>>10) + (k0*(M-uCmd));
    // For w[1]=2^15, q[1]=pi:
    // term1 = 30*2^15 ~= 2^20
    // term2 = 300*2^14*180/1000 ~= 2^20
    // term3 ~= 1000*2^10 ~= 2^20

    // qdd1H22 is in 2^30/(2000*PI/180)~=30760000 ticks/(N m)
    //int32_t qdd1H22 = ((IY_TAIL*Mddd)>>5);
    // For w[1]=2^15, q[1]=pi: qdd1H22 ~= 50*2^21/2^5 = 2^22

    // qdd2H22 is in 2^30/(2000*pi/180)~=30760000 ticks/(N m)
    int32_t qdd2H22 = - (Iy*((Mddd>>5)))
        + ((mgc*sin_theta*29)>>COS_PREC) // Gravitational compensation
        + 2*((FULL_MASS*(int32_t)leg>>10)*(int32_t)legVel>>10)*(int32_t)w[1]; // Coriolis
    // mgc is in 2^20 ticks/(N m): conversion is 2^10/(2000*pi/180) ~= 29.3354
    // For q[1]=pi/2, l=0.25m: term1 = 1013*2^8*2^7 = 2^25, term2 ~= 229*2^21 = 2^29

    // tau2 is in 2^30/(2000*pi/180)~=30760000 ticks/(N m)
    int32_t tau2 = qdd2H22; // + qdd1H22;

    int32_t tailTorque;
    if (gainsPD[6] || gainsPD[7]) {
        tailTorque = tau2/(30*TAIL_STALL); // conversion from N m to PWM 4000 ticks
            // (2^30/(2000*pi/180)/2^8)/4000 ~= 30
        tailTorque = tailTorque > MAX_TAIL ? MAX_TAIL :
                     tailTorque < -MAX_TAIL ? -MAX_TAIL :
                     tailTorque;
    } else {
        tailTorque = 0;
    }

    if (!keepLanding) {
        // Unrecoverable: just zero the angular momentum
        tailTorque = -TAIL_BRAKE*(tail_vel + TAIL_REVERSE*(w[1]>>8));
    }

    // Attitude PD controllers
    int32_t qErr[3];
    for (i=0; i<3; i++) {
        qErr[i] = q[i] - qCmd[i];
        while (qErr[i] > PI) {
            qErr[i] -= 2*PI;
        }
        while (qErr[i] < -PI) {
            qErr[i] += 2*PI;
        }
    }
    int32_t yawPD = ((gainsPD[0] * qErr[2])>>12) + ((gainsPD[1] * w[2])>>4);
    int32_t rolPD = ((gainsPD[3] * qErr[0])>>12) + ((gainsPD[4] * w[0])>>4);

    if (gainsPD[3] || gainsPD[4]) {
        // Add steady thrust to hold up stance balance
        rolPD += (mgc * sin_phi) >> (COS_PREC-1+3);
        // mgc is in 2^20 ticks/(N m)
        // thrusters produce 0.049 N * 0.08 m torque each @ 4000 PWM
        //      thrusters are 2*1019368.0 PWM ticks/(N m)
        // conversion: 1.94 ~= 1<<1
    }

    attitudeActuators(rolPD, tailTorque, yawPD);
}

void balanceOffsetEstimator(void) {
// Adjust the roll and pitch estimates for balancing on toe

    //*
    // Balance Offset Observer from Roy Featherstone's group
    //#define IY_CG 126  // moment of inertia about CG y axis (1.2E-4 N m^2)
    //#define IX_CG 98   // moment of inertia about CG x axis (9.3E-5 N m^2)
    //#define IY_TAIL 47 // tail moment of inertia (4.5E-5 N m^2)

    //int32_t I_cg = (FULL_MASS*(((int32_t)leg)*((int32_t)leg) >> 8)) >> 12;
    //int32_t Iy = IY_CG + I_cg;
    int32_t My = Iy*wLP[1] + IY_TAIL*(173*tail_vel + wLP[1]) * 1;
    Ix = IX_CG + I_cg;
    int32_t Mx = Ix*wLP[0];
    // conversion from tail_vel to wLP is 173
    // wLP is 2^15 ticks/(2000 deg/s) = 938.7 ticks/(rad/s)
    // FULL_MASS is 2^8 ticks/kg
    // leg is 2^16 ticks/m
    // Ix and Iy are 2^20 ticks/(kg m^2)
    // Mx and My are in 938.7*2^20 ticks/(N m s)
    // My's final multiply is a fudge factor for the tail

    //int32_t mgc = ((int32_t)leg*FULL_MASS*GRAV_ACC) >> 6;
    // FULL_MASS is 2^8 ticks/kg
    // GRAV_ACC is 2^2 ticks/(m/s^2)
    // leg is 2^16 ticks/m
    // mgc is in 2^20 ticks/(N m)

    if (gainsPD[3] || gainsPD[4]) {
        q0offset = 147*((Mx-MxBuff[Mind])*1/(BOE_DEC*N_MBUFF) - tauXSum/N_MBUFF)/(mgc>>7);
    } else {
        q0offset = 0;
    }
    if (gainsPD[6] || gainsPD[7]) {
        if (q[1] < PI/6 && q[1] > -PI/6) {
            q1offset = 73*((My-MyBuff[Mind])*1/(BOE_DEC*N_MBUFF) - tauYSum/N_MBUFF)/(mgc>>7);
        } else if (q[1] > 5*PI/6 || q[1] < -5*PI/6) {
            q1offset = -73*((My-MyBuff[Mind])*1/(BOE_DEC*N_MBUFF) - tauYSum/N_MBUFF)/(mgc>>7);;
        } else {
            q1offset = 0;
        }
    } else {
        q1offset = 0;
    }

    MxBuff[Mind] = Mx;
    MyBuff[Mind] = My;

    // Saturate correction rate to 30 deg/s
    q0offset = q0offset > 3*169*BOE_DEC ? 3*169*BOE_DEC :
               q0offset < -3*169*BOE_DEC ? -3*169*BOE_DEC :
               q0offset;

    q1offset = q1offset > 6*169*BOE_DEC ? 6*169*BOE_DEC :
               q1offset < -6*169*BOE_DEC ? -6*169*BOE_DEC :
               q1offset;

    q[0] += q0offset;
    q[1] += q1offset;

    // M_ and M_Buff are 938.7*2^20 ticks/(N m s)
    // BOE_DEC is 1000 tick/s
    // tau_ are 2^20 ticks/(N m)
    //      conversion ~ 1
    // mgc is in 2^20 ticks/(N m)
    // q are in 938734.0 ticks/rad
    //      a gain of 1/2 per step makes this 469367; 1/100 is 73.3*128

    tauY = (mgc*sin_theta) >> COS_PREC;
    tauX = ((mgc*sin_phi) >> COS_PREC) - 1*((int32_t)foreLP+(int32_t)aftLP) * 2;//3/2;
    // mgc is in 2^20 ticks/(N m)
    // tauX and tauY are 2^20 ticks/(N m)
    // thrusters produce 0.049 N * 0.08 m torque each (conversion ~= 1 ~= 66/64)
    // tauX final multiply is a fudge factor for the thrusters

    tauXSum += tauX-tauXBuff[Mind];
    tauYSum += tauY-tauYBuff[Mind];

    tauXBuff[Mind] = tauX;
    tauYBuff[Mind] = tauY;

    Mind++;
    if (Mind == N_MBUFF){
        Mind = 0;
    }
    //*/
}

int32_t deadbeatVelCtrl(int16_t* vi, int16_t* vo, int32_t* ctrl) {
    // TODO description
    long ox = vo[0];
    long oy = vo[1];
    long oz = vo[2]-6600;

    //long ix = ((long)vi[0]*cos_psi + (long)vi[1]*sin_psi)>>COS_PREC;//vi[0];
    //long iy = (-(long)vi[0]*sin_psi + (long)vi[1]*cos_psi)>>COS_PREC;//vi[1];
    long ix = vi[0];
    long iy = vi[1];
    long iz = (vi[2] > -4000 ? -4000 : vi[2]) + 6600;
    //long iz = -TOvz + 6600; // TODO: this is a hack that works for flat ground

    long ixix = (ix*ix)>>11; // >> 11 is approximately divide by 2000
    long iyiy = (iy*iy)>>11;
    long iziz = (iz*iz)>>11;
    long oxox = (ox*ox)>>11;
    long oyoy = (oy*oy)>>11;
    long ozoz = (oz*oz)>>11;

    long ixiz = (iz*ix)>>11;
    long iyiz = (iz*iy)>>11;
    long oxiz = (iz*ox)>>11;
    long oyiz = (iz*oy)>>11;
    long ixoz = (oz*ox)>>11;
    long iyoz = (oz*iy)>>11;
    long oxoz = (oz*ox)>>11;
    long oyoz = (oz*oy)>>11;

#ifndef FULL_POWER
    long ixixix = (ixix*ix)>>11;
    long iyiyiy = (iyiy*iy)>>11;
    long iziziz = (iziz*iz)>>11;
    long oxoxox = (oxox*ox)>>11;
    long oyoyoy = (oyoy*oy)>>11;
    long ozozoz = (ozoz*oz)>>11;

    long ixixiz = (ixix*iz)>>11;
    long iyiyiz = (iyiy*iz)>>11;
    long oxoxiz = (oxox*oz)>>11;
    long oyoyiz = (oyoy*oz)>>11;
    long ixixoz = (ixix*oz)>>11;
    long iyiyoz = (iyiy*oz)>>11;
    long oxoxoz = (oxox*oz)>>11;
    long oyoyoz = (oyoy*oz)>>11;
#endif

#ifdef FULL_POWER
    // 100% gains from runGridMotor18a
    long pit_ctrl = -91*ix +36*ox //-84*ix +32*ox // supposed to be 91, 36
        -17*ixiz +10*oxiz -2*ixoz -19*oxoz;
        // Scaled by 469 approx = PI/(3.14159*2000)

    long rol_ctrl = -(-91*iy +36*oy //-83*iy +34*oy // supposed to be 91, 36
        -17*iyiz +10*oyiz -2*iyoz -19*oyoz);

    long leg_ctrl = (77*65536 -472*iz -688*oz
        +62*(ixix+iyiy) -20*iziz -119*(oxox+oyoy) -123*ozoz);
        // Scaled by 65536/2000

    /*
    // 100% gains from runGridMotor18
    long pit_ctrl = -95*ix +40*ox // supposed to be 95, 40
        -17*ixiz +9*oxiz -4*ixoz -20*oxoz
        +1*ixixix -2*oxoxox;
        // Scaled by 469 approx = PI/(3.14159*2000)

    long rol_ctrl = -(-95*iy +40*oy // supposed to be 95, 40
        -17*iyiz +9*oyiz -4*iyoz -20*oyoz
        +1*iyiyiy -2*oyoyoy);

    long leg_ctrl = (77*65536 -542*iz -622*oz
        +58*(ixix+iyiy) -33*iziz -117*(oxox+oyoy) -142*ozoz
        +38*iziziz -82*ozozoz
        +18*(ixixiz+iyiyiz) +10*(oxoxiz+oyoyiz) -9*(ixixoz+iyiyoz) +1*(oxoxoz+oyoyoz));
        // Scaled by 65536/2000
    */
#else
    // Gains with shell: 108 g robot from runGridMotor17
    long pit_ctrl = -100*ix +31*ox // supposed to be 100 31
        -17*ixiz +11*oxiz -3*ixoz -19*oxoz
        +1*ixixix -2*oxoxox;
        // Scaled by 469 approx = PI/(3.14159*2000)

    long rol_ctrl = -(-100*iy +31*oy
        -17*iyiz +11*oyiz -3*iyoz -19*oyoz
        +1*iyiyiy -2*oyoyoy);

    long leg_ctrl = (67*65536 -437*iz -328*oz
        +28*(ixix+iyiy) -58*iziz -121*(oxox+oyoy) +153*ozoz
        +37*iziziz +32*ozozoz
        +14*(ixixiz+iyiyiz) -25*(oxoxiz+oyoyiz) -55*(ixixoz+iyiyoz) -31*(oxoxoz+oyoyoz));
        // Scaled by 65536/2000
#endif

    pit_ctrl = pit_ctrl > PI/6 ? PI/6 :
              pit_ctrl < -PI/6 ? -PI/6 :
              pit_ctrl;
    rol_ctrl = rol_ctrl > PI/6 ? PI/6 :
              rol_ctrl < -PI/6 ? -PI/6 :
              rol_ctrl;
    leg_ctrl = leg_ctrl > (80*65536) ? (80*65536) :
              leg_ctrl < (55*65536) ? (55*65536) :
              leg_ctrl;

    ctrl[0] = pit_ctrl;
    ctrl[1] = rol_ctrl;
    ctrl[2] = leg_ctrl;

#ifdef FULL_POWER
    return (90*65536); // ext_ctrl for 100% gains
#else
    return (80*65536); // ext_ctrl
#endif
}

void attitudeCtrl(void) {
    // TODO description
    uint8_t i;

    // Attitude PD controllers
    int32_t qErr[3];
    for (i=0; i<3; i++) {
        qErr[i] = q[i] - qCmd[i];
        while (qErr[i] > PI) {
            qErr[i] -= 2*PI;
        }
        while (qErr[i] < -PI) {
            qErr[i] += 2*PI;
        }
    }
    int32_t yawPD = ((gainsPD[0] * qErr[2])>>12) + ((gainsPD[1] * w[2])>>4);
    int32_t rolPD = ((gainsPD[3] * qErr[0])>>12) + ((gainsPD[4] * w[0])>>4);
    int32_t pitPD = ((gainsPD[6] * qErr[1])>>12) + ((gainsPD[7] * w[1])>>4);

    if (modeFlags & 0b1 &&
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)) {
        // Add steady thrust to hold up stance balance
        rolPD += (((((int32_t)leg*FULL_MASS*GRAV_ACC) >> (5+3)) * sin_phi) >> COS_PREC);
        // FULL_MASS is 2^8 ticks/kg
        // GRAV_ACC is 2^2 ticks/(m/s^2)
        // leg is 2^16 ticks/m
        //      m*g*leg is in 2^26 ticks/(N m)
        // thrusters produce 0.049 N * 0.08 m torque each @ 4000 PWM
        //      thrusters are 2*1019368.0 PWM ticks/(N m)
        // conversion: 1/33 ~= 1>>5
    }

    attitudeActuators(rolPD, pitPD, yawPD);
}

void swingUpCtrl(void) {
    // Variables:
    // q[3]: x,y,z (acutally Z,X,Y) Euler angles in 2^15/(2*pi/180) ticks/rad
    // w[3]: x,y,z angular velocity in in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s)
    // leg: CG height in 2^16 tick/m
    // legVel: CG radial velocity in 2000 ticks/(m/s)
    //
    // Useful constants:
    // PI: 3.14159 in the same units as q
    // GRAV_ACC: 9.81 m/s^2 in 2^2 ticks/(m/s^2)
    // FULL_MASS: 108 g in 2^8 ticks/kg
    //
    // Useful functions and variables to set:
    // cmdLegLen(r): returns motor angle command for a desired leg length
    // send_command_packet(...): commands a leg angle (only sent at about 100Hz)
    // tailCmd: tail torque: Dasher ~4000/0.07, Rudolph ~4000/0.04 ticks/(Nm)
    //      NOTE: must call tailMotor = tailLinearization(&tailCmd); and
    //          tiHSetDC(0+1, tailMotor);
    // balanceCtrl(): balance using the tail
    // modeFlags: when bit 1 is set (modeFlags|=0b1), use balance offset estimation.
    //      Stop using it when bit 1 is unset (modeFlags&=~0b1)


    if (mj_state != MJ_STOP && mj_state != MJ_STOPPED && mj_state != MJ_IDLE) {
        // Running
        int32_t r, wSquared;

        uint32_t energy_gains = (5*655*65536)+(20*7); // leg control gains 2 6
        uint32_t balance_gains = (1*655*65536)+(5*7); // leg control gains

        mj_state = MJ_STAND;

        #define GRAV_SQUARED 24636 // 96.2 in 2^8 ticks/(m^2/s^4)
        #define LEG_ADJUST 0//656

        // State estimation
        kinematicUpdate();
        modeEstimation();
        leg = leg+LEG_ADJUST;

        if (0 && swingMode) {
            // Balance on toe
            balanceCtrl();

            send_command_packet(&uart_tx_packet_global, 0, balance_gains, 2);

            if (q[1] > PI/8 || q[1] < -PI/8) {
                swingMode = 0; // Switch to use energy controller
                modeFlags &= ~0b1; // don't use balance offset estimator
            }
        } else {
            // Leg pumping to add energy
            if ((w[1] > 0 && q[1] < 0) || (w[1] < 0 && q[1] > 0)){
                // Retract leg
                wSquared = ((int32_t)w[1]*(int32_t)w[1])/55076;
                // w[1] is in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s)
                // wSquared is in 2^4 ticks/(rad/s)^2; conversion ~= 1/55076
                if (q[1] < (-PI/2) || q[1] > (PI/2)) {
                    // Hanging down
                    // r is in 
                    r = ((sqrtApprox(
                        2*((int32_t)leg*(int32_t)leg>>20)*(wSquared*wSquared>>20)
                        + ((GRAV_SQUARED*cos_theta*cos_theta)>>(2*COS_PREC+8))
                        - 2*GRAV_ACC*((int32_t)leg*wSquared>>22) ) << 20)
                        + (GRAV_ACC*cos_theta>>(COS_PREC+2)) )
                        / wSquared - LEG_ADJUST;
                    // sqrt argument is in 2^0 ticks/(m^2/s^4)
                    // leg is 2^16 ticks/m; max is 2^14
                    // w[1] is in 2^15/(2000*pi/180)~=938.7 ticks/(rad/s); max is 2^15
                    // wSquared is in 2^4 ticks/(rad/s)^2; max is 2^15
                } else {
                    // Up
                    r = (sqrtApprox(2*(int32_t)leg*(
                        ((int32_t)leg*wSquared>>18)
                        - (GRAV_ACC)
                        + (GRAV_ACC*cos_theta>>(COS_PREC))) >> 8 ) << 15)
                        / (w[1]/59) - LEG_ADJUST;
                    // sqrt argument is in 2^10 ticks/(m^2/s^2)
                    // leg is 2^16 ticks/m; max is 2^14
                    // w[1] is in 2^15/(2000*pi/180) ticks/(rad/s); max is 2^14
                    //      conversion to 2^4 ~= 1/58.7
                    // wSquared is in 2^4 ticks/(rad/s)^2; max is 2^15
                }

            } else {
                // Extend leg
                r = 13107;
            }

            r = r < 5898 ? 5898 :
                r > 13107? 13107 :
                r;

            //send_command_packet(&uart_tx_packet_global, cmdLegLen(r), energy_gains, 2);
            //send_command_packet(&uart_tx_packet_global, cmdLegLen(11141), energy_gains, 2);
            send_command_packet(&uart_tx_packet_global, forceSetpoint(leg, 600, 0, -100, -21)+BLDC_MOTOR_OFFSET, energy_gains, 2);

            if ((q[1] > 7*PI/8 || q[1] < -7*PI/8) && w[1] < 2000 && w[1] > -2000) {
                // Tail pumping
                if (w[1] > -500 && q[1] > 0) {
                    tailCmd = -2000;
                } else {
                    tailCmd = 2000;
                }
            } else {
                // Tail braking
                tailCmd = -TAIL_BRAKE*(tail_vel + TAIL_REVERSE*(w[1] > 0 ? -50 : 50));//(w[1]>>8));
                tailMotor = tailLinearization(&tailCmd); // Linearizing the actuator response
            }
            //tiHSetDC(0+1, tailMotor); // send tail command to H-bridge

            if ((q[1] < 49152 && q[1] > -196608 && w[1] < 3000 && w[1] > -1000) ||
                (q[1] < 196608 && q[1] > -49152 && w[1] < 1000 && w[1] > -3000)) {
                //swingMode = 1; // Switch to use balance controller
                //modeFlags |= 0b1; // use balance offset estimator
            }
        }

    } else if (mj_state == MJ_STOP) {
        // Stopping
        send_command_packet(&uart_tx_packet_global, 0, 0, 0);
        tiHSetDC(0+1, 0);
        tiHSetDC(2+1, 0);
        tiHSetDC(3+1, 0);
        mj_state = MJ_STOPPED;
    }
}

void attitudeActuators(int32_t roll, int32_t pitch, int32_t yaw){
    // Attitude mixing, saturation, linearization, and PWM output
    int i;

    
    if (modeFlags & 0b1 && 
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)) {
        // Notch filters
        /*
        // Period of 20 cycles, 0.02 bandwidth
        roll = roll > 8191 ? 8191 :
               roll < -8191 ? -8191 :
               roll;
        rolI[2] = rolI[1];
        rolI[1] = rolI[0];
        rolI[0] = roll;
        rolO[2] = rolO[1];
        rolO[1] = rolO[0];
        roll = (118*(int32_t)rolO[1] - 60*(int32_t)rolO[2]
            + 62*(int32_t)rolI[0] - 118*(int32_t)rolI[1] + 62*(int32_t)rolI[2])>>6;
        rolO[0] = roll;
        //*/
        /*
        yaw = yaw > 8191 ? 8191 :
              yaw < -8191 ? -8191 :
              yaw;
        // Period of 20 cycles, 0.1 bandwidth
        yawI[2] = yawI[1];
        yawI[1] = yawI[0];
        yawI[0] = yaw;
        yawO[2] = yawO[1];
        yawO[1] = yawO[0];
        yaw = (105*(int32_t)yawO[1] - 46*(int32_t)yawO[2]
            + 55*(int32_t)yawI[0] - 105*(int32_t)yawI[1] + 55*(int32_t)yawI[2])>>6;
        yawO[0] = yaw;
        */

        //*
        pitch = pitch > 8191 ? 8191 :
                pitch < -8191 ? -8191 :
                pitch;
        pitI[2] = pitI[1];
        pitI[1] = pitI[0];
        pitI[0] = pitch;
        pitO[2] = pitO[1];
        pitO[1] = pitO[0];
        #if ROBOT_NAME == SALTO_1P_DASHER
        // Period of 8 cycles, 0.125 bandwidth
        pitO[0] = (75*(int32_t)pitO[1] - 43*(int32_t)pitO[2]
            + 53*(int32_t)pitI[0] - 75*(int32_t)pitI[1] + 53*(int32_t)pitI[2])>>6;
        pitch = ((8-TAIL_CMD_ALPHA)*pitch + TAIL_CMD_ALPHA*pitO[0]) >> 3; // low pass filter
        #elif ROBOT_NAME == SALTO_1P_RUDOLPH
        // Period of 10 cycles, 0.125 bandwidth
        pitO[0] = (86*(int32_t)pitO[1] - 43*(int32_t)pitO[2]
            + 53*(int32_t)pitI[0] - 86*(int32_t)pitI[1] + 53*(int32_t)pitI[2])>>6;
        pitch = ((8-TAIL_CMD_ALPHA)*pitch + TAIL_CMD_ALPHA*pitO[0]) >> 3; // low pass filter
        #else
        #endif
        //*/
    }

    // Attitude actuator mixing
    foreCmd = (cos_theta*roll - cos_theta*yaw + sin_theta*roll + sin_theta*yaw)>>COS_PREC;//(foreCmd>>1) + ((roll - yaw)>>1);
    aftCmd = (cos_theta*roll + cos_theta*yaw - sin_theta*roll + sin_theta*yaw)>>COS_PREC;//(aftCmd>>1) + ((roll + yaw)>>1);
    tailCmd = pitch;

    if (modeFlags & 0b1 && 
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND)) {
        // Balance on toe tail velocity feedback
        tailCmd += gainsPD[9]*tail_vel;
    } else if (mj_state != MJ_AIR) { // Tail braking on the ground
        tailCmd = -TAIL_BRAKE*(tail_vel + TAIL_REVERSE*(w[1]>>8));
    }

    // Linearizing the actuator response
    foreThruster = thrusterLinearization(&foreCmd, foreVel);
    aftThruster = thrusterLinearization(&aftCmd, aftVel);
    tailMotor = tailLinearization(&tailCmd);

    // Set motor PWM commands
    if (mj_state != MJ_STOP && mj_state != MJ_STOPPED && mj_state != MJ_IDLE) {
        if (mj_state == MJ_AIR &&
            ((tail_vel < -150 &&
            tailCmd > -5*tail_vel) ||
            (tail_vel > 150 &&
            tailCmd < -5*tail_vel))) {
            tiHChangeMode(1, TIH_MODE_BRAKE);
            tiHSetDC(1, tailCmd > 0 ? 4000 : -4000);
        } else {
            tiHChangeMode(1, TIH_MODE_COAST);
            tiHSetDC(0+1, tailMotor);
        }
        tiHSetDC(2+1, foreThruster);
        tiHSetDC(3+1, aftThruster);
    } else {
        for (i=0; i<4; i++) {
            tiHSetDC(i+1, 0);
            //tiHChangeMode(1, TIH_MODE_BRAKE); // brake the tail
        }
    }
}

int32_t thrusterLinearization(int32_t* thruster, int16_t velocity){
    // Linearize the thruster force by inverting the normalized Force vs. Volt
    // calibratin curve
    // INPUT: thruster should be in PWM ticks between -4000 and 4000
    // OUTPUT: thrusterOut in PWM ticks between -4000 and 4000
    // Updates thruster if thrusterOut saturates

    int16_t thrusterOut;

    // Inverting the force calibration curve
    if (*thruster <= -2800) {
        *thruster = -2800;
        thrusterOut = -4000;
    } else if (*thruster <= -400) {
        thrusterOut = 6*(*thruster+2800)/7 - 4000;
    } else if (*thruster <= 400) {
        thrusterOut = 3* *thruster;
    } else if (*thruster <= 4000) {
        thrusterOut = 7*(*thruster-400)/9 + 1200;
    } else {
        thrusterOut = 4000;
        *thruster = 4000;
    }

    //*
    // Accelerating thrusters (inertia)
    if (velocity < 2000 && thrusterOut > 0 && 2*thrusterOut > velocity) {
        thrusterOut += ((2*thrusterOut > 2000 ? 2000 : 2*thrusterOut) - velocity)>>1;//>>2;
    }
    if (velocity > -2000 && thrusterOut < 0 && 2*thrusterOut < velocity) {
        thrusterOut += ((2*thrusterOut < -2000 ? -2000 : 2*thrusterOut) - velocity)>>1;//>>2;
    }
    //*/

    // Control saturation
    thrusterOut = thrusterOut > MAX_THROT ? MAX_THROT :
                  thrusterOut < -MAX_THROT ? -MAX_THROT :
                  thrusterOut;

    return thrusterOut;
}

int32_t tailLinearization(int32_t* tail){
    // Linearize the tail motor torque with a simple motor and friction model
    // INPUT: tail should be in PWM ticks between -4000 and 4000
    // OUTPUT: tailOut in PWM ticks between -4000 and 4000
    // Updates tail if tailOut saturates

    // input tail should be in about 4000/(0.06) ~= 60000 ticks/(N m)
    // Free running speed ~ 120 rad/s (at tail, not the encoder) at 3800 PWM
    // This should be about 6, but that makes it unstable

    // Actuator saturation
    *tail = *tail > MAX_TAIL ? MAX_TAIL :
            *tail < -MAX_TAIL ? -MAX_TAIL :
            *tail;

    int16_t tailOut = *tail;

    if (!gainsPD[6] && !gainsPD[7]) {
        *tail = 0;
        return 0;
    }

#if ROBOT_NAME == SALTO_1P_DASHER
    if (modeFlags & 0b1 &&
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND) ) {
        tailOut += 5*tail_vel; // more aggressive linearization for toe balancing
    } else {
        tailOut += 4*tail_vel;
    }

    // Friction (stiction) compensation
    if (tail_vel < -20) {
        tailOut -= 20;
    } else if (tail_vel > 20) {
        tailOut += 20;
    } else {
        tailOut += 1*tail_vel;
    }
#elif ROBOT_NAME == SALTO_1P_RUDOLPH
    if (modeFlags & 0b1 &&
        (mj_state == MJ_LAUNCH || mj_state == MJ_GND || mj_state == MJ_STAND) ) {
        tailOut += 3*tail_vel; // more aggressive linearization for toe balancing
    } else {
        tailOut += 2*tail_vel;
    }

    // Friction (stiction) compensation
    if (tail_vel < -20) {
        tailOut -= 60;
    } else if (tail_vel > 20) {
        tailOut += 60;
    } else {
        tailOut += 3*tail_vel;
    }
#endif

    tailOut = tailOut > MAX_TAIL ? MAX_TAIL :
              tailOut < -MAX_TAIL ? -MAX_TAIL :
              tailOut;

    *tail -= (*tail-tailOut);

    return tailOut;
}

int32_t launchVelocity(void) {
    if (rddes == 0) {
        return 65526*(int32_t)30; // pre-wind a little bit
    }
    int32_t absLegVel = legVel > 0 ? legVel : -legVel;
    // 676 from 1/(0.65*sqrt(1/0.108*0.3759274976416161)) conversion to motor units
    int32_t mot_cmd = 676*(int32_t)(rddes - absLegVel) + (25*(int32_t)crank<<2)
        + (66*sqrtApprox(13107-leg) << 8);
        // sqrt(2*0.108*9.81*l/0.3) = th
        // 25*sqrt(2*0.108*9.81/0.3) ~= 66 rad/m^0.5 ticks/ticks
    // conversion 25*0.8246 to 2^16/2000 (rad/(m/s))
    if (crank < 4096 && mot_cmd < 65535*(int32_t)60) {
        mot_cmd = 65536*(int32_t)60;
    }

    if (mot_cmd < 65536*(int32_t)30) {
        mot_cmd = 65536*(int32_t)30;
    } else if (mot_cmd > 65536*(int32_t)100) {
        mot_cmd = 65536*(int32_t)100;
    }
    return mot_cmd;
}


// Communications functions ===================================================
void setGains(int16_t* gains) {
    uint8_t i;
    for (i=0; i<10; i++) {
        gainsPD[i] = gains[i];
    }
}

void setAttitudeSetpoint(long yaw, long roll, long pitch){
    qCmd[2] = yaw;
    qCmd[0] = roll;
    qCmd[1] = pitch;
}

void setLegSetpoint(long length){
    legSetpoint = length << 8;
    originalLegSetpoint = legSetpoint;
}

void setPushoffCmd(long cmd){
    pushoffCmd = cmd << 8;
    originalPushoffCmd = pushoffCmd;
}

void setMotorPos(uint32_t gain, long pos){
    mj_state = MJ_STAND;
    GAINS_STAND = gain;
    pushoffCmd = pos;
    modeFlags |= 0b10000;
}

void setBodyAngle(long* qSet) {
    // INPUT: long[3] {yaw, roll, pitch}
    q[0] = qSet[1];
    q[1] = qSet[2];
    q[2] = qSet[0];
}

void adjustBodyAngle(long* qAdjust){
    // INPUT: long[3] {yaw adjustment, roll adjustment, pitch adjustment}
    q[0] += qAdjust[1];
    q[1] += qAdjust[2];
    q[2] += qAdjust[0];
}

void updateBodyAngle(long* qUpdate){
    if (modeFlags && 0b11) {
        // Don't accept offboard angle updates if onboard balance offset 
        // estimation or toe balance control is active
        return;
    }

    //eulerUpdate(q,qLagSum,1); // attempt to counter comms lag

    q[0] = 3*(q[0] >> 2) + (qUpdate[1] >> 2);
    q[1] = 3*(q[1] >> 2) + (qUpdate[2] >> 2);
    q[2] = 3*(q[2] >> 2) + (qUpdate[0] >> 2);
}

void accZeroAtt(){
    // get an APPROXIMATE Euler angle relative to the g vector from the accelerometer radings

    int16_t xldata[3];  // accelerometer data
    mpuGetXl(xldata);
    int32_t body_acc[3]; // body-frame accelerometer readings
    orientImageproc(body_acc, xldata);

    q[0] = PI*body_acc[1]/(3.14159*body_acc[2]) + PI*0.01; // roll
    q[1] = -PI*body_acc[0]/(3.14159*body_acc[2]) + PI*0.01; // pitch
}

void calibGyroBias(){
    DisableIntT1;
    mpuRunCalib(0,10); //re-offset gyro, assumes stationary
    // TODO enableing and disabling interrupts is a little shady
    EnableIntT1;
}

void expStart(uint8_t startSignal) {
    mj_state = MJ_START;
    start_time = t1_ticks;
    u_time = t1_ticks;
}

void expStop(uint8_t stopSignal) {
    mj_state = MJ_STOP;
}

void setOnboardMode(uint8_t flags, uint8_t mode) {
    modeFlags = flags;
}

void setVelocitySetpoint(int16_t* newCmd, int32_t newYaw) {
    int i;

    // Onboard trajectory generation variables
    int32_t tCycle = 0;
    int32_t vTraj[3] = {0,0,5800}; // 2.9m/s takeoff vel

    if (!(modeFlags & 0b1000)) {
        // don't jump too low when coming in fast
        newCmd[2] = newCmd[2] < vB[0]<<1 ? vB[0]<<1 : 
                    newCmd[2] < -vB[0]<<1 ? -vB[0]<<1 : 
                    newCmd[2] < vB[1]<<1 ? vB[1]<<1 : 
                    newCmd[2] < -vB[1]<<1 ? -vB[1]<<1 :
                    newCmd[2];

        newCmd[2] = newCmd[2] > 8000 ? 8000 : // max height
                    newCmd[2] < 4000 ? 4000 : // min height
                    newCmd[2];

        // don't go too fast when jumping low
        newCmd[0] = newCmd[0] > newCmd[2]-2000 ? newCmd[2]-2000 : 
                    newCmd[0] < -(newCmd[2]-2000) ? -(newCmd[2]-2000) :
                    newCmd[0];
        newCmd[1] = newCmd[1] > (newCmd[2]-2000)>>1 ? (newCmd[2]-2000)>>1 :
                    newCmd[1] < -(newCmd[2]-2000)>>1 ? -(newCmd[2]-2000)>>1 :
                    newCmd[1];

        // limit horizontal velocity change
        newCmd[0] = newCmd[0] > vB[0]+4000 ? vB[0]+4000 : 
                    newCmd[0] < vB[0]-4000 ? vB[0]-4000 :
                    newCmd[0];
        newCmd[1] = newCmd[1] > vB[1]+2000 ? vB[1]+2000 :
                    newCmd[1] < vB[1]-2000 ? vB[1]-2000 :
                    newCmd[1];
        //*
        // Usual velocity command
        for (i=0; i<3; i++){
            vCmd[i] = newCmd[i];
        }
        qCmd[2] = newYaw;

        while (qCmd[2] > PI) {
            qCmd[2] -= 2*PI;
        }
        while (qCmd[2] < -PI) {
            qCmd[2] += 2*PI;
        }
        //*/

        // Command positions
        /*
        pCmd[0] += newCmd[0]*1; // *50/25; // only integrate at 1/2 speed
        pCmd[1] += newCmd[1]*1; // *50/25;
        vCmd[2] = newCmd[2];
        qCmd[2] = 0;

        vCmd[0] = (pCmd[0]-p[0])/(50*2) + newCmd[0];
        vCmd[1] = (pCmd[1]-p[1])/(50*2) + newCmd[1];
        */
    } else {
        // Trajectory
        tCycle = (t1_ticks - start_time) % 20000;
        if (tCycle < 4000) {
            vTraj[0] = 0;
            vTraj[1] = 0;
            pCmd[0] = 0;
            pCmd[1] = 0;
        }
        if (tCycle < 8000) {
            // 1 m/s forwards for 2 seconds
            vTraj[0] = 1500;
            vTraj[1] = 0;
            pCmd[0] = (tCycle-4000)*vTraj[0]/20;
            pCmd[1] = 0;
        } else if (tCycle < 12000) {
            // 0.5 m/s left for 2 seconds
            vTraj[0] = 0;
            vTraj[1] = 750;
            pCmd[0] = 300000;
            pCmd[1] = (tCycle-8000)*vTraj[1]/20;
        } else if (tCycle < 16000) {
            // 1 m/s backwards for 2 seconds
            vTraj[0] = -1500;
            vTraj[1] = 0;
            pCmd[0] = 300000 + (tCycle-12000)*vTraj[0]/20;
            pCmd[1] = 150000;
        } else {
            // 0.5 m/s right for 2 seconds
            vTraj[0] = 0;
            vTraj[1] = -750;
            pCmd[0] = 0;
            pCmd[1] = 150000 + (tCycle-16000)*vTraj[1]/20;
        }

        vCmd[0] = (pCmd[0]-p[0])/100 + vTraj[0];
        vCmd[1] = (pCmd[1]-p[1])/100 + vTraj[1];
        vCmd[2] = vTraj[2];
    }

}

void setTilt(int16_t u_in, int16_t ud_in, int16_t udd_in, int16_t uddd_in) {
    u = u_in;
    ud = ud_in;
    udd = udd_in;
    uddd = (int32_t)uddd_in << 1;
    u_time = t1_ticks;
}

void setLeg(int16_t rdes_in, int16_t rddes_in, int16_t rdddes_in, int16_t k1des_in, int16_t k2des_in) {
    rdes = rdes_in;
    rddes = rddes_in;
    rdddes = rdddes_in;
    k1des = k1des_in;
    k2des = k2des_in;

    if (mj_state == MJ_GND || mj_state == MJ_LAUNCH) {
        mj_state = MJ_STAND;
    }
}

void setMocapVel(int16_t* new_vel, int32_t yaw) {
    int i;
    //q[2] = (15*q[2]>>4) + (yaw>>4);
    if (mj_state != MJ_AIR) {
        return;
    }
    for (i=0; i<3; i++) {
        v[i] = (3*v[i]>>2) + (new_vel[i]>>2);
    }
}

void setLaunch(int32_t rol_in, int16_t rddes_in) {
    if (mj_state == MJ_AIR || mj_state == MJ_STOPPED) {
        return;
    }
    mj_state = MJ_LAUNCH;
    rddes = rddes_in;
    qCmd[0] = rol_in;
}

void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags){
    // Send UART command packets to the MBed KL25Z Brushless DC Motor Driver.
    // 
    // INPUTS:
    // uart_tx_packet: packet structure (reused on each sending)
    // position: int32_t whose meaning is set by flags
    // current: uint32_t whose meaning is set by flags
    // flags: control packet type such as position command or calibration
    //  command. "position" and "current" inputs are interpreted differently
    //  depending on the mode selected by the value of flags.

    // TODO: remove reliance on t1_ticks to measure passage of time.  Perhaps system clock?

    if (((sclockGetTime() - t_cmd_last) < UART_PERIOD) ||
        (position == position_last && current == current_last && flags == 2)) {
        return; // skip sending if last command was too recent or was identical
    }

    // Create dummy UART TX packet
    uart_tx_packet->packet.header.start = PKT_START_CHAR;
    uart_tx_packet->packet.header.type = PKT_TYPE_COMMAND;
    uart_tx_packet->packet.header.length = sizeof(header_t) + sizeof(command_data_t) + 1;
    command_data_t* command_data = (command_data_t*)&(uart_tx_packet->packet.data_crc);
    
    // Settable things
    uart_tx_packet->packet.header.flags = flags;
    command_data->position_setpoint = position;
    command_data->current_setpoint = current;
    uartSend(uart_tx_packet->packet.header.length, (unsigned char*)&(uart_tx_packet->raw));
 
    // Save values for next time
    position_last = position;
    current_last = current;
    flags_last = flags;   
    t_cmd_last = sclockGetTime();
}

// Utility functions ==========================================================

void orientImageproc(int32_t* v_b, int16_t* v_ip) {
    // Rotate vectors from the ImageProc IMU frame into the robot body frame.
    // ImageProc frame aligned to board: 0:x,right, 1:y,forwards, 2:z,up
    // Robot body frame: 0:z,up,yaw, 1:x,forwards,roll, 2:y,left,pitch
    // 
    // INPUTS:
    // v_ip: int16_t [3] (from MPU IMU) to be transformed from the IP frame
    // v_b: int32_t [3] output of the transformation in the body frame

#if ROBOT_NAME == SALTO_1P_RUDOLPH
    // -55 degrees about roll
    // x axis right, y axis forwards, z axis up from ImageProc
    v_b[2] = (147*((int32_t)v_ip[2]) - 210*((int32_t)v_ip[0]))>>8; //yaw
    v_b[0] = -v_ip[1]; // roll
    v_b[1] = (147*((int32_t)v_ip[0]) + 210*((int32_t)v_ip[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_DASHER
    // -50 degrees about x, follwed by 180 degrees about body z
    //v_b[2] = (165*((int32_t)v_ip[2]) - 196*((int32_t)v_ip[0]))>>8; //yaw
    //v_b[0] = -v_ip[1]; // roll
    //v_b[1] = (165*((int32_t)v_ip[0]) + 196*((int32_t)v_ip[2]))>>8; //pitch
    // -54 degrees about x, follwed by 180 degrees about body z
    v_b[2] = (139*((int32_t)v_ip[2]) - 215*((int32_t)v_ip[0]))>>8; //yaw
    v_b[0] = -v_ip[1]; // roll
    v_b[1] = (139*((int32_t)v_ip[0]) + 215*((int32_t)v_ip[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_SANTA
    // -45 degrees about pitch
    v_b[2] = (((int32_t)(v_ip[2] + v_ip[1]))*181)>>8; //yaw
    v_b[0] = (((int32_t)(v_ip[1] - v_ip[2]))*181)>>8; //roll
    v_b[1] = -v_ip[0]; //pitch
#endif
}

int32_t calibPos(uint8_t idx){
    // Return the angles of encoders including the offsets and absolute angle
    // unwrapping.  Uses the encPos objects: reads the current position, 
    // number of wraps, and offsets then adds them together.
    //
    // OUTPUT: absolute position (with unwrapping)
    // INTPUTS:
    // idx: index of the encPos object (0: tail, 1: femur)

    int32_t temp;
    if (idx == 0) { // tail
        temp = (int32_t)(encPos[0].pos << 2); // pos 14 bits 0x0 -> 0x3fff
        return temp + (encPos[0].oticks << 16);
    }
    else if (idx == 1) { // femur
        temp = -(((int32_t)encPos[1].pos - (int32_t)encPos[1].offset) << 2);
        return temp - (encPos[1].oticks << 16);
    }
    else {
        return -1;
    }
}

int32_t forceSetpoint(int16_t r_des, int16_t rd_des, int16_t rdd_des, int16_t k1, int16_t k2){
    // r_des in 2^16 ticks per m
    // rd_des in 2000 ticks per m/s
    // rdd_des in 2^10 ticks per m/s^2
    // k1 in 2^0 ticks per unit
    // k2 in 2^0 ticks per unit
    #define k 5 // 2^4 ticks/(Nm/rad) ~= 0.3
    
    int32_t legError = (leg - r_des) >> 6;
    int32_t velError = (legVel - rd_des) >> 1; // ~= 2^10 ticks/(m/s)
    int32_t accel = ((int32_t)k1)*legError + ((int32_t)k2)*velError;

    int32_t divider = ((int32_t)ma>>9)*k >> 2; // in 2^2 ticks/(Nm/rad)
    divider = divider == 0 ? 1 : divider;
    int32_t F_des = FULL_MASS*(((int32_t)rdd_des + accel) >>2); //2^16 ticks per N
    if (F_des < 0) {
        F_des = 0; // Try to keep the force positive so the foot doesn't come up
    }
    int32_t motorAngle =  (F_des/divider + (int32_t)crank) * 25;
    returnable = motorAngle << 2;

    if (leg <  7209 && returnable > 60*65535) { // Don't push too far out of singularity
        returnable = 60*65535;
    }
    returnable = returnable < 0*65536 ? 0*65536: returnable > 100*65536 ? 100*65536: returnable;
    return returnable;
}

int32_t cmdLegLen(int16_t r) {
    // Find crank angle for desired leg length
    //
    // OUTPUT: motor angle for leg (2^14 ticks/rad)
    // INPUT: leg: desired length in 2^16 ticks/m

    #define CMD_LEG_LEN_THRSH 100 // a little under a half millimeter
    #define CMD_LEG_LEN_ITERS 4

    uint8_t i;

    int32_t targetLeg = (int32_t)r << 2;

    // Guess index in femur LUT from desired leg
    int32_t guess = (11*((targetLeg>>4) - (int32_t)(leg_femur_256lut[0]>>4)))>>7;
    guess = guess < 0 ? 0 :
            guess > 255 ? 255 :
            guess;

    // Iteratively refine guess of femur LUT index
    int16_t step = 1<<(CMD_LEG_LEN_ITERS-1);
    for (i=0; i<CMD_LEG_LEN_ITERS; i++) {
        if (leg_femur_256lut[guess] < targetLeg - CMD_LEG_LEN_THRSH) {
            guess += step;
        } else if (leg_femur_256lut[guess] > targetLeg + CMD_LEG_LEN_THRSH) {
            guess -= step;
        } else {
            break;
        }
        guess = guess < 0 ? 0 :
            guess > 255 ? 255 :
            guess;
        step >>= 1;
    }

    // Look up the crank LUT for the final index and return the motor angle
    return 100*(int32_t)crank_femur_256lut[guess];
    // this 100 is a hardcoded 1/motPos_to_femur_crank_units (which is 0.01)

}

int32_t cosApprox(int32_t x) {
    // Cosine approximation
    //
    // OUTPUT: cosine(x) scaled to a value between -2^COS_PREC and 2^COS_PREC
    // INPUTS:
    // x: angle scaled according to the value of #define constant PI

    /*
    // Lookup Table
    if (x < 0) { x = -x; } // cosine is even
    x = (x+PI) % (PI<<1) - PI;
    if (x < 0) { x = -x; } // cosine is even
    if (x > (PI>>1)) { // quadrants 2 and 3
        x = (PI - x)/5760;
        x = x > 255 ? 255 :
            x < 0 ? 0 :
            x;
        return -(long)lut_cos[x];
    } else {
        x = x/5760; // quadrants 1 and 4
        x = x > 255 ? 255 :
            x < 0 ? 0 :
            x;
        return (long)lut_cos[x];
    }
    */

    //*
    // Bhaskara I's method:
    // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
    uint16_t xSq; // Intermediary value x squared and bit shifted down
    int16_t out; // Variable for return value

    if (x < 0) { x = -x; } // take absolute value since cosine is even
    x = (x+PI) % (PI<<1) - PI;
    if (x < 0) { x = -x; }

    if (x > (PI>>1)) { // quadrants 2 and 3
        x = (x - PI) >> 14;
        xSq = x*x;
        out = -(int16_t)((PI_SQUARE - (xSq<<2))/((PI_SQUARE + xSq)>>COS_PREC));
    } else { // quadrants 1 and 4
        x = x >> 14; 
        xSq = x*x;
        out = (PI_SQUARE - (xSq<<2))/((PI_SQUARE + xSq)>>COS_PREC);
    }
    return (int16_t) out;
    //*/
}

int32_t sqrtApprox(int32_t n) {
// Approximation to square root from (June 2019):
// https://en.wikipedia.org/wiki/Integer_square_root
// https://en.wikipedia.org/wiki/Methods_of_computing_square_roots
// accepts values up to 2^16 only
    if (n > (65536)) {
        return 256;
    } else if(n < 0) {
        return 0;
    }

    uint16_t res = 0;
    uint16_t bit = 1 << 14; // The second-to-top bit is set: 1 << 30 for 32 bits

    // "bit" starts at the highest power of four <= the argument.
    while (bit > n) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        }
        else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

uint16_t med3(uint16_t* arr) {
// Find median of three elements for filtering

    if (arr[0] > arr[1]) {
        if (arr[0] > arr[2]) {
            if (arr[1] > arr[2]) {
                return arr[1];
            } else {
                return arr[2];
            }
        } else {
            return arr[0];
        }
    } else {
        if (arr[0] > arr[2]) {
            return arr[0];
        } else {
            if (arr[1] > arr[2]) {
                return arr[2];
            } else {
                return arr[1];
            }
        }
    }
}

