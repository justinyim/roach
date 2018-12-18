
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
int32_t gainsPD[9];      // PD controller gains (yaw, rol, pit) (P, D, other)

#define TAIL_ALPHA 25 // Low pass tail velocity out of 128
#define W_ALPHA 64  // Low pass body angular velocity out of 256

#define P_AIR ((3*65536)/100) // leg proportional gain in the air (duty cycle/rad * 65536)
#define D_AIR ((0*65536)/1000) // leg derivative gain in the air (duty cycle/[rad/s] * 65536)
#define P_GND ((5*65536)/10) // leg proportional gain on the ground
#define D_GND ((1*65536)/1000)
#define P_STAND ((4*65536)/100) // leg proportional gain for standing
#define D_STAND ((3*65536)/10000)

uint32_t GAINS_AIR = (P_AIR<<16)+D_AIR;
uint32_t GAINS_GND = (P_GND<<16)+D_GND;
uint32_t GAINS_STAND = (P_STAND<<16)+D_STAND;

#define MAXTHROT 3800   // Maximum thruster & tail PWM

// Communication and telemetry constants
#define UART_PERIOD 10

#define TELEM_DECIMATE 1 
int32_t telemDecimateCount = 0;
#define T1_MAX 0xffffff

#define LAG_MS 20 // Vicon lag in milliseconds

// Global Variables -----------------------------------------------------------
// Miscellaneous important variables
uint16_t procFlags = 0;     // Which functions to process
    // 1: Run takeoff processes immediately after takeoff
uint8_t modeFlags = 0;     // Running modes
    // 1: stance balance control enabled (1) or default aerial balance (0)
    // 2: balance offset estimation enabled (1) or disabled (0)
    // 4: use onboard velocity control (1) or accept offboard attitude cmd (0)
    // 8: use onboard trajectory (1) or accept offboard horz. velocity cmd (0)
uint32_t t1_ticks = 0;      // Time (1 tick per ms)
uint8_t interrupt_count = 0;// How many processing cycles have passed

// Continuous dynamics state variables
int32_t q[3];               // ZXY Body Euler angles (x, y, z) [2*PI ticks/rev]
int32_t w[3];               // Body ang vel (x, y, z) [2^15 ticks/(2000 deg/s)]
int32_t p[3];               // Robot position (x, y, z) [100,000 ticks/m]
int16_t v[3];               // Robot body vel (x, y, z) [1000 ticks/(m/s)]

int32_t tail_pos = 0;       // Tail angle [2^16 ticks/(2*pi rad)]
int32_t tail_prev = 0;      // Previous tail angle for velocity estimation
int32_t tail_vel = 0;       // Tail vel [(2^16 ticks)/(2*pi*1000 rad/s)]

// Control output
int32_t foreThruster;       // Control output
int32_t aftThruster;        // Control output
int32_t tailMotor;          // Control output

// Commands
int32_t qCmd[3];            // Attitude setpoint
int32_t legSetpoint = 0;    // Aerial leg motor angle command
int32_t pushoffCmd = 0;     // Ground leg motor angle command

int16_t vCmd[3] = {0,0,5000};// Velocity command for takeoff [2000 ticks/(m/s)]

// Discrete mode variables
uint8_t mj_state = MJ_IDLE; // Jump mode
uint8_t last_state = MJ_IDLE;// Jump mode at last estimation step
int32_t transition_time = 0;// Time of last mode transition

// Estimation State and Intermediate Variables
//int32_t q0[3];              // Ang offset (rol, pit, yaw) [PI ticks/(pi rad)]

int32_t qLagLog[LAG_MS][3]; // Attitude circular buffer for lag compensation
int32_t qLagSum[3];         // Attitude circular buffer sum for lag comp.
uint8_t qLagInd;            // Attitude circular buffer index

int32_t gdataBody[3];       // Raw gyro data in the body frame
int16_t vB[3];              // CG vel in world aligned to the body-fixed frame

int32_t mot;                // Motor angle [2^14 ticks/rad at the gear]
int32_t last_mot;           // Last motor angle for rejecting bad samples
int32_t femur;              // Femur angle [2^16 ticks/rot]
int32_t crank;              // Crank angle [2^14 ticks/rad]
int32_t foot;               // Foot distance [2^14 ticks/m]
int32_t ma;                 // Mech. adv. [2^9 ticks/(N/Nm)]

int32_t spring;             // Spring deflection [2^14 ticks/rad]
int32_t sTorque;            // Spring torque [2^14 ticks/(Nm)]
int32_t force;              // Foot force [2^10 ticks/N]
int16_t leg;                // Stance leg length [2^16 ticks/m]
int16_t legVel;             // Stance leg velocity [2000 ticks/(m/s)]

int32_t sin_theta = 0;      // pitch angle in COS_PREC bits
int32_t cos_theta = 1<<COS_PREC;
int32_t sin_phi = 0;        // roll angle
int32_t cos_phi = 1<<COS_PREC;
int32_t sin_psi = 0;        // yaw angle
int32_t cos_psi = 1<<COS_PREC;

int32_t ctrl_vect[3];       // deadbeat controller commands
uint16_t g_accumulator;     // count steps to integrate v change due to gravity
int16_t TOleg;
int16_t TOlegVel;
long TOq[3];
long TOw[3];
long TDq[3];
long TDqCmd[3];
int16_t TDvCmd[3];
int16_t TDv[3];

int32_t att_correction[2];

#define VEL_BUF_LEN 10  // Buffer to find peak velocity at takeoff
unsigned char vel_ind = 0;
int16_t legBuf[VEL_BUF_LEN*2];
long angBuf[VEL_BUF_LEN*6];
int32_t last_mot;

// TODO move this somewhere else?
#define STEP_MS 1

#define ATT_CORRECTION_GAIN_X 12
#define ATT_CORRECTION_GAIN_Y 8



// Communications variables ---------------------------------------------------
int16_t gdata[3];                       // Rate gyro data array
int16_t xldata[3];                      // Accelerometer data array

packet_union_t uart_tx_packet_global;   // BLDC motor driver packet
extern EncObj encPos[NUM_ENC];          // Encoder angle objects

extern packet_union_t* last_bldc_packet;// BLDC motor driver packet in
extern uint8_t last_bldc_packet_is_new; // BLDC motor driver packets

// Last BLDC packet sent information: TODO, slightly hacky
volatile unsigned long t_cmd_last = 0;
volatile int32_t position_last = 0;
volatile uint32_t current_last = 0;
volatile uint8_t flags_last =0;

// TODO remove these debugging things below
uint32_t ctrlCount;


// Interrupt running loop at 2 kHz ============================================
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    interrupt_count++;
    uint8_t i;

    //Telemetry save, at 1Khz
    //TODO: Break coupling between PID module and telemetry triggering
    if(interrupt_count == 1) {
        // Sensing
        mpuBeginUpdate(); // Start IMU and encoder reads
        amsEncoderStartAsyncRead();

        // Estimation and control
        kinematicUpdate();
        jumpModes();
        modeEstimation();

        if (t1_ticks == T1_MAX) t1_ticks = 0;
        t1_ticks++;
        if (!telemDecimateCount){
            telemSaveNow();
        }
        telemDecimateCount = (telemDecimateCount+1)%TELEM_DECIMATE;
    }

    if(interrupt_count == 2) {
        interrupt_count = 0;
        ctrlCount++;

        // Sensing
        mpuGetGyro(gdata); // This should be the only call to mpuGetGyro(gdata)
        mpuGetXl(xldata); // Similarly, this should only be called once
        orientImageproc(gdataBody, gdata); // orient gyro readings to body

        for (i=0; i<3; i++) {
            // Low pass the gyro signal
            w[i] = ((256-W_ALPHA)*w[i] + W_ALPHA*gdataBody[i])>>8;

            // Keep a circular buffer of readings for compensating mocap lag
            qLagSum[i] += (-qLagLog[qLagInd][i] + gdataBody[i]);
            qLagLog[qLagInd][i] = gdataBody[i];
        }
        qLagInd = (qLagInd+1)%LAG_MS; // Circular buffer index

        // Estimation and control
        eulerUpdate(q,gdataBody,STEP_MS); // attitude integration
        attitudeCtrl();

        /*
        if (ROBOT_NAME == SALTO_1P_SANTA) {
            if (mj_state == MJ_AIR) { // in the air
                tiHChangeMode(1, TIH_MODE_COAST);
            } else { // brake on the ground
                tiHChangeMode(1, TIH_MODE_BRAKE);
            }
        }
        */
        
    }
    _T1IF = 0;
}

/*
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    if (t1_ticks == T1_MAX) t1_ticks = 0;
    t1_ticks++;
    if (!telemDecimateCount){
        telemSaveNow();
    }
    telemDecimateCount = (telemDecimateCount+1)%TELEM_DECIMATE;
    _T5IF = 0;
}
*/


void salto1p_functions(void) {
    // Low priority processing to be run in the main loop

    // Update yaw angle approximations
    cos_psi = cosApprox(q[0]);
    sin_psi = cosApprox(q[0]-PI/2);

    // Body velocity
    vB[0] = ((long)v[0]*cos_psi + (long)v[1]*sin_psi)>>COS_PREC;//vi[0];
    vB[1] = (-(long)v[0]*sin_psi + (long)v[1]*cos_psi)>>COS_PREC;//vi[1];
    vB[2] = v[2];

    // Onboard velocity control using flight phase attitude
    if (0) {
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
    
    // Timer setup
    SetupTimer1();
    //SetupTimer5();
    
    EnableIntT1; // turn on main interrupt
    //EnableIntT5; 

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

}

void SetupTimer5(void) {
    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:8
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_8 & T5_INT_PRIOR_1 & T5_SOURCE_INT;
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

    // Read femur angle and lookup tables indexed by the femur angle
    femur = calibPos(1);
    uint32_t femur_index = femur/ 64; // Scale position to 8 bits
    if(femur_index<255 || femur_index > 0){
        crank = crank_femur_256lut[femur_index];
        foot = leg_femur_256lut[femur_index];
        ma = MA_femur_256lut[femur_index];
    } // Reject bad femur readings outside of physical range

    // Read the motor
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (sensor_data->position*motPos_to_femur_crank_units) - BLDC_MOTOR_OFFSET; //UNITFIX
    if (mot - last_mot > 1<<14 || last_mot - mot > 1<<14) {
        mot = last_mot; // reject bad samples
    }
    last_mot = mot;

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
    tail_pos = calibPos(0);
    tail_vel = (((128-TAIL_ALPHA)*tail_vel) >> 7) + ((TAIL_ALPHA*(tail_pos - tail_prev)/STEP_MS) >> 7);
    // difference >> 1 (divide by 2) because it updates at 500Hz instead of 1kHz now
    tail_prev = tail_pos;

}

void jumpModes(void) {
    // Estimation based on mode
    // DESCRIPTION TODO

    // Hacky e-stop if the robot falls over
    if ((mj_state != MJ_STOPPED) 
            && ((q[0] > PI/4 || q[0] < -PI/4)
            || (q[1] > PI/4 || q[1] < -PI/4))) {
        mj_state = MJ_STOP;
    }

    switch(mj_state) {
        case MJ_START:
            mj_state = MJ_LAUNCH;
            break;

        case MJ_STOP:
            // TODO: how to stop?
            send_command_packet(&uart_tx_packet_global, 0, 0, 0);
            mj_state = MJ_STOPPED;
            break;

        case MJ_AIR:
            // Ground contact transition out of air to ground
            if (t1_ticks - transition_time > 200 
                    && (spring > 1000)) {
                mj_state = MJ_GND;
                transition_time = t1_ticks;
            } else { // remain in air state
                send_command_packet(&uart_tx_packet_global, legSetpoint+BLDC_CMD_OFFSET, GAINS_AIR, 2);
            }
            break;

        case MJ_GND:
            // Liftoff transition from ground to air
            if (t1_ticks - transition_time > 50
                    && (spring < 500 || femur > FULL_EXTENSION)
                    && crank > 8192) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            } else { // remain in ground state
                send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_GND, 2);
            }
            break;

        case MJ_LAUNCH:
            // Liftoff transition from launch to air
            if (t1_ticks - transition_time > 50 
                    && (spring < 500 || femur > FULL_EXTENSION)
                    && crank > 8192) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            } else { // remain in launch state
                send_command_packet(&uart_tx_packet_global, pushoffCmd+BLDC_CMD_OFFSET, GAINS_GND, 2);
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

    if (mj_state == MJ_GND || mj_state == MJ_LAUNCH) {
        if (last_state == MJ_AIR) { // The robot just touched down
            flightStanceTrans();
        }
        stanceUpdate();
    } else if (mj_state == MJ_AIR) { // Flight phase estimation
        if (last_state == MJ_GND || last_state == MJ_LAUNCH) { // robot thinks it just took off
            stanceFlightTrans();
        }
        flightUpdate();
    } else {
        leg = foot >> 2;
        legVel = 0;
    }
    last_state = mj_state;

    if (legVel > 20000) { legVel = 20000;}
    if (legVel < -20000) { legVel = -20000;}
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
    if (0){//procFlags & 0b10) {
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
    legVel += (((-GRAV_ACC + force/FULL_MASS) * STEP_MS) >> 1) + (legErr << 1);
    // legVel is in 1 m/s / (1000*2 ticks)
    // acceleration is in m/s^2 / (2^2 ticks)

    v[0] = 0; // zero velocities; not really necessary
    v[1] = 0;
    v[2] = 0; // also zeroing vertical velocity
}

void flightUpdate(void) {
    // Description TODO
    uint8_t j;
    leg = foot >> 2;
    legVel -= (GRAV_ACC*STEP_MS) >> 1; // gravitational acceleration

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
    for (j=0; j<3; j++) {
        TDvCmd[j] = vCmd[j]; // Save the desired velocities
        TDq[j] = q[j]; // Save the touchdown body angles
        TDv[j] = v[j]; // Save the touchdown body velocities
        TDqCmd[j] = qCmd[j];
    }
}

void stanceFlightTrans(void) {
    // Description TODO
    int16_t i, j;
    int16_t cntr;
    // save the pose and velocity states for calculating takeoff velocities
    // first, check if takeoff actually happened earlier
    cntr = vel_ind;
    TOlegVel = legVel;
    for (i=0; i<VEL_BUF_LEN; i++) {
        if (legBuf[cntr*2+1] - ((GRAV_ACC*i*STEP_MS) >> 1) > legVel) { // TODO: time assumed constant
            legVel = legBuf[cntr*2+1] - ((GRAV_ACC*i*STEP_MS) >> 1) ; // take max velocity as takeoff velocity

            TOlegVel = legBuf[cntr*2+1];
            g_accumulator = i*STEP_MS; // set the accumulator to the number of steps elapsed

            TOleg = legBuf[cntr*2];
            for (j=0; j<3; j++) {
                TOq[j] = angBuf[cntr*6+j];
                TOw[j] = angBuf[cntr*6+3+j];
            }
        }
        cntr -= 1;
        if (cntr < 0) {cntr = VEL_BUF_LEN - 1;}
    }
    legVel = legVel + 700; // takeoff boost of 0.35 m/s
    TOlegVel = TOlegVel + 700;

    //velocity[2] = -velocity[2]; // velocity estimate until real calculation is done
    procFlags |= 0b1; // tell the main loop to calculate the takeoff velocities
}

void takeoffEstimation(void ) {
    //Calculate estimated velocities on takeoff

    //*
    long TOcos_theta = cosApprox(TOq[1]);
    long TOsin_theta = cosApprox(TOq[1]-PI/2);
    long TOcos_phi = cosApprox(TOq[0]);
    long TOsin_phi = cosApprox(TOq[0]-PI/2);
    long TOcos_psi = cosApprox(TOq[2]);
    long TOsin_psi = cosApprox(TOq[2]-PI/2);

    // Compensate for CG offset
    TOw[1] -= 0.05*0.469*TOlegVel; // (2^15/2000*180/pi)/2000 = 0.4694
    TOw[0] -= -0.05*0.469*TOlegVel;
    //TObody_vel_LP[2] -= 0.10*0.469*TOlegVel; // (2^15/2000*180/pi)/2000 = 0.4694
    //TObody_vel_LP[1] -= -0.10*0.469*TOlegVel;

    // Body velocity rotation matrix
    int32_t vxw = 14418*(int32_t)TOw[1]/30760; // locking TOleg at 0.22m
    int32_t vyw = -14418*(int32_t)TOw[0]/30760;
    //int32_t vxw = (int32_t)TOleg*(int32_t)TOw[2]/30760;
    //int32_t vyw = -(int32_t)TOleg*(int32_t)TOw[1]/30760;
    // native units (body_vel_LP*leg: 2000/2^15 (deg/s)/tick * 1/2^16 m/ticks * pi/180 rad/deg
    // final units (legVel): 1/2000 (m/s)/tick
    // unit conversion: 1/30760.437 tick/tick

     
    long velX = (((((TOcos_psi*TOcos_theta) >> COS_PREC)
            - ((((TOsin_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        - ((((TOcos_phi*TOsin_psi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOcos_psi*TOsin_theta) >> COS_PREC) 
            + ((((TOcos_theta*TOsin_psi) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
    long velY = (((((TOcos_theta*TOsin_psi) >> COS_PREC)
            + ((((TOcos_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        + ((((TOcos_psi*TOcos_phi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOsin_psi*TOsin_theta) >> COS_PREC)
            - ((((TOcos_psi*TOcos_theta) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
    long velZ = - ((((TOcos_phi*TOsin_theta) >> COS_PREC)*vxw) >> COS_PREC)
        + ((TOsin_phi*vyw) >> COS_PREC)
        + ((((TOcos_theta*TOcos_phi) >> COS_PREC)*TOlegVel) >> COS_PREC);
    //Z1X2Y3 https://en.wikipedia.org/wiki/Euler_angles

    //velY = 17*velY/20;

    v[0] = velX;
    v[1] = velY;
    v[2] = velZ;

    int i;
    for (i=1; i<3; i++) {
        TDq[i] -= TDqCmd[i]; // calculate angle error
    }

    // Deadbeat-based correction
    //long predicted_angles[3];
    //deadbeat(TDvelocity, velocity, predicted_angles);
    //att_correction[0] = (TDbody_angle[2] - predicted_angles[0])>>2;
    //att_correction[1] = (TDbody_angle[1] - predicted_angles[1])>>2;

    // Velocity-based correction
    long velocity_x = (v[0]*TOcos_psi + v[1]*TOsin_psi)>>COS_PREC;
    long velocity_y = (-v[0]*TOsin_psi + v[1]*TOcos_psi)>>COS_PREC;

    att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - TDvCmd[0]));// - (TDq[2]>>1));
    att_correction[1] = ATT_CORRECTION_GAIN_Y*(velocity_y - TDvCmd[1]);// + (TDq[1]>>1);

    //att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - stance_vel_des[0]) * 188) /
    //    ((long)(-velocity[2] + TDvelocity[2]) >> 6);
    //att_correction[1] = (ATT_CORRECTION_GAIN_Y*(velocity_y - stance_vel_des[1]) * 188) /
    //    ((long)(-velocity[2] + TDvelocity[2]) >> 6); // (3*2000 >> 6)*2 is about 188

    att_correction[0] = att_correction[0] > 15000 ? 15000 :
                        att_correction[0] < -15000 ? -15000 :
                        att_correction[0];
    att_correction[1] = att_correction[1] > 15000 ? 15000 :
                        att_correction[1] < -15000 ? -15000 :
                        att_correction[1];

    procFlags |= 0b10;
    procFlags &= ~0b1;
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
    long iz = (vi[2] > -2000 ? -2000 : vi[2]) + 6600;

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

#ifdef FULL_POWER
    // 100% gains from runGridMotor18
    long pit_ctrl = -95*ix +40*ox
        -17*ixiz +9*oxiz -4*ixoz -20*oxoz
        +1*ixixix -2*oxoxox; // Scaled by 469 approx = PI/(3.14159*2000)

    long rol_ctrl = -(-95*iy +40*oy
        -17*iyiz +9*oyiz -4*iyoz -20*oyoz
        +1*iyiyiy -2*oyoyoy);

    long leg_ctrl = (77*65536 -542*iz -622*oz
        +58*(ixix+iyiy) -33*iziz -117*(oxox+oyoy) -142*ozoz
        +38*iziziz -82*ozozoz
        +18*(ixixiz+iyiyiz) +10*(oxoxiz+oyoyiz) -9*(ixixoz+iyiyoz) +1*(oxoxoz+oyoyoz));
        // Scaled by 65536/2000
#else
    // Gains with shell: 108 g robot from runGridMotor17
    long pit_ctrl = -100*ix +31*ox // supposed to be 100 31
        -17*ixiz +11*oxiz -3*ixoz -19*oxoz
        +1*ixixix -2*oxoxox; // Scaled by 469 approx = PI/(3.14159*2000)

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


#define TAIL_BRAKE 20
#define TAIL_REVERSE 5 // out of 128

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

    // Attitude actuator mixing
    foreThruster = rolPD - yawPD;
    aftThruster = rolPD + yawPD;
    tailMotor = pitPD;

    if (mj_state != MJ_AIR) {
        tailMotor = -TAIL_BRAKE*(tail_vel + TAIL_REVERSE*(w[1]>>7));
    }

    foreThruster = foreThruster > MAX_THROT ? MAX_THROT :
                   foreThruster < -MAX_THROT ? -MAX_THROT :
                   foreThruster;
    aftThruster = aftThruster > MAX_THROT ? MAX_THROT :
                  aftThruster < -MAX_THROT ? -MAX_THROT :
                  aftThruster;
    tailMotor = tailMotor > MAX_THROT ? MAX_THROT :
                tailMotor < -MAX_THROT ? -MAX_THROT :
                tailMotor;

    if (mj_state != MJ_STOP && mj_state != MJ_STOPPED && mj_state != MJ_IDLE) {
        tiHSetDC(0+1, tailMotor);
        tiHSetDC(2+1, foreThruster);
        tiHSetDC(3+1, aftThruster);
    } else {
        for (i=0; i<4; i++) {
            tiHSetDC(i+1, 0);
        }
    }
}


// Communications functions ===================================================
void setGains(int16_t* gains) {
    uint8_t i;
    for (i=0; i<9; i++) {
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
}

void setPushoffCmd(long cmd){
    pushoffCmd = cmd << 8;
}

void setBodyAngle(long* qSet) {
    q[0] = qSet[1];
    q[1] = qSet[2];
    q[2] = qSet[0];
}

void adjustBodyAngle(long* qAdjust){
    q[0] += qAdjust[1];
    q[1] += qAdjust[2];
    q[2] += qAdjust[0];
}

void updateBodyAngle(long* qUpdate){
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

    q[0] = PI*body_acc[1]/(3.14159*body_acc[2]); // roll
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
    //start_time = t1_ticks;
}

void expStop(uint8_t stopSignal) {
    mj_state = MJ_STOP;
}

void setOnboardMode(uint8_t flags, uint8_t mode) {
    modeFlags = flags;
}

void setVelocitySetpoint(int16_t* newCmd, int32_t newYaw) {
    int i;

    newCmd[2] = newCmd[2] < vB[0]<<1 ? vB[0]<<1 : // don't jump too low when going fast
                newCmd[2] < -vB[0]<<1 ? -vB[0]<<1 : 
                newCmd[2] < vB[1]<<1 ? vB[1]<<1 : 
                newCmd[2] < -vB[1]<<1 ? -vB[1]<<1 :
                newCmd[2];
    newCmd[2] = newCmd[2] > 8000 ? 8000 : // max height
                newCmd[2] < 4000 ? 4000 : // min height
                newCmd[2];

    newCmd[0] = newCmd[0] > newCmd[2]-2000 ? newCmd[2]-2000 : // limit speed by height
                newCmd[0] < -(newCmd[2]-2000) ? -(newCmd[2]-2000) :
                newCmd[0];
    newCmd[1] = newCmd[1] > (newCmd[2]-2000)>>1 ? (newCmd[2]-2000)>>1 :
                newCmd[1] < -(newCmd[2]-2000)>>1 ? -(newCmd[2]-2000)>>1 :
                newCmd[1];

    newCmd[0] = newCmd[0] > vB[0]+3000 ? vB[0]+3000 : // limit velocity change
                newCmd[0] < vB[0]-3000 ? vB[0]-3000 :
                newCmd[0];
    newCmd[1] = newCmd[1] > vB[1]+1500 ? vB[1]+1500 :
                newCmd[1] < vB[1]-1500 ? vB[1]-1500 :
                newCmd[1];

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

    if (((t1_ticks - t_cmd_last) < UART_PERIOD) ||
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
    t_cmd_last = t1_ticks;
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
        v_b[2] = (165*((int32_t)v_ip[2]) - 196*((int32_t)v_ip[0]))>>8; //yaw
        v_b[0] = -v_ip[1]; // roll
        v_b[1] = (165*((int32_t)v_ip[0]) + 196*((int32_t)v_ip[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_DASHER
        // -50 degrees about x, follwed by 180 degrees about body z
        v_b[2] = (165*((int32_t)v_ip[2]) - 196*((int32_t)v_ip[0]))>>8; //yaw
        v_b[0] = -v_ip[1]; // roll
        v_b[1] = (165*((int32_t)v_ip[0]) + 196*((int32_t)v_ip[2]))>>8; //pitch
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

int32_t cosApprox(int32_t x) {
    // Cosine approximation by Bhaskara I's method:
    // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
    //
    // OUTPUT: cosine(x) scaled to a value between -2^COS_PREC and 2^COS_PREC
    // INPUTS:
    // x: angle scaled according to the value of #define constant PI

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
}