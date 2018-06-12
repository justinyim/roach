/*
 * Name: salto_ctrl.c
 * Desc: Salto attitude control: inertial tail and two thrusters
 * Date: 2017-05-01
 * Author: JKY (Justin Yim)
 *
 */

#include "salto_ctrl.h"
#include "settings.h"

#include "led.h"
#include "mpu6000.h"
#include "timer.h"
#include "tih.h"

#include "as5047.h"
#include "ams-enc.h"
#include "protocol.h"
#include "uart_driver.h"
#include "lut.h"

#include "ports.h"

#include "pid-ip2.5.h"
extern pidPos pidObjs[NUM_PIDS];


#define PI          2949120 // 180(deg) * 2^15(ticks)/2000(deg/s) * 1000(Hz)
#define PISQUARED   132710400 // bit shifted 16 bits down
#define COS_PREC    15 // bits of precision in output of cosine


packet_union_t uart_tx_packet_global; // BLDC motor driver packet

static volatile unsigned char interrupt_count = 0;
volatile unsigned char mj_state = MJ_IDLE;

extern EncObj encPos[NUM_ENC];
extern unsigned long t1_ticks;

// Orientation
int gdata[3];           // raw gyro readings

long body_angle[3];     // Current body angle estimate (0 yaw, 1 roll, 2 pitch)
long body_velocity[3];  // Current body velocity estimate
long vicon_angle[3];    // Last Vicon-measured body angle
long body_vel_LP[3];    // Low-passed body velocity estimate (0 yaw, 1 roll, 2 pitch)
#define VICON_LAG 20    // Vicon comms lag in ImageProc control steps (one step per ms)
long body_lag_log[VICON_LAG][3];    // circular buffer history of body angles for countering lag
long body_lag_sum[3] = {0,0,0};     // average angular velocity over VICON_LAG steps
unsigned char BLL_ind = 0;          // circular buffer index

// Setpoints and commands
char pitchControlFlag = 0; // enable/disable attitude control
char onboardMode = 0; // mode for onboard controllers & estimators
volatile long pitchSetpoint = 0;
volatile long rollSetpoint = 0;
volatile long yawSetpoint = 0;
volatile long legSetpoint = 0;  // Aerial leg setpoint
volatile long pushoffCmd = 0;   // Ground leg command

#define TAIL_ALPHA 25 // out of 128
long tail_pos = 0; // in ticks
long tail_prev = 0; // in ticks
long tail_vel = 0; // in ticks / count


#define P_AIR ((1*65536)/10) // leg proportional gain in the air (duty cycle/rad * 65536)
#define D_AIR ((2*65536)/1000) // leg derivative gain in the air (duty cycle/[rad/s] * 65536)
#define P_GND ((5*65536)/10) // leg proportional gain on the ground
#define D_GND ((1*65536)/1000)
#define P_STAND ((5*65536)/1000) // leg proportional gain for standing
#define D_STAND ((3*65536)/1000)

uint32_t GAINS_AIR = (P_AIR<<16)+D_AIR;
uint32_t GAINS_GND = (P_GND<<16)+D_GND;
uint32_t GAINS_STAND = (P_STAND<<16)+D_STAND;


void setPitchControlFlag(char state){
    pitchControlFlag = state;
}

void setOnboardMode(char mode, char flags){
    // onboard mode switch (first introduced for testing onboard gyro integration only 29 May 2018)
    // 0: previous default operation
    // 1: use only onboard gyro integration and no vicon attitude updates
    LED_1 ^= 1;
    onboardMode = mode;
}

void setAttitudeSetpoint(long yaw, long roll, long pitch){
    yawSetpoint = yaw;
    rollSetpoint = roll;
    pitchSetpoint = pitch;
}

void setLegSetpoint(long length){
    legSetpoint = length << 8;
}

void setPushoffCmd(long cmd){
    pushoffCmd = cmd << 8;
}

void updateViconAngle(long* new_vicon_angle){
    int i;

    if (onboardMode == 1 || onboardMode == 3 || onboardMode == 7){ // onboardMode 1: use only gyro integration
        return;
    }

    updateEuler(vicon_angle,body_lag_sum,1); // attempt to counter comms lag

    for (i=0; i<3; i++){
        vicon_angle[i] = new_vicon_angle[i];
        body_angle[i] = 3*(body_angle[i] >> 2) + (new_vicon_angle[i] >> 2);
    }
}

void resetBodyAngle(){
    //mpuRunCalib(0,100); //re-offset gyro, assumes stationary
    body_angle[0] = 0;
    body_angle[1] = 0;
    body_angle[2] = 0;
}

void calibGyroBias(){
    mpuRunCalib(0,3); //re-offset gyro, assumes stationary
    // TODO this fucntion call is sometimes causing Salto to hang
}

void accZeroAtt(){
    // get an APPROXIMATE Euler angle relative to the g vector from the accelerometer radings

    int xldata[3];  // accelerometer data
    mpuGetXl(xldata);
    long body_acc[3]; // body-frame accelerometer readings
    orientImageProc(body_acc, xldata);

    body_angle[1] = PI*body_acc[2]/(3.14159*body_acc[0]); // roll
    body_angle[2] = -PI*body_acc[1]/(3.14159*body_acc[0]); // pitch
}

void expStart(uint8_t startSignal) {
    mj_state = MJ_START;
}

void expStop(uint8_t stopSignal) {
    mj_state = MJ_STOP;
}



void tailCtrlSetup(){
    body_angle[0]=0;
    body_angle[1]=0;
    body_angle[2]=0;
    initPIDObjPos( &(pidObjs[0]), 0,0,0,0,0);
    // initPIDObjPos( &(pidObjs[2]), -500,0,-500,0,0);
    initPIDObjPos( &(pidObjs[2]), 0,0,0,0,0);
    initPIDObjPos( &(pidObjs[3]), 0,0,0,0,0); //100,100
    SetupTimer5();
    EnableIntT5;

    pidObjs[0].timeFlag = 0;
    pidObjs[0].mode = 0;
    pidSetInput(0, 0);
    pidObjs[0].p_input = pidObjs[0].p_state;
    pidObjs[2].p_input = 0;
    pidObjs[3].p_input = 0;
    pidObjs[2].v_input = 0;
    pidObjs[3].v_input = 0;

    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16);
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16);

}


#define BVLP_ALPHA 64 // out ouf 256
///////        Tail control ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;
    int i;

    if(interrupt_count <= 5) {
        // Do signal processing on gyro
        mpuGetGyro(gdata); // This should be the only call to mpuGetGyro(gdata)
        orientImageProc(body_velocity, gdata); // orient gyro readings gdata into body frame body_velocity

        for (i=0; i<3; i++) {
            body_vel_LP[i] = ((256-BVLP_ALPHA)*body_vel_LP[i] + BVLP_ALPHA*body_velocity[i])>>8;
        }

        body_lag_sum[0] += (-body_lag_log[BLL_ind][0] + body_velocity[0]);
        body_lag_sum[1] += (-body_lag_log[BLL_ind][1] + body_velocity[1]);
        body_lag_sum[2] += (-body_lag_log[BLL_ind][2] + body_velocity[2]);
        body_lag_log[BLL_ind][0] = body_velocity[0];
        body_lag_log[BLL_ind][1] = body_velocity[1];
        body_lag_log[BLL_ind][2] = body_velocity[2];
        BLL_ind = (BLL_ind+1)%VICON_LAG;

        //updateEuler(body_angle,body_vel_LP,1);
        updateEuler(body_angle,body_velocity,1);

        // Additional 1kHz attitude estimator actions
        if (onboardMode == 3 || onboardMode == 7) {
            body_angle[1] += (pidObjs[2].output>>7) + (pidObjs[3].output>>7);
            body_angle[2] += (pidObjs[0].output>>5);//(tail_vel>>3);
        }

        // Tail estimation
        tail_pos = calibPos(1);
        tail_vel = (((128-TAIL_ALPHA)*tail_vel) >> 7) + ((TAIL_ALPHA*(tail_pos - tail_prev)) >> 7);
        tail_prev = tail_pos;

    }
    if(interrupt_count == 5) 
    {
        interrupt_count = 0;
        multiJumpFlow();
        if(pitchControlFlag == 0){
            //LED_2 = 0;
            // Control/motor off
            pidObjs[0].onoff = 0;
            pidObjs[2].onoff = 0;
            pidObjs[3].onoff = 0;
        } else {
            //LED_2 = 1;
            // Control pitch, roll, and yaw
            if (mj_state == MJ_AIR && pidObjs[0].mode != 2) {
                tiHChangeMode(1, TIH_MODE_COAST);
                pidObjs[0].mode = 0;
                pidObjs[0].p_input = pitchSetpoint;
            } else if (onboardMode == 3 || onboardMode == 7) { // standing on the ground
                tiHChangeMode(1, TIH_MODE_COAST);
                pidObjs[0].mode = 2;
            } else { // brake on the ground
                if (ROBOT_NAME == SALTO_1P_SANTA) {
                    tiHChangeMode(1, TIH_MODE_BRAKE);
                    pidObjs[0].mode = 1;
                } else {
                    pidObjs[0].mode = 1;
                }
                //pidObjs[0].pwmDes = 0; this is not useful
            }
            pidObjs[2].p_input = rollSetpoint;
            pidObjs[3].p_input = yawSetpoint;
        }
    }

    _T5IF = 0;
}


void SetupTimer5() {
    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:64
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    T5PERvalue = 625; //1Khz
    OpenTimer5(T5CON1value, T5PERvalue);
}


long transition_time = 0;
void multiJumpFlow() {
    //int gdata[3];
    //mpuGetGyro(gdata);
    switch(mj_state) {
        case MJ_START:
            pidObjs[0].timeFlag = 0;
            setPitchControlFlag(1);
            pidOn(0);
            pidOn(2);
            pidOn(3);

            mj_state = MJ_GND;
            break;

        case MJ_STOP:
            pidObjs[0].onoff = 0;
            pidObjs[2].onoff = 0;
            pidObjs[3].onoff = 0;
            tiHSetDC(1,0);
            tiHSetDC(3,0);
            tiHSetDC(4,0);
            send_command_packet(&uart_tx_packet_global, 0, 0, 0);
            mj_state = MJ_STOPPED;
            break;

        case MJ_AIR:
            // Ground contact transition out of air to ground
            if (t1_ticks - transition_time > 200 && (footContact() == 1)) {
                mj_state = MJ_GND;
                transition_time = t1_ticks;
            } else { // remain in air state
                if (onboardMode != 7) {
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_AIR, 2);
                } else {
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_STAND, 2);
                }
            }
            break;

        case MJ_GND:
            // Liftoff transition from ground to air
            if (t1_ticks - transition_time > 50 && (footTakeoff() == 1 || calibPos(2) > FULL_EXTENSION)) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            } else { // remain in ground state
                if (onboardMode != 7) {
                    send_command_packet(&uart_tx_packet_global, pushoffCmd, GAINS_GND, 2);
                } else {
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_STAND, 2);
                }
            }
            break;

        case MJ_STOPPED:
            break;

        default:
            mj_state = MJ_IDLE;
            break;
    }
}


volatile unsigned long t_cmd_last = 0;
volatile int32_t position_last = 0;
volatile uint32_t current_last = 0;
volatile uint8_t flags_last =0;
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags){
    if (((t1_ticks - t_cmd_last) < UART_PERIOD) ||
        (position == position_last && current == current_last && flags == flags_last)) {
        return; // skip command sending if last command was too recent or was identical
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


extern packet_union_t* last_bldc_packet;
extern uint8_t last_bldc_packet_is_new;

char footContact(void) {
    int eps = 1000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur();
    if ( mot-BLDC_MOTOR_OFFSET>femur && (mot-BLDC_MOTOR_OFFSET - eps) > femur)
    {
        LED_1 = 1;
        return 1;
    } else {
        LED_1 = 0;
        return 0;
    }
}

char footTakeoff(void) {
    int eps = 1000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur();
    if ( (mot-BLDC_MOTOR_OFFSET + eps) < femur){
        return 1;
    } else {
        return 0;
    }
}

long calibPos(char idx){
    long temp;
    // idx: 0 = motor, 1 = tail, 2 = femur
    if (idx == 0) {
        return pidObjs[1].p_state;
    }
    else if (idx == 1) {
        temp = (long)(encPos[0].pos << 2);       // pos 14 bits 0x0 -> 0x3fff
        return temp + (encPos[0].oticks << 16);
    }
    else if (idx == 2) {
        temp = -(((long)encPos[1].pos - (long)encPos[1].offset) << 2);  // pos 14 bits 0x0 -> 0x3fff
        return temp - (encPos[1].oticks << 16);
    }
    else {
        return -1;
    }
}

unsigned int crankFromFemur(void) { 
    unsigned int femur;
    femur = calibPos(2) / 64; // Scale position to 8 bits
    if(femur>255){femur=255;}
    if(femur<0){femur=0;} 
    return crank_femur_256lut[femur];
}


void orientImageProc(long* body_frame, int* ImageProc_frame) {
    // Rotate ImageProc_frame vectors from the ImageProc IMU frame (0:x,right, 1:y,forwards, 2:z,up)
    // into body_frame in the robot body frame (0:z,up,yaw, 1:x,forwards,roll, 2:y,left,pitch)
    // ImageProc_frame is an int (from MPU IMU) but body_frame is a long
#if ROBOT_NAME == SALTO_1P_RUDOLPH
        // -55 degrees about roll
        // x axis right, y axis forwards, z axis up from ImageProc
        body_frame[0] = (147*((long)ImageProc_frame[2]) + 210*((long)ImageProc_frame[0]))>>8; //yaw
        body_frame[1] = ImageProc_frame[1];
        body_frame[2] = (-147*((long)ImageProc_frame[0]) + 210*((long)ImageProc_frame[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_DASHER
        // -55 degrees about x, follwed by 180 degrees about body z
        body_frame[0] = (147*((long)ImageProc_frame[2]) - 210*((long)ImageProc_frame[0]))>>8; //yaw
        body_frame[1] = -ImageProc_frame[1];
        body_frame[2] = (147*((long)ImageProc_frame[0]) + 210*((long)ImageProc_frame[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_SANTA
        // -45 degrees about pitch
        body_frame[0] = (((long)(ImageProc_frame[2] + ImageProc_frame[1]))*181)>>8; //yaw
        body_frame[1] = (((long)(ImageProc_frame[1] - ImageProc_frame[2]))*181)>>8; //roll
        body_frame[2] = -ImageProc_frame[0]; //pitch
#endif
}


void updateEuler(long* angs, long* vels, long time) {
    // Update the Euler angle estimates (usually body_angle).
    // INPUTS:
    // long angs[3] are the Euler angles to be updated
    // long vels[3] is the body-fixed angular velocities from the gyro
    // time: time in milliseconds (usually 1 to 20)

    // TODO: what if updateEuler is called reentrantly
    long temp_angle[3];
    int i;
    for (i=0; i<3; i++) {
        temp_angle[i] = angs[i];
    }

    long sin_theta = cosApprox(temp_angle[2]-PI/2);
    long cos_theta = cosApprox(temp_angle[2]);
    long sin_phi = cosApprox(temp_angle[1]-PI/2);
    long cos_phi = cosApprox(temp_angle[1]);

    // Prevent divide by zero
    if (cos_phi == 0) { cos_phi = 1; }

    // Update Euler angles
    temp_angle[2] += ((sin_phi*sin_theta*vels[1]/cos_phi
            + (vels[2] << COS_PREC)
            - cos_theta*sin_phi*vels[0]/cos_phi)*time) >> COS_PREC;
    temp_angle[1] += (cos_theta*vels[1] + sin_theta*vels[0])*time >> COS_PREC;
    temp_angle[0] += (-sin_theta*vels[1]/cos_phi + cos_theta*vels[0]/cos_phi)*time;

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

long cosApprox(long x) {
    // Cosine approximation by Bhaskara I's method:
    // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
    //
    // Returns a value between -2^COS_PREC and 2^COS_PREC
    // INPUTS:
    // x: angle scaled by 16384 ticks per degree (2000 deg/s MPU gyro integrated @ 1kHz).

    long xSquared;
    if (x < 0) { x = -x; }
    long x_mod = (x+PI) % (2*PI) - PI;
    if (x_mod > (PI/2)) { // quadrant 2
        x_mod = (x_mod - PI) >> 8;
        xSquared = x_mod*x_mod;
        return -(PISQUARED - 4*xSquared)/((PISQUARED + xSquared)>>COS_PREC);
    } else if (x_mod < (-PI/2)) { // quadrant 3
        x_mod = (x_mod + PI) >> 8;
        xSquared = x_mod*x_mod;
        return -(PISQUARED - 4*xSquared)/((PISQUARED + xSquared)>>COS_PREC);
    } else {
        x_mod = x_mod >> 8; // quadrants 1 and 4
        xSquared = x_mod*x_mod;
        return (PISQUARED - 4*xSquared)/((PISQUARED + xSquared)>>COS_PREC);
    }
}
