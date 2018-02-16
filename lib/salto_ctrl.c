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



packet_union_t uart_tx_packet_global; // BLDC motor driver packet

static volatile unsigned char interrupt_count = 0;
volatile unsigned char mj_state = MJ_IDLE;

extern EncObj encPos[NUM_ENC];
extern unsigned long t1_ticks;

// Orientation
long body_angle[3];     // Current body angle estimate
long body_velocity[3];  // Current body velocity estimate
long vicon_angle[3];    // Last Vicon-measured body angle
long body_vel_LP[3];    // Low-passed body velocity estimate

// Setpoints and commands
char pitchControlFlag = 0; // enable/disable attitude control
volatile long pitchSetpoint = 0;
volatile long rollSetpoint = 0;
volatile long yawSetpoint = 0;
volatile long legSetpoint = 0;  // Aerial leg setpoint
volatile long pushoffCmd = 0;   // Ground leg command

#define TAIL_ALPHA 25 // out of 128
long tail_pos = 0; // in ticks
long tail_prev = 0; // in ticks
long tail_vel = 0; // in ticks / count


void setPitchControlFlag(char state){
    pitchControlFlag = state;
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

#define LAG_MS  1
void updateViconAngle(long* new_vicon_angle){
    int i;
    for (i=0; i<3; i++){
        vicon_angle[i] = new_vicon_angle[i];
        body_angle[i] = 3*(body_angle[i] >> 2) + (new_vicon_angle[i] >> 2);
    }
    //updateEuler(body_vel_LP,LAG_MS);
}

void resetBodyAngle(){
    // mpuRunCalib(0,100); //re-offset gyro, assumes stationary
    body_angle[0] = 0;
    body_angle[1] = 0;
    body_angle[2] = 0;
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


#define BVLP_ALPHA 128 // out ouf 256
///////        Tail control ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;
    int i;

    if(interrupt_count <= 5) {
        // Do signal processing on gyro
        int gdata[3];
        mpuGetGyro(gdata);

        gdata[0] -= -5;//20; // ImageProc board short axis
        gdata[1] -= 15;//20; // ImageProc board long axis
        gdata[2] -= 0;//-20; // ImageProc board normal
#if ROBOT_NAME == SALTO_1P_RUDOLPH
        // -55 degrees about roll
        // x axis right, y axis forwards, z axis up from ImageProc
        body_velocity[0] = (147*((long)gdata[2]) + 210*((long)gdata[0]))>>8; //yaw
        body_velocity[1] = gdata[1];
        body_velocity[2] = (-147*((long)gdata[0]) + 210*((long)gdata[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_DASHER
        // -55 degrees about x, follwed by 180 degrees about body z
        body_velocity[0] = (147*((long)gdata[2]) - 210*((long)gdata[0]))>>8; //yaw
        body_velocity[1] = -gdata[1];
        body_velocity[2] = (147*((long)gdata[0]) + 210*((long)gdata[2]))>>8; //pitch
#elif ROBOT_NAME == SALTO_1P_SANTA
        // -45 degrees about pitch
        body_velocity[0] = (((long)(gdata[2] + gdata[1]))*181)>>8; //yaw
        body_velocity[1] = (((long)(gdata[1] - gdata[2]))*181)>>8; //roll
        body_velocity[2] = -gdata[0]; //pitch
#endif

        for (i=0; i<3; i++) {
            body_vel_LP[i] = ((256-BVLP_ALPHA)*body_vel_LP[i] + BVLP_ALPHA*body_velocity[i])>>8;
        }

        updateEuler(body_vel_LP,1);

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
            if (mj_state == MJ_AIR) {
                tiHChangeMode(1, TIH_MODE_COAST);
                pidObjs[0].mode = 0;
                pidObjs[0].p_input = pitchSetpoint;
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
    int gdata[3];
    mpuGetGyro(gdata);
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
            send_command_packet(&uart_tx_packet_global, legSetpoint, 0, 2);

            // Ground contact transition out of air to ground
            if (t1_ticks - transition_time > 200 && (footContact() == 1)) {
                mj_state = MJ_GND;
                transition_time = t1_ticks;
            }
            break;

        case MJ_GND:
            send_command_packet(&uart_tx_packet_global, pushoffCmd, 0, 2);

            // Liftoff transition from ground to air
            if (t1_ticks - transition_time > 50 && (footTakeoff() == 1 || calibPos(2) > FULL_EXTENSION)) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            }
            break;

        case MJ_STOPPED:
            send_command_packet(&uart_tx_packet_global, 0, 0, 0);
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
    if (((t1_ticks - t_cmd_last) < UART_PERIOD)) {//|| 
        //(position == position_last && current == current_last && flags == flags_last)) {
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


#define PI          2949120 // 180(deg) * 2^15(ticks)/2000(deg/s) * 1000(Hz)
#define PISQUARED   132710400 // bit shifted 16 bits down
#define COS_PREC    15 // bits of precision in output of cosine
void updateEuler(long* vels, long time) {
    // Update the body_angle Euler angle estimates.
    // INPUTS:
    // long vels[3] is the body-fixed angular velocities from the gyro
    // time: time in milliseconds (usually 1 to 20)

    // TODO: what if updateEuler is called reentrantly
    long temp_angle[3];
    int i;
    for (i=0; i<3; i++) {
        temp_angle[i] = body_angle[i];
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
        body_angle[i] = temp_angle[i];
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
