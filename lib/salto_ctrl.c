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
int gdata[3];           // raw gyro readings
#define BVLP_ALPHA 64  // Low pass attitude angles (out ouf 256)
#define VICON_LAG 20    // Vicon comms lag in ImageProc control steps (one step per ms)
long body_angle[3];     // Current body angle estimate (0 yaw, 1 roll, 2 pitch) [PI ticks/(pi rad)]
long body_velocity[3];  // Current angular velocity estimate (0 yaw, 1 roll, 2 pitch) [2^15 ticks/(2000 deg/s)]
long vicon_angle[3];    // Last Vicon-measured body angle (0 yaw, 1 roll, 2 pitch) [PI ticks/(pi rad)]
long body_vel_LP[3];    // Low-passed body velocity estimate (0 yaw, 1 roll, 2 pitch) [2^15 ticks/(2000 deg/s)]
long body_lag_log[VICON_LAG][3];    // circular buffer history of body angles for countering lag
long body_lag_sum[3] = {0,0,0};     // average angular velocity over VICON_LAG steps
long body_vel_500[3];   // Summed angular velocities for 500 Hz Euler update
unsigned char BLL_ind = 0;          // circular buffer index
unsigned char BLL_ind_prev = 0;     // previous BLL_ind

long sin_theta = 0;     // pitch angle in COS_PREC bits
long cos_theta = 1<<COS_PREC;
long sin_phi = 0;       // roll angle
long cos_phi = 1<<COS_PREC;
long sin_psi = 0;       // yaw angle
long cos_psi = 1<<COS_PREC;

// Robot position estimation states
int16_t position[3];    // Esimated robot location in world frame (UNUSED)
int16_t velocity[3];    // Estimated robot velocity in world frame (x, y, z) [2000 ticks/(m/s)]
int16_t leg;            // Estimated stance phase leg length [2^16 ticks/m]
int16_t legVel;         // Estimated stance phase leg extension velocity [2000 ticks/(m/s)]
int16_t vel_des[3];     // Desired velocities [2000 ticks/(m/s)]
int16_t stance_vel_des[3]; // Desired velocities at touchdown [2000 ticks/(m/s)]
long att_correction[2]; // Stance attitude correction [PI ticks/(pi rad)]

int32_t crank;      // Crank angle [2^14 ticks/rad]
int32_t foot;       // Foot distance [2^14 ticks/m]
int32_t MA;         // Mechanical advantage [2^9 ticks/(N/Nm)]
int32_t spring;     // Spring torque [2^14 ticks/(Nm)]
int32_t force;      // Foot force [2^10 ticks/N]

#define T_STEP 70//70 // 70 ms
#define K_RAIBERT 8//8 // 0.008 m/(m/s) = 8 ms
#define CTRL_CONVERT 1 // >> 1 is about 0.469 = PI/3.14159  * 1/(1000*2000) // from meters to radians
#define INVERSE_LEG_LEN 5 // 5 = 1/0.2m
long x_ctrl, y_ctrl;    // Onboard roll and pitch commands

// Setpoints and commands
char controlFlag = 0; // enable/disable attitude control
char onboardMode = 0; // mode for onboard controllers & estimators
volatile long pitchSetpoint = 0;
volatile long rollSetpoint = 0;
volatile long yawSetpoint = 0;
volatile long legSetpoint = 0;  // Aerial leg setpoint
volatile long pushoffCmd = 0;   // Ground leg command

#define TAIL_ALPHA 25 // Low pass tail velocity out of 128
long tail_pos = 0; // in ticks
long tail_prev = 0; // in ticks
long tail_vel = 0; // in ticks / count


#define P_AIR ((1*65536)/10) // leg proportional gain in the air (duty cycle/rad * 65536)
#define D_AIR ((3*65536)/1000) // leg derivative gain in the air (duty cycle/[rad/s] * 65536)
#define P_GND ((5*65536)/10) // leg proportional gain on the ground
#define D_GND ((1*65536)/1000)
#define P_STAND ((5*65536)/1000) // leg proportional gain for standing
#define D_STAND ((4*65536)/1000)

uint32_t GAINS_AIR = (P_AIR<<16)+D_AIR;
uint32_t GAINS_GND = (P_GND<<16)+D_GND;
uint32_t GAINS_STAND = (P_STAND<<16)+D_STAND;


void setControlFlag(char state){
    controlFlag = state;
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

    if (onboardMode & 0b1111){ // use only gyro integration
        return;
    }

    updateEuler(vicon_angle,body_lag_sum,1); // attempt to counter comms lag

    for (i=0; i<3; i++){
        vicon_angle[i] = new_vicon_angle[i];
        body_angle[i] = 3*(body_angle[i] >> 2) + (new_vicon_angle[i] >> 2);
    }
}

void setVelocitySetpoint(int16_t* new_vel_des, long new_yaw) {
    int i;

    new_vel_des[0] = new_vel_des[0] > 3000 ? 3000 :
                     new_vel_des[0] < -3000 ? -3000 :
                     new_vel_des[0];
    new_vel_des[1] = new_vel_des[1] > 3000 ? 3000 :
                     new_vel_des[1] < -3000 ? -3000 :
                     new_vel_des[1];

    for (i=0; i<3; i++){
        vel_des[i] = new_vel_des[i];
    }
    yawSetpoint = new_yaw;
}

void resetBodyAngle(){
    //mpuRunCalib(0,100); //re-offset gyro, assumes stationary
    body_angle[0] = 0;
    body_angle[1] = 0;
    body_angle[2] = 0;
}

void calibGyroBias(){
    mpuRunCalib(0,5); //re-offset gyro, assumes stationary
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

    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16); // set BLDC angle offset
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, BLDC_CALIB, 16);
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, (3*65536/4), 17); // set BLDC max PWM (65536=100%)
    delay_ms(10);
    send_command_packet(&uart_tx_packet_global, 0, (3*65536/4), 17);

}


///////        Tail control ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;
    int i;

    // Do signal processing on gyro
    mpuGetGyro(gdata); // This should be the only call to mpuGetGyro(gdata)
    orientImageProc(body_velocity, gdata); // orient gyro readings gdata into body frame body_velocity
    for (i=0; i<3; i++) {
        // Low pass the gyro signal
        body_vel_LP[i] = ((256-BVLP_ALPHA)*body_vel_LP[i] + BVLP_ALPHA*body_velocity[i])>>8;

        // Keep a circular buffer of gyro readings for compensating mocap lag
        body_lag_sum[i] += (-body_lag_log[BLL_ind][i] + body_velocity[i]);
        body_lag_log[BLL_ind][i] = body_velocity[i];

        // Sum the gyro readings from the most recent two cycles for updateEuler
        body_vel_500[i] = body_velocity[i] + body_lag_log[BLL_ind_prev][i];
    }
    BLL_ind_prev = BLL_ind;
    BLL_ind = (BLL_ind+1)%VICON_LAG; // Circular buffer index

    // Processes to run less frequently -------------------
    // Estimators and planning below swap off on odd and even counts
    if(interrupt_count % 2) { // odd
        updateEuler(body_angle,body_vel_500,1); // attitude integration
    } else { // even
        // Tail estimation
        tail_pos = calibPos(1);
        tail_vel = (((128-TAIL_ALPHA)*tail_vel) >> 7) + ((TAIL_ALPHA*(tail_pos - tail_prev) >> 1) >> 7);
        // difference >> 1 (divide by 2) because it updates at 500Hz instead of 1kHz now
        tail_prev = tail_pos;

        // Additional 1kHz attitude estimator actions for toe balance
        if (onboardMode & 0b110) { // balance CG above toe and zero attitude
            body_angle[1] += ((pidObjs[2].output>>7) + (pidObjs[3].output>>7))<<1;
            body_angle[2] += (pidObjs[0].output>>5) << 1;
            // << 1 because it updates at 500Hz instead of 1kHz now
        } else if (onboardMode == 0b1000) { // onboard hopping control
            pitchSetpoint = x_ctrl;
            rollSetpoint = y_ctrl;
            if(att_correction[0] != 0 || att_correction[1] != 0) { // gyro anti-drift
                body_angle[1] -= att_correction[1];
                body_angle[2] -= att_correction[0];
                att_correction[0] = 0;
                att_correction[1] = 0;
            }
        }

        femurLUTs(); // calculate crank angle, foot distance, and MA
        updateVelocity(2); // leg length estimation
        multiJumpFlow(); // state machine
    }

    if(interrupt_count == 2) {
        raibert(); // onboard velocity control
    }

    if(interrupt_count == 4) { // reset interrupt_count after 4 counts
        interrupt_count = 0;

        if(controlFlag == 0){ // Control/motor off
            pidObjs[0].onoff = 0;
            pidObjs[2].onoff = 0;
            pidObjs[3].onoff = 0;
        } else { // Control pitch, roll, and yaw
            if (onboardMode & 0b110) { // standing on the ground
                tiHChangeMode(1, TIH_MODE_COAST);
                pidObjs[0].mode = 2;
            } else { // normal jumping operation
                if (mj_state == MJ_AIR) { // in the air
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
            setControlFlag(1);
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
                if (onboardMode & 0b100) { // static standing on ground
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_STAND, 2);
                } else { // normal operation
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_AIR, 2);
                }
            }
            break;

        case MJ_GND:
            // Liftoff transition from ground to air
            if (t1_ticks - transition_time > 50 && footTakeoff() == 1) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            } else { // remain in ground state
                if (onboardMode & 0b100) { // static standing on ground
                    send_command_packet(&uart_tx_packet_global, legSetpoint, GAINS_STAND, 2);
                } else { // normal operation
                    send_command_packet(&uart_tx_packet_global, pushoffCmd, GAINS_GND, 2);
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

char last_contact = 0;
char footContact(void) {
    int eps = 2000;
    unsigned int mot;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position*motPos_to_femur_crank_units); //UNITFIX
    char this_contact = mot-BLDC_MOTOR_OFFSET>crank && (mot-BLDC_MOTOR_OFFSET - eps) > crank;
    char contact = this_contact && last_contact;
    last_contact = this_contact;
    return contact;
}

char footTakeoff(void) {
    int eps = 1000;
    unsigned int mot;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position*motPos_to_femur_crank_units); //UNITFIX
    return ((mot-BLDC_MOTOR_OFFSET + eps) < crank) || calibPos(2) > FULL_EXTENSION;
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

void femurLUTs(void) { 
    // calculate crank, foot, and MA
    uint32_t femur;//, femurInd, femurDelta;

    /*
    // Linearly interpolate between lookup table entries
    femur = calibPos(2);
    femurInd = femur / 64; // Scale position to 8 bits
    femurDelta = femur % 64;

    if(femurInd>254){femurInd = 254;}
    if(femurInd<0){femurInd = 0;}

    crank = ((64-femurDelta)*(uint32_t)crank_femur_256lut[femurInd] + 
        femurDelta*(uint32_t)crank_femur_256lut[femurInd+1]) >> 6;
    foot = ((64-femurDelta)*(uint32_t)leg_femur_256lut[femurInd] + 
        femurDelta*(uint32_t)leg_femur_256lut[femurInd+1]) >> 6;
    MA = ((64-femurDelta)*(uint32_t)MA_femur_256lut[femurInd] + 
        femurDelta*(uint32_t)MA_femur_256lut[femurInd+1]) >> 6;
    */

    //*
    // Round down to nearest lookup table entry
    femur = calibPos(2) / 64; // Scale position to 8 bits
    if(femur>255 || femur < 0){return;} // Femur reading is bad: out of physical range

    crank = crank_femur_256lut[femur];
    foot = leg_femur_256lut[femur];
    MA = MA_femur_256lut[femur];
    //*/
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

    sin_theta = cosApprox(temp_angle[2]-PI/2);
    cos_theta = cosApprox(temp_angle[2]);
    sin_phi = cosApprox(temp_angle[1]-PI/2);
    cos_phi = cosApprox(temp_angle[1]);

    // Prevent divide by zero
    if (cos_phi == 0) { cos_phi = 1; }

    // Update Euler angles
    temp_angle[2] += (((sin_phi*sin_theta/cos_phi)*vels[1]
            + (vels[2] << COS_PREC)
            - (cos_theta*sin_phi/cos_phi)*vels[0])*time) >> COS_PREC;
    temp_angle[1] += (cos_theta*vels[1] + sin_theta*vels[0])*time >> COS_PREC;
    temp_angle[0] += ((-sin_theta*vels[1])/cos_phi + (cos_theta*vels[0])/cos_phi)*time;

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

// Pass these estimates to takeoff_est.c for velocity estimation on takeoff
unsigned char TOcompFlag = 0;
unsigned char g_accumulator = 0;
int16_t TOleg;
int16_t TOlegVel;
long TObody_angle[3];
long TObody_vel_LP[3];

unsigned char last_state = MJ_IDLE;
#define VEL_BUF_LEN 10  // Buffer to find peak velocity at takeoff
unsigned char vel_ind = 0;
int16_t legBuf[VEL_BUF_LEN*2];
long angBuf[VEL_BUF_LEN*6];
int32_t last_mot;

#define GRAV_ACC 39 // -9.81 m/s * (2^2 ticks / m/s^2)

void updateVelocity(long time) {
    // Update the onboard leg velocity estiamtes
    unsigned char i, j;
    char cntr;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    int32_t mot = sensor_data->position*motPos_to_femur_crank_units; //UNITFIX
    if (mot - last_mot > 1<<14 || last_mot - mot > 1<<14) {
        mot = last_mot; // reject bad samples
    }
    last_mot = mot;

    int32_t spring_def = mot - crank;
    if(spring_def < 0){spring_def=0;}

    spring = SPRING_LINEAR*spring_def -
        (SPRING_QUADRATIC*((spring_def*spring_def) >> 14));
    force = ((MA)*(spring) >> 13);

    force -= ((legVel>0?1:-1)*LEG_FRICTION*force/1000); // LEG_FRICTION/1000
    // sensor_data->position is 1 rad / 2^16 ticks (through a 25 to 1 gear ratio)
    // crank and spring_def are 4 rad / 2^16 tick, or 1 rad / 2^14 ticks
    // spring is 1 Nm / 2^14 ticks
    // MA is 1 N/Nm / 2^9 ticks
    // force is 1 N / (2^10 ticks)

    // Update leg and leg velocity estimates
    int32_t legErr;
    if (mj_state == MJ_GND) { // Stance phase estimation

        if (last_state == MJ_AIR) { // The robot just touched down
            for (j=0; j<3; j++) {
                stance_vel_des[j] = vel_des[j]; // Save the desired velocities
            }
        }

        legErr = (foot>>2) - leg;
        if (legErr > 1000) {legErr = 1000;}
        if (legErr < -1000) {legErr = -1000;}
        leg += legVel/31 * time + 3*(legErr >> 2);
        // leg is in 1 m / 2^16 ticks
        // conversion from legVel*time to leg is 1000000*2/2^16 or about 31
        legVel += (((-GRAV_ACC + force/FULL_MASS) * time) >> 1) + (legErr << 1);
        // legVel is in 1 m/s / (1000*2 ticks)
        // acceleration is in m/s^2 / (2^2 ticks)

        velocity[0] = 0; // zero velocities; not really necessary
        velocity[1] = 0;
    } else if (mj_state == MJ_AIR) { // Flight phase estimation

        if (last_state == MJ_GND) { // robot thinks it just took off
            // save the pose and velocity states for calculating takeoff velocities
            // first, check if takeoff actually happened earlier
            cntr = vel_ind;
            TOlegVel = legVel;
            for (i=0; i<VEL_BUF_LEN; i++) {
                if (legBuf[cntr*2+1] - ((GRAV_ACC*i*time) >> 1) > legVel) { // TODO: time assumed constant
                    legVel = legBuf[cntr*2+1] - ((GRAV_ACC*i*time) >> 1) ; // take max velocity as takeoff velocity

                    TOlegVel = legBuf[cntr*2+1];
                    g_accumulator = i*time; // set the accumulator to the number of steps elapsed

                    TOleg = legBuf[cntr*2];
                    for (j=0; j<3; j++) {
                        TObody_angle[j] = angBuf[cntr*6+j];
                        TObody_vel_LP[j] = angBuf[cntr*6+3+j];
                    }
                }
                cntr = cntr-1;
                if (cntr < 0) {cntr = VEL_BUF_LEN - 1;}
            }
            legVel += 700; // takeoff boost of 0.35 m/s
            TOlegVel += 700;

            velocity[2] = -velocity[2]; // velocity estimate until real calculation is done
            TOcompFlag = 1; // tell the main loop to calculate the takeoff velocities
        }

        leg = foot >> 2;
        legVel -= (GRAV_ACC*time) >> 1; // gravitational acceleration

        g_accumulator+=time;
        if (!TOcompFlag) {
            velocity[2] -= (GRAV_ACC*g_accumulator) >> 1; // gravitational acceleration
            g_accumulator = 0;
        }

    } else {
        leg = foot >> 2;
        legVel = 0;
    }

    if (legVel > 20000) { legVel = 20000;}
    if (legVel < -20000) { legVel = -20000;}
    if (velocity[2] > 20000) { velocity[2] = 20000;}
    if (velocity[2] < -20000) { velocity[2] = -20000;}

    // Save the recent velocities to check takeoff
    legBuf[vel_ind*2] = leg;
    legBuf[vel_ind*2+1] = legVel;
    for (j=0; j<3; j++){
        angBuf[vel_ind*6+j] = body_angle[j];
        angBuf[vel_ind*6+3+j] = body_vel_LP[j];
    }
    vel_ind = (vel_ind+1)%VEL_BUF_LEN;

    last_state = mj_state;

}


void raibert() {
    x_ctrl = (( -((T_STEP*(long)velocity[0])>>1) + K_RAIBERT*((long)vel_des[0] - (long)velocity[0]) )
        *INVERSE_LEG_LEN)>>CTRL_CONVERT;
    y_ctrl = (( ((T_STEP*(long)velocity[1])>>1) - K_RAIBERT*((long)vel_des[1] - (long)velocity[1]) )
        *INVERSE_LEG_LEN)>>CTRL_CONVERT;
    x_ctrl = x_ctrl > PI/6 ? PI/6 :
             x_ctrl < -PI/6 ? -PI/6 :
             x_ctrl;
    y_ctrl = y_ctrl > PI/6 ? PI/6 :
             y_ctrl < -PI/6 ? -PI/6 :
             y_ctrl;
}


long cosApprox(long x) {
    // Cosine approximation by Bhaskara I's method:
    // https://en.wikipedia.org/wiki/Bhaskara_I%27s_sine_approximation_formula
    //
    // Returns a value between -2^COS_PREC and 2^COS_PREC
    // INPUTS:
    // x: angle scaled by 16384 ticks per degree (2000 deg/s MPU gyro integrated @ 1kHz).

    long xSquared, out;
    if (x < 0) { x = -x; } // cosine is even
    x = (x+PI) % (PI<<1) - PI;
    if (x < 0) { x = -x; } // cosine is even
    if (x > (PI>>1)) { // quadrants 2 and 3
        x = (x - PI) >> 8;
        xSquared = x*x;
        out = -(PISQUARED - (xSquared<<2))/((PISQUARED + xSquared)>>COS_PREC);
    } else {
        x = x >> 8; // quadrants 1 and 4
        xSquared = x*x;
        out = (PISQUARED - (xSquared<<2))/((PISQUARED + xSquared)>>COS_PREC);
    }
    return out;
}
