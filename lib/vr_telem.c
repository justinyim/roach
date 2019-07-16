
// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.c , OctoRoACH specific telemetry packet format




#include <xc.h>
#include "vr_telem.h"
#include "ams-enc.h"
#include "mpu6000.h"
#include "adc_pid.h"
#include "tih.h"
#include "salto1p.h"
#include "protocol.h"
#include "utils.h"
#include "settings.h"

extern packet_union_t* last_bldc_packet;
extern uint8_t last_bldc_packet_is_new;

extern int32_t tail_pos;
extern int32_t tail_vel;
extern int16_t gdata[3];
extern int16_t xldata[3];

extern unsigned char mj_state;
extern char running;
extern uint8_t modeFlags;
extern int32_t femur;
extern int32_t crank;
extern int32_t foot;
extern int32_t MA;
extern int32_t force;
extern int16_t leg;
extern int32_t legVel;
extern int32_t q[3];
extern int16_t v[3];
extern int32_t w[3];
extern int32_t p[3];

extern int16_t foreThruster;
extern int16_t aftThruster;
extern int16_t tailMotor;

extern uint32_t ctrlCount;
extern uint32_t t1_ticks;

extern int16_t foreVel;
extern int16_t aftVel;
extern int32_t q0offset;
extern int32_t q1offset;


//extern long x_ctrl;
//extern long y_ctrl;
//extern int16_t vel_des[3];
//extern long att_correction[2];
extern int32_t qCmd[3];
extern int32_t TOw[3];

//void vrTelemGetData(unsigned char* ptr) {
void vrTelemGetData(vrTelemStruct_t* ptr) {

    uint8_t running = mj_state != MJ_STOP && mj_state != MJ_STOPPED && mj_state != MJ_IDLE;
    
    //vrTelemStruct_t* tptr;
    //tptr = (vrTelemStruct_t*) ptr;

    //Motion control
    ptr->posTail = tail_pos;
    ptr->posFemur = femur;
    
    ptr->pitch = q[1];//pidObjs[0].p_state;
    ptr->roll = q[0];//pidObjs[2].p_state;
    ptr->yaw = q[2];//pidObjs[3].p_state;
    ptr->pitchSet = tail_vel;//pidObjs[0].p_input + pidObjs[0].interpolate;
    ptr->dcTail = tailMotor;//pidObjs[0].output; // left

    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    ptr->posMotor = sensor_data->position - BLDC_MOTOR_OFFSET;
    ptr->dcBLDC = sensor_data->current;
    last_bldc_packet_is_new = 0;

    ptr->dcProp1 = foreThruster;//pidObjs[2].output; // Rear
    ptr->dcProp2 = aftThruster;//pidObjs[3].output; // Fore

    //gyro and XL
    //*
    ptr->gyroX = w[0];//gdata[0];
    ptr->gyroY = w[1];//gdata[1];
    ptr->gyroZ = w[2];//gdata[2];
    ptr->accelX = xldata[0];
    ptr->accelY = xldata[1];
    ptr->accelZ = xldata[2];
    //*/
    /*
    //ptr->otherMode = 6; // onboard body velocities and lowpass angular velocity in place of accels
    ptr->gyroX = gdata[0];
    ptr->gyroY = gdata[1];
    ptr->gyroZ = gdata[2];
    ptr->accelX = body_vel_LP[0];
    ptr->accelY = body_vel_LP[1];
    ptr->accelZ = body_vel_LP[2];
    */


    /*
    // leg dynamics
    ptr->otherMode = 4;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = force;
    ptr->foot = leg;
    ptr->footVel = legVel;
    */
    //*
    // body velocity
    ptr->otherMode = 5;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = v[0];
    ptr->foot = v[1];
    ptr->footVel = v[2];
    //*/
    /*
    // onboard velocity control
    ptr->otherMode = 7;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = x_ctrl/90;
    ptr->foot = y_ctrl/90;
    ptr->footVel = velocity[2];
    */
    /*
    // onboard velocity control
    ptr->otherMode = 8;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = qCmd[1]/90;
    ptr->foot = qCmd[0]/90;
    ptr->footVel = legVel;
    ptr->accelX = v[0];
    ptr->accelY = v[1];
    ptr->accelZ = v[2];
    //ptr->accelX = vCmd[0];
    //ptr->accelY = vCmd[1];
    //ptr->accelZ = vCmd[2];
    */
    /*
    // onboard velocity control
    ptr->otherMode = 9;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = velocity[0];
    ptr->foot = velocity[1];
    ptr->footVel = velocity[2];
    ptr->accelX = vel_des[0];
    ptr->accelY = vel_des[1];
    ptr->accelZ = vel_des[2];
    */
    /*
    // leg and body velocity
    ptr->otherMode = 10;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->accelX = TOw[0];
    ptr->accelY = TOw[1];
    ptr->accelZ = legVel;
    ptr->force = v[0];
    ptr->foot = v[1];
    ptr->footVel = v[2];
    */
    /*
    // body velocity and attitude corrections
    ptr->otherMode = 11;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->accelX = att_correction[0];
    ptr->accelY = att_correction[1];
    ptr->accelZ = legVel;
    ptr->force = velocity[0];
    ptr->foot = velocity[1];
    ptr->footVel = velocity[2];
    */
    /*
    // robot position
    ptr->otherMode = 12;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->accelX = robot_pos[0]/100;
    ptr->accelY = robot_pos[1]/100;
    ptr->accelZ = robot_pos[2]/100;
    ptr->force = velocity[0];
    ptr->foot = velocity[1];
    ptr->footVel = velocity[2];
    */
    /*
    // body velocity
    ptr->otherMode = 13;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = force;
    ptr->crank = crank;
    ptr->force = velocity[0];
    ptr->foot = velocity[1];
    ptr->footVel = velocity[2];
    */
    /*
    // BEMF on thrusters
    ptr->otherMode = 14;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = foreVel;
    ptr->foot = aftVel;
    ptr->footVel = adcGetVbatt();
    */
    /*
    // Balance offset correction
    ptr->otherMode = 15;
    ptr->onboardMode = mj_state + (running <<7) + (modeFlags << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = q0offset;
    ptr->foot = q1offset;
    ptr->footVel = t1_ticks;
    */
}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}
