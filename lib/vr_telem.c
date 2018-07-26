
// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.c , OctoRoACH specific telemetry packet format




#include <xc.h>
#include "vr_telem.h"
#include "ams-enc.h"
#include "mpu6000.h"
#include "adc_pid.h"
#include "tih.h"
#include "pid-ip2.5.h"
#include "protocol.h"
#include "utils.h"

// TODO (apullin) : Remove externs by adding getters to other modules
//extern pidObj motor_pidObjs[NUM_MOTOR_PIDS];
//extern int bemf[NUM_MOTOR_PIDS];

//externs added back in for VR telem porting (pullin 10/9/14)
extern int bemf[NUM_PIDS];
extern pidPos pidObjs[NUM_PIDS];

extern packet_union_t* last_bldc_packet;
extern uint8_t last_bldc_packet_is_new;

extern long tail_vel;
extern int gdata[3];

extern unsigned char mj_state;
extern char controlFlag;
extern char onboardMode;
extern int32_t crank;
extern int32_t foot;
extern int32_t MA;
extern int32_t force;
extern int16_t leg;
extern int32_t legVel;
extern int16_t velocity[3];
extern long body_vel_LP[3];

extern long x_ctrl;
extern long y_ctrl;

//void vrTelemGetData(unsigned char* ptr) {
void vrTelemGetData(vrTelemStruct_t* ptr) {
    
    //vrTelemStruct_t* tptr;
    //tptr = (vrTelemStruct_t*) ptr;

    //int gdata[3];   //gyrodata
    int xldata[3];  // accelerometer data
    /////// Get XL data
    //mpuGetGyro(gdata); // this is read in salto_ctrl.c
    mpuGetXl(xldata);

    //Motion control
    ptr->posTail = (long)(encPos[0].pos << 2) + (encPos[0].oticks << 16);
    ptr->posFemur = -(((long)encPos[1].pos - (long)encPos[1].offset) << 2) - (encPos[1].oticks << 16);
    
    ptr->pitch = pidObjs[0].p_state;
    ptr->roll = pidObjs[2].p_state;
    ptr->yaw = pidObjs[3].p_state;
    ptr->pitchSet = tail_vel;//pidObjs[0].p_input + pidObjs[0].interpolate;
    ptr->dcTail = pidObjs[0].output; // left

    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    ptr->posMotor = sensor_data->position;
    ptr->dcBLDC = sensor_data->current;
    last_bldc_packet_is_new = 0;
    LED_3 = 0;

    ptr->dcProp1 = pidObjs[2].output; // Rear
    ptr->dcProp2 = pidObjs[3].output; // Fore

    //gyro and XL
    //*
    ptr->gyroX = gdata[0];
    ptr->gyroY = gdata[1];
    ptr->gyroZ = gdata[2];
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
    ptr->otherMode = 4;
    ptr->onboardMode = mj_state + (controlFlag <<7) + (onboardMode << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = force;
    ptr->foot = leg;
    ptr->footVel = legVel;
    */
    /*
    // leg velocity
    ptr->otherMode = 5;
    ptr->onboardMode = mj_state + (controlFlag <<7) + (onboardMode << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = velocity[0];
    ptr->foot = velocity[1];
    ptr->footVel = velocity[2];
    */
    //*
    // onboard velocity control
    ptr->otherMode = 7;
    ptr->onboardMode = mj_state + (controlFlag <<7) + (onboardMode << 8);
    ptr->voltage = sensor_data->voltage;
    ptr->crank = crank;
    ptr->force = x_ctrl;
    ptr->foot = y_ctrl;
    ptr->footVel = velocity[2];
    //*/

}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}
