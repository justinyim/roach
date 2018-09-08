
#include "takeoff_est.h"
#include "salto_ctrl.h"


extern int16_t TOleg;
extern int16_t TOlegVel;
extern long TObody_angle[3];
extern long TObody_vel_LP[3];

extern long TDbody_angle[3];
extern int16_t TDvelocity[3];
extern long TDangle_setpoint[3];

extern unsigned char TOcompFlag;
extern int16_t velocity[3];
extern char ac_flag;

volatile extern int16_t stance_vel_des[3];
volatile extern long att_correction[2];

void takeoffEstimation(){
    //Calculate estimated velocities on takeoff

    long TOcos_theta = cosApprox(TObody_angle[2]);
    long TOsin_theta = cosApprox(TObody_angle[2]-PI/2);
    long TOcos_phi = cosApprox(TObody_angle[1]);
    long TOsin_phi = cosApprox(TObody_angle[1]-PI/2);
    long TOcos_psi = cosApprox(TObody_angle[0]);
    long TOsin_psi = cosApprox(TObody_angle[0]-PI/2);

    // Compensate for CG offset
    TObody_vel_LP[2] -= 0.20*0.469*TOlegVel; // (2^15/2000*180/pi)/2000 = 0.4694
    TObody_vel_LP[1] -= -0.10*0.469*TOlegVel;
    //TObody_vel_LP[2] -= 0.30*0.469*TOlegVel; // (2^15/2000*180/pi)/2000 = 0.4694
    //TObody_vel_LP[1] -= -0.20*0.469*TOlegVel;

    // Body velocity rotation matrix
    int32_t vxw = 14418*(int32_t)TObody_vel_LP[2]/30760; // locking TOleg at 0.22m
    int32_t vyw = -14418*(int32_t)TObody_vel_LP[1]/30760;
    //int32_t vxw = (int32_t)TOleg*(int32_t)TObody_vel_LP[2]/30760;
    //int32_t vyw = -(int32_t)TOleg*(int32_t)TObody_vel_LP[1]/30760;
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

    velY = 17*velY/20;

    velocity[0] = velX;
    velocity[1] = velY;
    velocity[2] = velZ;

    int i;
    for (i=1; i<3; i++) {
        TDbody_angle[i] -= TDangle_setpoint[i]; // calculate angle error
    }

    /*
    // Deadbeat-based correction
    long predicted_angles[3];
    deadbeat(TDvelocity, velocity, predicted_angles);
    att_correction[0] = (TDbody_angle[2] - predicted_angles[0])>>2;
    att_correction[1] = (TDbody_angle[1] - predicted_angles[1])>>2;
    */

    //*
    // Velocity-based correction
    long velocity_x = (velocity[0]*TOcos_psi + velocity[1]*TOsin_psi)>>COS_PREC;
    long velocity_y = (-velocity[0]*TOsin_psi + velocity[1]*TOcos_psi)>>COS_PREC;

    att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - stance_vel_des[0]) - (TDbody_angle[2]>>1));
    att_correction[1] = ATT_CORRECTION_GAIN_Y*(velocity_y - stance_vel_des[1]) + (TDbody_angle[1]>>1);
    //*/

    /*
    att_correction[0] = -(ATT_CORRECTION_GAIN_X*(velocity_x - stance_vel_des[0]) * 188) /
        ((long)(-velocity[2] + TDvelocity[2]) >> 6);
    att_correction[1] = (ATT_CORRECTION_GAIN_Y*(velocity_y - stance_vel_des[1]) * 188) /
        ((long)(-velocity[2] + TDvelocity[2]) >> 6); // (3*2000 >> 6)*2 is about 188
    */

    att_correction[0] = att_correction[0] > 15000 ? 15000 :
                        att_correction[0] < -15000 ? -15000 :
                        att_correction[0];
    att_correction[1] = att_correction[1] > 15000 ? 15000 :
                        att_correction[1] < -15000 ? -15000 :
                        att_correction[1];
    ac_flag = 1;

    TOcompFlag = 0;
}