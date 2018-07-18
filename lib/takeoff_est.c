
#include "takeoff_est.h"
#include "salto_ctrl.h"


extern int16_t TOleg;
extern int16_t TOlegVel;
extern long TObody_angle[3];
extern long TObody_vel_LP[3];

extern unsigned char TOcompFlag;
extern int16_t velocity[3];


void takeoffEstimation(){
    //Calculate estimated velocities on takeoff

    long TOcos_theta = cosApprox(TObody_angle[2]);
    long TOsin_theta = cosApprox(TObody_angle[2]-PI/2);
    long TOcos_phi = cosApprox(TObody_angle[1]);
    long TOsin_phi = cosApprox(TObody_angle[1]-PI/2);
    long TOcos_psi = cosApprox(TObody_angle[0]);
    long TOsin_psi = cosApprox(TObody_angle[0]-PI/2);

    // Compensate for CG offset
    TObody_vel_LP[2] -= 0.3*4/4*TOlegVel*0.469; // (2^15/2000*180/pi)/2000 = 0.4694
    TObody_vel_LP[1] -= 0.2*4/4*TOlegVel*0.469;

    // Body velocity rotation matrix
    int32_t vxw = (int32_t)TOleg*(int32_t)TObody_vel_LP[2]/30760;
    int32_t vyw = -(int32_t)TOleg*(int32_t)TObody_vel_LP[1]/30760;
    // native units (body_vel_LP*leg: 2000/2^15 (deg/s)/tick * 1/2^16 m/ticks * pi/180 rad/deg
    // final units (legVel): 1/2000 (m/s)/tick
    // unit conversion: 1/30760.437 tick/tick

    velocity[0] = (((((TOcos_psi*TOcos_theta) >> COS_PREC)
            - ((((TOsin_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        - ((((TOcos_phi*TOsin_psi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOcos_psi*TOsin_theta) >> COS_PREC) 
            + ((((TOcos_theta*TOsin_psi) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
    velocity[1] = (((((TOcos_theta*TOsin_psi) >> COS_PREC)
            + ((((TOcos_psi*TOsin_phi) >> COS_PREC)*TOsin_theta) >> COS_PREC))*vxw) >> COS_PREC)
        + ((((TOcos_psi*TOcos_phi) >> COS_PREC)*vyw) >> COS_PREC)
        + (((((TOsin_psi*TOsin_theta) >> COS_PREC)
            - ((((TOcos_psi*TOcos_theta) >> COS_PREC)*TOsin_phi) >> COS_PREC))*TOlegVel) >> COS_PREC);
    velocity[2] = - ((((TOcos_phi*TOsin_theta) >> COS_PREC)*vxw) >> COS_PREC)
        + ((TOsin_phi*vyw) >> COS_PREC)
        + ((((TOcos_theta*TOcos_phi) >> COS_PREC)*TOlegVel) >> COS_PREC);
    //Z1X2Y3 https://en.wikipedia.org/wiki/Euler_angles

    TOcompFlag = 0;
}