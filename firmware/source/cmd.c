#include "cmd.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "sclock.h"
#include "led.h"
#include "blink.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "radio.h"
#include "dfmem.h"
#include "tests.h"
#include "version.h"
#include "settings.h"
#include "timer.h"
#include "tih.h"
#include "ams-enc.h"
#include "carray.h"
#include "telem.h"
#include "uart_driver.h"
#include "protocol.h"
#include "salto1p.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char (*cmd_func[MAX_CMD_FUNC])(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned int);
void cmdError(void);

//extern pidPos pidObjs[NUM_PIDS];
extern EncObj encPos[NUM_ENC];
extern EncObj motPos;
extern volatile CircArray fun_queue;

packet_union_t uart_tx_packet_cmd;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdGetAMSPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//Jumper functions
static unsigned char cmdSetPitchSetpoint(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdResetBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMotorPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStartExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetExperimentParams(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdIntegratedVicon(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStopExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdCalibrateMotor(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdOnboardMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdGyroBias(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdGVectAtt(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetVelocity(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdAdjustBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdTilt(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStance(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMocapVel(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdLaunch(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//Motor and PID functions
static unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
//Experiment/Flash Commands
static unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC; ++i) {
        cmd_func[i] = &cmdNop;
    }
    cmd_func[CMD_TEST_RADIO] = &test_radio;
    cmd_func[CMD_TEST_MPU] = &test_mpu;
    cmd_func[CMD_SET_THRUST_OPEN_LOOP] = &cmdSetThrustOpenLoop;
    cmd_func[CMD_SET_MOTOR_MODE] = &cmdSetMotorMode;
    cmd_func[CMD_PID_START_MOTORS] = &cmdPIDStartMotors;
    cmd_func[CMD_SET_PID_GAINS] = &cmdSetPIDGains;
    cmd_func[CMD_GET_AMS_POS] = &cmdGetAMSPos;
    cmd_func[CMD_START_TELEMETRY] = &cmdStartTelemetry;
    cmd_func[CMD_ERASE_SECTORS] = &cmdEraseSectors;
    cmd_func[CMD_FLASH_READBACK] = &cmdFlashReadback;
    cmd_func[CMD_SET_VEL_PROFILE] = &cmdSetVelProfile;
    cmd_func[CMD_WHO_AM_I] = &cmdWhoAmI;
    cmd_func[CMD_ZERO_POS] = &cmdZeroPos;   
    cmd_func[CMD_START_TIMED_RUN] = &cmdStartTimedRun;
    cmd_func[CMD_PID_STOP_MOTORS] = &cmdPIDStopMotors;

    cmd_func[CMD_SET_PITCH_SET] = &cmdSetPitchSetpoint;
    cmd_func[CMD_RESET_BODY_ANG] = &cmdResetBodyAngle;
    cmd_func[CMD_SET_MOTOR_POS] = &cmdSetMotorPos;
    cmd_func[CMD_START_EXP] = &cmdStartExperiment;
    cmd_func[CMD_SET_EXP_PARAMS] = &cmdSetExperimentParams;
    cmd_func[CMD_INTEGRATED_VICON] = &cmdIntegratedVicon;
    cmd_func[CMD_STOP_EXP] = &cmdStopExperiment;
    cmd_func[CMD_CALIBRATE_MOTOR] = &cmdCalibrateMotor;
    cmd_func[CMD_ONBOARD_MODE] = &cmdOnboardMode;
    cmd_func[CMD_GYRO_BIAS] = &cmdGyroBias;
    cmd_func[CMD_G_VECT_ATT] = &cmdGVectAtt;
    cmd_func[CMD_SET_VELOCITY] = &cmdSetVelocity;
    cmd_func[CMD_ADJUST_BODY_ANG] = &cmdAdjustBodyAngle;
    cmd_func[CMD_TILT] = &cmdTilt;
    cmd_func[CMD_STANCE] = &cmdStance;
    cmd_func[CMD_SET_MOCAP_VEL] = &cmdSetMocapVel;
    cmd_func[CMD_LAUNCH] = &cmdLaunch;
}

void cmdPushFunc(MacPacket rx_packet) {
    Payload rx_payload;
    unsigned char command;

    rx_payload = macGetPayload(rx_packet);
    if(rx_payload != NULL) {
        command = payGetType(rx_payload);

        if(command < MAX_CMD_FUNC && cmd_func[command] != NULL) {
            rx_payload->test = cmd_func[command];
            carrayAddTail(fun_queue, rx_packet);
        } else {
            cmdError();   // halt on error - could also just ignore....
        }
    }
}


// send robot info when queried
unsigned char cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    unsigned char i, string_length; unsigned char *version_string;
    // maximum string length to avoid packet size limit
    version_string = (unsigned char*)versionGetString();
    i = 0;
    while((i < 127) && version_string[i] != '\0') {
        i++;
    }
    string_length=i;
    radioSendData(src_addr, status, CMD_WHO_AM_I, //TODO: Robot should respond to source of query, not hardcoded address
            string_length, version_string, 0);
    return 1; //success
}

unsigned char cmdGetAMSPos(unsigned char type, unsigned char status,
        unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    long motor_count[2];
    motor_count[0] = pidObjs[0].p_state;
    motor_count[1] = pidObjs[1].p_state;

    // motor_count[0] = encPos[0].pos;
    // motor_count[1] = encPos[1].pos;

    radioSendData(src_addr, status, CMD_GET_AMS_POS,  //TODO: Robot should respond to source of query, not hardcoded address
            sizeof(motor_count), (unsigned char *)motor_count, 0);
    */
    // pid-ip2.5.c removed
    return 1;
}
// ==== Jumper Commands ==============================================================================
// =============================================================================================================


unsigned char cmdStartExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    uint8_t mode = frame[0];
    expStart(mode);
    return 1;
}

unsigned char cmdSetExperimentParams(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    /*
    int32_t mode = frame[0];

    switch(mode) {
        case EXP_WALL_JUMP: ;
            int32_t params[5];
            int i = 0;
            int j = 0;
            int idx;
            for (i = 0; i < 5; i=i+1)
            {
                params[i] = 0;
                for (j = 0; j < 4; j=j+1)
                {
                    idx = j+4*i+4;
                    params[i] += ((long)frame[idx] << 8*j );
                }
            }
            exp_set_params_wj(params[0],params[1],params[2],params[3],params[4]);
            break;
        case EXP_SINGLE_JUMP: ;
            int16_t duration = frame[2] + (frame[3] << 8);
            int16_t leg_extension = frame[4] + (frame[5] << 8);
            int32_t conv_leg_extension;
            conv_leg_extension = (long)(leg_extension)*6554; //1/10 radian to 15.16 radians representation;
            exp_set_params_sj(duration, conv_leg_extension);
            break;
        default:
            break;
    }
    */
    // no longer used
    return 1;
}

unsigned char cmdSetPitchSetpoint(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    /*
    long pos = 0;
    int i;
    
    for (i = 0; i < 4; i++)
    {
        pos += ((long)frame[i] << 8*i );
    }
    setControlFlag(1);
    setAttitudeSetpoint(0,0,pos);
    */
    // no longer used
    return 1;
}

unsigned char cmdIntegratedVicon(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Receive Vicon-measured attitude (3), desired attitude (3), leg length (1), and push-off command (1)
    long new_vicon_angle[3];
    long new_setpoints[3];
    int i;
    for (i=0; i<3; i++){
        new_vicon_angle[i] = (int16_t)frame[2*i] + ((int16_t)frame[2*i+1] << 8);
        new_vicon_angle[i] = new_vicon_angle[i] << 8;
    }
    for (i=0; i<3; i++){
        new_setpoints[i] = (int16_t)frame[2*i+6] + ((int16_t)frame[2*i+7] << 8);
        new_setpoints[i] = new_setpoints[i] << 8;
    }
    long leg_length = (int16_t)frame[12] + ((int16_t)frame[13] << 8);
    long pushoff = (int16_t)frame[14] + ((int16_t)frame[15] << 8);

    updateBodyAngle(new_vicon_angle);
    setAttitudeSetpoint(new_setpoints[0],new_setpoints[1],new_setpoints[2]);
    setLegSetpoint(leg_length);
    setPushoffCmd(pushoff);

    return 1;
}

unsigned char cmdSetVelocity(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Set desired velocity to be tracked by onboard hopping control
    int16_t new_vel_des[3];
    long new_yaw;
    int i;
    for (i=0; i<3; i++){
        new_vel_des[i] = (int16_t)frame[2*i] + ((int16_t)frame[2*i+1] << 8);
    }
    new_yaw = ((long)frame[6] + ((long)frame[7] << 8)) << 8;
    setVelocitySetpoint(new_vel_des, new_yaw);

    return 1;
}

unsigned char cmdAdjustBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Set desired velocity to be tracked by onboard hopping control
    long body_adjust[3];
    int i;
    for (i=0; i<3; i++){
        body_adjust[i] = (int16_t)frame[2*i] + ((int16_t)frame[2*i+1] << 8);
        body_adjust[i] = body_adjust[i] << 8;
    }
    adjustBodyAngle(body_adjust);

    return 1;
}

unsigned char cmdStopExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Stop PID and leg
    expStop((int)frame[0]);
    return 1;
}

unsigned char cmdCalibrateMotor(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Calibrate BLDC motor driver encoder offset
#define N_SPEED 10
    int i;
    extern packet_union_t* last_bldc_packet;
    sensor_data_t* sensor_data;
    int32_t speed[N_SPEED];
    
    int32_t voltage_command = (frame[2] + (frame[3]<<8)) << 1;
    uint32_t calibration_point = frame[0] + (frame[1]<<8);

    send_command_packet(&uart_tx_packet_cmd, 0, calibration_point, 16);
    delay_ms(50);
    send_command_packet(&uart_tx_packet_cmd, voltage_command, 0, 3);
    delay_ms(600);
    for (i=0;i<N_SPEED;i++){
        sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
        speed[i] = sensor_data->velocity;
        delay_ms(10);
    }
        
    radioSendData(src_addr, status, CMD_CALIBRATE_MOTOR, 
        sizeof(speed), (unsigned char *)speed, 0);
    return 1;
}

unsigned char cmdOnboardMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    setOnboardMode((char)frame[0], (char)frame[1]);
    return 1;
}

unsigned char cmdGyroBias(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    calibGyroBias();
    return 1;
}

unsigned char cmdGVectAtt(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    accZeroAtt();
    return 1;
}

unsigned char cmdResetBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    long pitch = ((int32_t)frame[0] + ((int32_t)frame[1] << 8))  << 8;
    long body_angle[3] = {0, 0, pitch};
    setBodyAngle(body_angle);
    return 1;
}

unsigned char cmdSetMotorPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    long pos = ((long)frame[0] << 8) + ((long)frame[1]<<16);
    uint32_t p_gain = (uint32_t)frame[2] + ((uint32_t)frame[3]<<8);
    uint32_t d_gain = (uint32_t)frame[4] + ((uint32_t)frame[5]<<8);

    uint32_t gain = (p_gain<<16) + d_gain;
    setMotorPos(gain, pos);
    
    //send_command_packet(&uart_tx_packet_cmd, pos, gain, 2);

    return 1;
}

unsigned char cmdTilt(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    int16_t u = (int16_t)frame[0] + ((int16_t)frame[1]<<8);
    int16_t ud = (int16_t)frame[2] + ((int16_t)frame[3]<<8);
    int16_t udd = (int16_t)frame[4] + ((int16_t)frame[5]<<8);
    int16_t uddd = (int16_t)frame[6] + ((int16_t)frame[7]<<8);

    setTilt(u,ud,udd,uddd);
    return 1;
}

unsigned char cmdStance(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    int16_t u = (int16_t)frame[0] + ((int16_t)frame[1]<<8);
    int16_t ud = (int16_t)frame[2] + ((int16_t)frame[3]<<8);
    int16_t udd = (int16_t)frame[4] + ((int16_t)frame[5]<<8);
    int16_t uddd = (int16_t)frame[6] + ((int16_t)frame[7]<<8);
    
    int16_t r = (int16_t)frame[8] + ((int16_t)frame[9]<<8);
    int16_t rd = (int16_t)frame[10] + ((int16_t)frame[11]<<8);
    int16_t rdd = (int16_t)frame[12] + ((int16_t)frame[13]<<8);
    int16_t k1 = (int16_t)frame[14] + ((int16_t)frame[15]<<8);
    int16_t k2 = (int16_t)frame[16] + ((int16_t)frame[17]<<8);

    setTilt(u,ud,udd,uddd);
    setLeg(r,rd,rdd,k1,k2);
    return 1;
}

unsigned char cmdSetMocapVel(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    // Set desired velocity to be tracked by onboard hopping control
    int16_t new_vel[3];
    long new_yaw;
    int i;
    for (i=0; i<3; i++){
        new_vel[i] = (int16_t)frame[2*i] + ((int16_t)frame[2*i+1] << 8);
    }
    new_yaw = (int16_t)frame[6] + ((int16_t)frame[7] << 8);
    new_yaw = new_yaw << 8;
    setMocapVel(new_vel, new_yaw);

    return 1;
}

unsigned char cmdLaunch(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    int16_t u = (int16_t)frame[0] + ((int16_t)frame[1]<<8);
    int16_t ud = (int16_t)frame[2] + ((int16_t)frame[3]<<8);
    int16_t udd = (int16_t)frame[4] + ((int16_t)frame[5]<<8);
    int16_t uddd = (int16_t)frame[6] + ((int16_t)frame[7]<<8);
    
    int16_t rol = ((int16_t)frame[8] + ((int16_t)frame[9]<<8)) << 8;
    int16_t rdd = (int16_t)frame[10] + ((int16_t)frame[11]<<8);

    setTilt(u,ud,udd,uddd);
    setLaunch(rol, rdd);
    return 1;
}

// ==== Flash/Experiment Commands ==============================================================================
// =============================================================================================================
unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    /*
    unsigned int run_time = frame[0] + (frame[1] << 8);
    int i;
    for (i = 0; i < NUM_PIDS; i++){
        pidObjs[i].timeFlag = 1;
        pidSetInput(i, 0);
        checkSwapBuff(i);
        pidOn(i);
    }
    pidObjs[0].mode = 0;
    pidStartTimedTrial(run_time);

    */
    // pid-ip2.5.c removed
    return 1;
}

unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    if (numSamples != 0) {
        telemSetStartTime(); // Start telemetry samples from approx 0 time
        telemSetSamplesToSave(numSamples);
    }
    return 1;
}
unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    telemErase(numSamples);
    
    //Send confirmation packet; this only happens when flash erase is completed.
    radioSendData(src_addr, 0, CMD_ERASE_SECTORS, length, frame, 0);

    LED_RED = ~LED_RED;
    return 1;
}
unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    telemReadbackSamples(numSamples, src_addr);
    return 1;
}

// ==== Motor PID Commands =====================================================================================
// =============================================================================================================

unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    // int thrust1 = frame[0] + (frame[1] << 8);
    // int thrust2 = frame[2] + (frame[3] << 8);
    // unsigned int run_time_ms = frame[4] + (frame[5] << 8);



    // DisableIntT1;   // since PID interrupt overwrites PWM values

    // tiHSetDC(1, thrust1);
    // tiHSetDC(2, thrust2);
    // LED_3 = 1;
    // delay_ms(run_time_ms);
    // tiHSetDC(1,0);
    // tiHSetDC(2,0);

    // EnableIntT1;
    /// HIGHJACKING FUNCTION 8/3/2016 FOR PROP GAINS
    int Kpr = frame[0] + (frame[1] << 8);
    int Kir = frame[2] + (frame[3] << 8);
    int Kdr = frame[4] + (frame[5] << 8);
    int Kpy = frame[6] + (frame[7] << 8);
    int Kiy = frame[8] + (frame[9] << 8);
    int Kdy = frame[10] + (frame[11] << 8);
    pidSetGains(2,Kpr,Kir,Kdr,0,0);
    pidSetGains(3,Kpy,Kiy,Kdy,0,0);
    */
    // pid-ip2.5.c removed
    return 1;
 } 

 unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    int thrust1 = frame[0] + (frame[1] << 8);
    int thrust2 = frame[2] + (frame[3] << 8);

    pidObjs[0].pwmDes = thrust1;
    pidObjs[1].pwmDes = thrust2;

    pidObjs[0].mode = 1;
    */
    // pid-ip2.5.c removed
    return 1;
 }

unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    uint8_t i;
    int16_t gains[10];

    for (i=0; i<10; i++) {
        gains[i] = frame[i*2] + (frame[i*2+1] << 8);
    }
    setGains(gains);

    radioSendData(src_addr, status, CMD_SET_PID_GAINS, 20, frame, 0); //TODO: Robot should respond to source of query, not hardcoded address
    //Send confirmation packet
    // WARNING: Will fail at high data throughput
    //radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_PID_GAINS, status, 20, frame);
    
    return 1; //success
}

unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    int interval[NUM_VELS], delta[NUM_VELS], vel[NUM_VELS], period, onceFlag;
    int idx = 0, i = 0;
    // Packet structure [Period, delta[NUM_VELS], FLAG, Period, delta[NUM_VELS], FLAG]
    period = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;
    for(i = 0; i < NUM_VELS; i ++) {
        interval[i] = period/NUM_VELS;
        delta[i] = (frame[idx]+ (frame[idx+1]<<8));
            if(delta[i]>=8192){
                delta[i] = 8191;
            } else if(delta[i] < -8192){
                delta[i] = -8192;
            }
        delta[i] = delta[i]<<2;
        vel[i] = delta[i]/interval[i];
        idx+=2;
    }
    onceFlag = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;    

    setPIDVelProfile(0, interval, delta, vel, onceFlag);
    
    period = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;
    for(i = 0; i < NUM_VELS; i ++) {
        interval[i] = period/NUM_VELS;
        delta[i] = (frame[idx]+ (frame[idx+1]<<8));
            if(delta[i]>=8192){
                delta[i] = 8191;
            } else if(delta[i] < -8192){
                delta[i] = -8192;
            }
        delta[i] = delta[i]<<2;
        vel[i] = delta[i]/interval[i];
        idx+=2;
        }
    onceFlag = frame[idx] + (frame[idx + 1]<<8);

    setPIDVelProfile(1, interval, delta, vel, onceFlag);

    //Send confirmation packet
    // TODO : Send confirmation packet with packet index
    */
    // pid-ip2.5.c removed
    return 1; //success
}

unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    int i;
    for(i=0;i<NUM_PIDS;i++){  
    pidObjs[i].timeFlag = 0;
    pidSetInput(i, 0);
    pidObjs[i].p_input = pidObjs[i].p_state;
    pidOn(i);
    }
    setControlFlag(1);
    */
    // pid-ip2.5.c removed
    return 1;
}

unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    int i;
    for(i=0;i<NUM_PIDS;i++){  
        pidObjs[i].onoff = 0;
    }
    */
    // pid-ip2.5.c removed
    return 1;
}

unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    /*
    long motor_count[2];
    motor_count[0] = pidObjs[0].p_state;
    motor_count[1] = pidObjs[1].p_state;

    radioSendData(src_addr, status, CMD_ZERO_POS, 
        sizeof(motor_count), (unsigned char *)motor_count, 0);
    pidZeroPos(0);
    pidZeroPos(1);
    */
    // pid-ip2.5.c removed
    return 1;
}

void cmdError() {
    int i;
    // TODO make a real emergency stop
    //EmergencyStop();
    for(i= 0; i < 10; i++) {
        LED_1 ^= 1;
        delay_ms(200);
        LED_2 ^= 1;
        delay_ms(200);
        LED_3 ^= 1;
        delay_ms(200);
    }
}

static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    return 1;
}
