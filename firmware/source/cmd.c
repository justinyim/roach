#include "settings.h"
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
#include "version.h"
#include "timer.h"
#include "tih.h"
#include "pid-ip2.5.h"
#include "ams-enc.h"
#include "carray.h"
#include "telem.h"
#include "sync_servo.h" // JY edits

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PKT_UNPACK(type, var, pktframe) type* var = (type*)(pktframe);

unsigned char (*cmd_func[MAX_CMD_FUNC])(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned int);
void cmdError(void);

//extern pidPos pidObjs[NUM_PIDS];
//extern EncObj encPos[NUM_ENC];
//volatile CircArray fun_queue;
//CircArray fun_queue;

//static Payload rx_payload;
//static MacPacket rx_packet;
//static test_function rx_function;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdGetAMSPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//Motor and PID functions
static unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetPhase(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetServo(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr); // JY edits set servo angle
static unsigned char cmdSaltoVicon1(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr); // JY edits Salto Vicon jumps

//Experiment/Flash Commands
static unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdCalibInit(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr); // JY edits initial calibration
/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    //fun_queue = carrayCreate(12);

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC; ++i) {
        cmd_func[i] = &cmdNop;
    }

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
    cmd_func[CMD_SET_PHASE] = &cmdSetPhase;   
    cmd_func[CMD_START_TIMED_RUN] = &cmdStartTimedRun;
    cmd_func[CMD_PID_STOP_MOTORS] = &cmdPIDStopMotors;
    cmd_func[CMD_SET_SERVO] = &cmdSetServo; // JY edits
    cmd_func[CMD_CALIB_INIT] = &cmdCalibInit; // JY edits
    cmd_func[CMD_SALTO_VICON1] = &cmdSaltoVicon1; // JY edits

}

void cmdHandleRadioRxBuffer(void) {
    MacPacket packet;
    Payload pld;
    unsigned char command, status;
    unsigned int rx_src_addr;
    unsigned char* payData;
    unsigned char payDataLength;

    if ((packet = radioDequeueRxPacket()) != NULL) {
        //LED_YELLOW = 1;
        pld = macGetPayload(packet);

        status = payGetStatus(pld);
        command = payGetType(pld);
        payData = payGetData(pld);
        payDataLength = payGetDataLength(pld);
        //We will respond to the packet source
        rx_src_addr = packet->src_addr.val;

        if (command < MAX_CMD_FUNC) {
            cmd_func[command](command, status, payDataLength, payData, rx_src_addr);
        }
        radioReturnPacket(packet);
    }

    return;
}


//void cmdPushFunc(MacPacket rx_packet) {
//    Payload rx_payload;
//    unsigned char command;
//
//    rx_payload = macGetPayload(rx_packet);
//    if(rx_payload != NULL) {
//        command = payGetType(rx_payload);
//
//        if( (command < MAX_CMD_FUNC) && (cmd_func[command] != NULL) ) {
//            rx_payload->test = cmd_func[command];
//            carrayAddTail(fun_queue, rx_packet);
//        } else {
//            cmdError();   // halt on error - could also just ignore....
//        }
//    }
//}


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
    long motor_count[2];
    motor_count[0] = pidGetPState(LEFT_LEGS_PID_NUM);
    motor_count[1] = pidGetPState(RIGHT_LEGS_PID_NUM);

    radioSendData(src_addr, status, CMD_GET_AMS_POS,  //TODO: Robot should respond to source of query, not hardcoded address
            sizeof(motor_count), (unsigned char *)motor_count, 0);

    return 1;
}
// ==== Flash/Experiment Commands ==============================================================================
// =============================================================================================================
unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdStartTimedRun, argsPtr, frame);

    int i;
    for (i = 0; i < NUM_PIDS; i++){
        pidSetTimeFlag(i,1);
        //pidObjs[i].timeFlag = 1;
        pidSetInput(i, 0);
        checkSwapBuff(i);
        pidOn(i);
    }
    
    pidSetMode(LEFT_LEGS_PID_NUM ,PID_MODE_CONTROLED);
    pidSetMode(RIGHT_LEGS_PID_NUM ,PID_MODE_CONTROLED);

    pidStartTimedTrial(argsPtr->run_time);

    return 1;
}

unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdStartTelemetry, argsPtr, frame);

    if (argsPtr->numSamples != 0) {
        telemSetStartTime(); // Start telemetry samples from approx 0 time
        telemSetSamplesToSave(argsPtr->numSamples);
    }
    return 1;
}
unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdEraseSector, argsPtr, frame);

    telemErase(argsPtr->samples);

    //Send confirmation packet; this only happens when flash erase is completed.
    radioSendData(src_addr, 0, CMD_ERASE_SECTORS, length, frame, 0);

    LED_RED = ~LED_RED;
    return 1;
}
unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    PKT_UNPACK(_args_cmdFlashReadback, argsPtr, frame);
    
    telemReadbackSamples(argsPtr->samples, src_addr);
    return 1;
}

// ==== Motor PID Commands =====================================================================================
// =============================================================================================================

unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetThrustOpenLoop, argsPtr, frame);

    DisableIntT1;   // since PID interrupt overwrites PWM values

    tiHSetDC(argsPtr->channel, argsPtr->dc);

    EnableIntT1;
    return 1;
 } 

 unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetMotorMode, argsPtr, frame);


    pidSetPWMDes(LEFT_LEGS_PID_NUM, argsPtr->thrust1);
    pidSetPWMDes(RIGHT_LEGS_PID_NUM, argsPtr->thrust2);

    pidSetMode(LEFT_LEGS_PID_NUM,1);

    return 1;
 }

 unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetPIDGains, argsPtr, frame);
    pidSetGains(LEFT_LEGS_PID_NUM,
            argsPtr->Kp1,argsPtr->Ki1,argsPtr->Kd1,argsPtr->Kaw1, argsPtr->Kff1);

    pidSetGains(RIGHT_LEGS_PID_NUM,
            argsPtr->Kp2,argsPtr->Ki2,argsPtr->Kd2,argsPtr->Kaw2, argsPtr->Kff2);

    radioSendData(src_addr, status, CMD_SET_PID_GAINS, length, frame, 0); //TODO: Robot should respond to source of query, not hardcoded address

    return 1; //success
}

unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetVelProfile, argsPtr, frame);

    int interval1[NUM_VELS], vel1[NUM_VELS], delta1[NUM_VELS];
    int interval2[NUM_VELS], vel2[NUM_VELS], delta2[NUM_VELS];
    int i;

    for(i = 0; i < NUM_VELS; i++){
        delta1[i] = argsPtr->deltaL[i] << 2;
        delta2[i] = argsPtr->deltaR[i] << 2;
        //Clipping of deltas to range [-8192, 8191] ?

        // Calculation of intervals is fixed to equally spaced intervals.
    
        interval1[i] = argsPtr->periodLeft/NUM_VELS;
        interval2[i] = argsPtr->periodRight/NUM_VELS;

        vel1[i] = delta1[i] / interval1[i];
        vel2[i] = delta2[i] / interval2[i];
    }

    setPIDVelProfile(LEFT_LEGS_PID_NUM, interval1, delta1, vel1, argsPtr->flagLeft);
    setPIDVelProfile(RIGHT_LEGS_PID_NUM, interval2, delta2, vel2, argsPtr->flagRight);

    //Send confirmation packet
    // TODO : Send confirmation packet with packet index
    return 1; //success
}

unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    //All actions have been moved to a PID module function
    pidStartMotor(LEFT_LEGS_PID_NUM);
    pidStartMotor(RIGHT_LEGS_PID_NUM);

    return 1;
}

unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    pidOff(LEFT_LEGS_PID_NUM);
    pidOff(RIGHT_LEGS_PID_NUM);

    return 1;
}

unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    long motor_count[2];
    motor_count[0] = pidGetPState(0);
    motor_count[1] = pidGetPState(1);

    radioSendData(src_addr, status, CMD_ZERO_POS, 
        sizeof(motor_count), (unsigned char *)motor_count, 0);

    pidZeroPos(LEFT_LEGS_PID_NUM);
    pidZeroPos(RIGHT_LEGS_PID_NUM);
    
    return 1;
}

unsigned char cmdSetPhase(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    //Unpack unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSetPhase, argsPtr, frame);

    long p_state[2], error;
    p_state[0] = pidGetPState(LEFT_LEGS_PID_NUM);
    p_state[1] = pidGetPState(RIGHT_LEGS_PID_NUM);
    
    error = argsPtr->offset - ( (p_state[0] & 0x0000FFFF) - (p_state[1] & 0x0000FFFF) );

    pidSetPInput(LEFT_LEGS_PID_NUM, p_state[0] + error/2);
    pidSetPInput(RIGHT_LEGS_PID_NUM, p_state[1] - error/2);

    return 1;
}


void cmdError() {
    int i;
    EmergencyStop();
    for(i= 0; i < 10; i++) {
        LED_1 ^= 1;
        delay_ms(200);
        LED_2 ^= 1;
        delay_ms(200);
        LED_3 ^= 1;
        delay_ms(200);
    }
}

// JY edits
unsigned char cmdSetServo(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    PKT_UNPACK(_args_cmdSetServo, argsPtr, frame);
    servoSet(argsPtr->angle);

    return 1;
}

// JY edits
unsigned char cmdCalibInit(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    // Initial calibration, to be run just before the experiment starts

    // Nothing implemented yet

    return 1;
}

unsigned char cmdSaltoVicon1(unsigned char type, unsigned char status, unsigned char length, unsigned char * frame, unsigned int src_addr) {
    // Unpacked unsigned char* frame into structured values
    PKT_UNPACK(_args_cmdSaltoVicon1, argsPtr, frame);
    
    int rolo, pito, yawo, thbo, thlo, Ilo;
    rolo = argsPtr->rol;
    pito = argsPtr->pit;
    yawo = argsPtr->yaw;
    thbo = argsPtr->thb;
    thlo = argsPtr->thl;
    Ilo = argsPtr->Il;

    return 1;
}


static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    return 1;
}
