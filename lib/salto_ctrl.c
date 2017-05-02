/*
 * Name: salto_ctrl.c
 * Desc: Salto attitude control: inertial tail and two thrusters
 * Date: 2017-05-01
 * Author: JKY (Justin Yim)
 *
 */

#include "salto_ctrl.h"

#include "led.h"
#include "mpu6000.h"
#include "timer.h"
#include "tih.h"

#include "as5047.h"
#include "ams-enc.h"
#include "protocol.h"
#include "uart_driver.h"
#include "lut.h"



#include "tail_ctrl.h"
#include "pid-ip2.5.h"
extern pidPos pidObjs[NUM_PIDS];



packet_union_t uart_tx_packet_global;

extern EncObj motPos;
extern EncObj encPos[NUM_ENC];
extern unsigned long t1_ticks;

extern long body_angle[3];
volatile long legSetpoint;
volatile long pushoffCmd;

volatile unsigned char mj_state = MJ_IDLE;

void setLegSetpoint(long length){
    legSetpoint = length << 8;
}

void setPushoffCmd(long cmd){
    pushoffCmd = cmd << 8;
}


void expStart(uint8_t startSignal) {
    mj_state = MJ_START;
}

void expStop(uint8_t stopSignal) {
    mj_state = MJ_STOP;
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
            mj_state = MJ_IDLE;
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
        return;
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

#define MOTOR_OFFSET    1000
char footContact(void) {
    int eps = 1000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position/111);//*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur(); //TODO: put into Scripts/lut.m (lib/lut.h) constant
    if ( mot-MOTOR_OFFSET>femur && (mot-MOTOR_OFFSET - eps) > femur)
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
    mot = (unsigned int)(sensor_data->position/111);//*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur();
    if ( (mot-MOTOR_OFFSET + eps) < femur){
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
