#!/usr/bin/env python
"""
cmd module
authors: stanbaek, apullin
Created on 2010-07-07:
Modified by Andrew Pullin for OctoROACH

"""

# CMD values of 0x00 - 0x7F(127) are defined here
# Add CMD definitions 
# for bootloader (0x00 - 0x1F)
CMD_NACK = 0x00        # START_APPLICATION = 0
CMD_ACK = 0x01
CMD_READ_PM = 0x02
CMD_WRITE_PM = 0x03
CMD_READ_EE = 0x04
CMD_WRITE_EE = 0x05
CMD_READ_CM = 0x06
CMD_WRITE_CM = 0x07
CMD_RESET = 0x08
CMD_READ_ID = 0x09
CMD_READ_GOTO = 0x10

SET_THRUST = 0x11
SET_STEER = 0x12
ECHO = 0x1F      # send back the received packet

# for IMU (0x20 - 0x3F)
GET_IMU_DATA = 0x20
GET_IMU_LOOP = 0x21
START_IMU_SAVE = 0x22
STOP_IMU_SAVE = 0x23

SET_POSE_SAVE_FLASH = 0x25
SET_ESTIMATE_POSE = 0x26

TX_SAVED_IMU_DATA = 0x2A
TX_SAVED_STATE_DATA = 0x2B
TX_DUTY_CYCLE = 0x2C

START_AUTO_CTRL = 0x30
STOP_AUTO_CTRL = 0x31

ERASE_MEM_SECTOR = 0x3A
 
RESET_STOPWATCH = 0x3B

BASE_ECHO = 0x3f


# CMD values of 0x80(128) - 0xEF(239) are available for user applications.
SET_THRUST_OPEN_LOOP    =   0x80
PID_START_MOTORS        =   0x81
SET_PID_GAINS           =   0x82
GET_PID_TELEMETRY       =   0x83
GET_AMS_POS	            =   0x84
GET_IMU_LOOP_ZGYRO      =   0x85 #these aren't implemented in roach/firmware/source/cmd.c and .h
SET_MOVE_QUEUE          =   0x86 #these aren't implemented in roach/firmware/source/cmd.c and .h
SET_STEERING_GAINS      =   0x87 #these aren't implemented in roach/firmware/source/cmd.c and .h
SOFTWARE_RESET          =   0x88 #these aren't implemented in roach/firmware/source/cmd.c and .h
ERASE_SECTORS           =   0x8A
FLASH_READBACK          =   0x8B
SLEEP                   =   0x8C #these aren't implemented in roach/firmware/source/cmd.c and .h
SET_VEL_PROFILE         =   0x8D
WHO_AM_I                =   0x8E
START_TELEMETRY         =   0x8F
ZERO_POS                =   0x90
START_TIMED_RUN         =   0x91
PID_STOP_MOTORS         =   0x92
SET_PHASE               =   0x93
SET_MOTOR_MODE          =   0x94

SET_TAIL_POS            =   0x95 #these aren't implemented in roach/firmware/source/cmd.c and .h
SET_PITCH_SET           =   0x96
RESET_BODY_ANG          =   0x97
SET_CURRENT_LIMITS      =   0x98
SET_MOTOR_POS           =   0x99
START_EXPERIMENT        =   0x9A
SET_EXP_PARAMS          =   0x9B

STOP_EXPERIMENT         =   0xA0
INTEGRATED_VICON        =   0xA1
CALIBRATE_MOTOR         =   0xA2
ONBOARD_MODE            =   0xA3
GYRO_BIAS               =   0xA4
G_VECT_ATT              =   0xA5
SET_VELOCITY            =   0xA6
ADJUST_BODY_ANG         =   0xA7
TILT                    =   0xA8

# CMD values of 0xF0(240) - 0xFF(255) are reserved for future use
