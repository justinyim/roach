#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys,os,traceback
import serial
import math

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))      # Some projects have a single-directory structure
import shared

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Kd , other , Kp , Kd , other , Kp , Kd , other , other ]
    motorgains = [100,80,0, 150,120,0, 120,15,0,0]
    motorgains = [0,0,0, 0,0,0, 0,0,0,0]

    duration = 4000#15000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True#False#
    repeat = False

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)
    setMotorGains(motorgains)

    sj_params = sjParams(300, 800)
    wj_params = wjParams(-551287, -40000, 80000, 5353068, 411774)
    wjParams.set(wj_params)


    while True:

        if not(params.repeat):
            settingsMenu(params, sj_params, wj_params)

        if params.telemetry:
            # Construct filename
            # path     = '/home/duncan/Data/'
            path     = 'Data/'
            name     = 'trial'
            datetime = time.localtime()
            dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
            root     = path + dt_str + '_' + name
            shared.dataFileName = root + '_imudata.txt'
            print "Data file:  ", shared.dataFileName
            print os.curdir

            numSamples = int(ceil(1000 * (params.duration + shared.leadinTime + shared.leadoutTime) / 1000.0))
            eraseFlashMem(numSamples)
            raw_input("Press enter to start run ...") 
            startTelemetrySave(numSamples)


        '''
        # basic leg extension test
        exp = [2]
        arbitrary = [0]
        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.02)
        
        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.02)
        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.02)

        viconTest = [0,0,0,0,0,0,55*256,90*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.02)
        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))

        #time.sleep(0.02)
        #for x in range(500):
        #    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #    time.sleep(0.01)

        '''


        '''
        # leg extension test with variable motor gains
        arbitrary = [0]
        legPosition = [30*256, 0.1*65535, 0.002*65535]
        # motor deflection [radians * 256], P gain [65535 * duty cyle/rad], D gain [65535 * duty cyle/(rad/s)]
        xb_send(0, command.SET_MOTOR_POS, pack('3H', *legPosition))
        '''


        #'''
        # Swing-up pendulum test
        exp = [2]
        arbitrary = [0]

        angle = [3667*3.14159]

        #motorgains = [0,0,0, 0,0,0, 200,12,0,12]
        motorgains = [0,0,0, 0,0,0, 200,12,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)


        #xb_send(0, command.RESET_BODY_ANG, pack('h', *angle))
        #time.sleep(0.01)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.01)
        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)
        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.01)
        adjust = [0,64,-128] # 3667 ticks per radian, yaw, roll, pitch (64 ticks per degree)
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.01)


        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)

        #modeSignal = [7]
        modeSignal = [64]#[33]#[32]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.1)
        #'''


        '''
        # Hacky constant output
        exp = [2]

        motorgains = [0,0,0, 0,0,0, 300,0,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(0.2)

        viconTest = [0,0,0,0,0,0,40*256,40*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        angle = [3667/229]
        xb_send(0, command.RESET_BODY_ANG, pack('h', *angle))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)

        for x in range(1000):
            angle = [3667/229]
            xb_send(0, command.RESET_BODY_ANG, pack('h', *angle))
            time.sleep(0.01)

        stopSignal = [0]
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.01)
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))

        '''


        '''
        # Balance on toe test
        #Start robot 0: wall jump, 1: single jump, 2: vicon jumps
        exp = [2]
        arbitrary = [0]

        motorgains = [20,15,0, 40,40,0, 0,0,0,0] #[50,25,0, 180,140,0, 0,0,0,0]
        #motorgains = [0,0,0, 0,0,0, 0,0,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(0.02)

        #viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
        viconTest = [0,0,0,0,0,0,3*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.01)

        adjust = [0,0,-256]#[0,192,-256]# 3667 ticks per radian, yaw, roll, pitch (64 ticks per degree)
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.01)

        #modeSignal = [3]
        modeSignal = [23]#[19]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        viconTest = [0,0,0, 0,3667*-0.5*3.14159/180,0, 3*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)
        #time.sleep(1.5)#2.0)

        #motorgains = [200,0,22,0,0, 0,0,0,0,0]
        #motorgains = [130,0,13,0,5, 0,0,0,0,0]
        #motorgains = [110,0,12,0,5, 0,0,0,0,0]
        motorgains = [40,15,0, 40,40,0, 100,14,0,0] #[50,25,0, 180,140,0, 160,12,0,12]
        #motorgains = [0,0,0, 0,0,0, 160,12,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        #time.sleep(15.0)

        time.sleep(1.5)
        # countDown = 2
        # for x in range(countDown):
        #     print countDown-x
        #     time.sleep(1.0)
        #     #viconTest = [0,0,0, 0,3667*(3-6*(x%2))*3.14159/360,0, 3*256,0*256]#55*256,70*256]
        #     #xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))


        # # Balance controller triangle wave
        # tEnd = 10.0 # duration (s)
        # a = 8*3.14159/180 # angular velocity (rad/s)
        # T = 1.5 # Period (s)
        # t0 = time.time()
        # t = 0.0
        # while t < tEnd:
        #     t = time.time() - t0
        #     tr = t%T
        #     if tr < (0.5*T):
        #         Mddd = 0.0
        #         Mdd = a
        #         Md = -a*T/4.0 + a*tr
        #         M = -a*T**2.0/16.0*tr + 0.5*a*tr**2
        #     else:
        #         Mddd = 0.0
        #         Mdd = -a
        #         Md = a*T/4.0 - a*(tr-0.5*T)
        #         M = a*T**2.0/16.0*(tr-0.5*T) - 0.5*a*(tr-0.5*T)**2

        #     tiltCmd = [M*938.7, Md*938.7, Mdd*938.7, Mddd*938.7]
        #     xb_send(0, command.TILT, pack('4h', *tiltCmd))
        #     print tiltCmd
        #     time.sleep(0.02)

        # tiltCmd = [0, 0, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.02)
        # tiltCmd = [0, 0, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.02)


        # # Balance controller sinusoidal tilt
        # tEnd = 10 # duration (s)
        # a = 5*3.14159/180*938.7 # amplitude (rad)
        # w = 1*2*3.14159 # angular velocity (rad/s)
        # t0 = time.time()
        # t = 0.0
        # while t < tEnd:
        #     # ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
        #     t = time.time() - t0
        #     tiltCmd = [a/w*np.sin(w*t), a*np.cos(w*t), -a*w*np.sin(w*t), -a*w*w*np.cos(w*t)]
        #     xb_send(0, command.TILT, pack('4h', *tiltCmd))
        #     print tiltCmd
        #     time.sleep(0.02)
        # tiltCmd = [0, 0, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.02)
        # tiltCmd = [0, 0, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.02)


        # # Balance control tilt once to 9/4*a*tau^2 rad and 1/2*a*tau rad/s
        # a = -25.0# angular acceleration (rad/s^2)
        # tau = 0.05#0.08#0.08# # time scale (s)
        # toHop = 1 # make a small jump (1) or not (0)

        # motorgains = [40,20,0, 120,80,0, 140,15,0,0]
        # xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        # time.sleep(0.01)

        # t0 = time.time()
        # t = 0.0
        # tEnd = 13.0*tau
        # while t < tEnd:
        #     # Md is in 2^15/(2000*pi/180)~=938.7 ticks/rad
        #     t = time.time() - t0

        #     if t < 0.0: # balance
        #         Mddd = 0.0
        #         Mdd = 0.0
        #         Md = 0.0
        #         M = 0.0
        #     elif t < tau: # begin lean back
        #         Mddd = -a
        #         Mdd = -a*t
        #         Md = -1.0/2.0*a*t**2.0
        #         M = -1.0/6.0*a*t**3.0
        #     elif t < 5.0*tau: # reverse lean toward forward
        #         tr = t - tau
        #         Mddd = 1.0/2.0*a
        #         Mdd = -a*tau + 1.0/2.0*a*tr
        #         Md = -1.0/2.0*a*tau**2.0 - a*tau*tr + 1.0/4.0*a*tr**2.0
        #         M = -1.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr - 1.0/2.0*a*tau*tr**2.0 + 1.0/12.0*a*tr**3.0
        #     elif t < 6.0*tau: # follow through forward tilt
        #         tr = t - 5.0*tau
        #         Mddd = 0.0
        #         Mdd = a*tau
        #         Md = -1.0/2.0*a*tau**2.0 + a*tau*tr
        #         M = -29.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0
        #     elif t < 7.0*tau: # slow forward tilt
        #         tr = t - 6.0*tau
        #         Mddd = -1.0/2.0*a
        #         Mdd = a*tau - 1.0/2.0*a*tr
        #         Md = 1.0/2.0*a*tau**2.0 + a*tau*tr - 1.0/4.0*a*tr**2.0
        #         M = -29.0/6.0*a*tau**3.0 + 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0 - 1.0/12.0*a*tr**3.0;
        #     elif t < (7.0+2.217+2.0)*tau: # hold forward tilt
        #         tr = t - 7.0*tau
        #         Mddd = 0.0
        #         Mdd = 1.0/2.0*a*tau
        #         Md = 5.0/4.0*a*tau**2.0 + 1.0/2.0*a*tau*tr
        #         M = -24/6*a*tau**3.0 + 5.0/4.0*a*tau**2.0*tr + 1.0/4.0*tau*tr**2.0
        #     else:
        #         Mddd = 0.0
        #         Mdd = 0.0
        #         Md = 0.0
        #         M = 0.0

        #     t_launchStart = 9.217*tau - (0.16-0.04)#(0.16)

        #     # Send tilt command
        #     tiltCmd = [M*938.7, Md*938.7, Mdd*938.7, Mddd*938.7]
        #     xb_send(0, command.TILT, pack('4h', *tiltCmd))
        #     print tiltCmd
        #     time.sleep(0.01)

        #     if t > t_launchStart and toHop == 1: # begin launch
        #         # Normal
        #         viconTest = [0,0,0,0,0,0,95*256,95*256]
        #         xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #         time.sleep(0.01)
        #         toHop = 2

        #         # Higher gains
        #         modeSignal = [1]#[7]
        #         xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #         time.sleep(0.01)
                
        #     if t > 11.0*tau and toHop == 2: # prepare for landing
        #         # # Make a few bounces, then stop
        #         # modeSignal = [6]
        #         # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #         # time.sleep(0.01)
        #         # toSend = [-2000, 0, 6000, 0]
        #         # xb_send(0, command.SET_VELOCITY, pack('4h',*toSend))
        #         # time.sleep(0.01)

        #         # # Set angle bounce
        #         # viconTest = [0,0,0, 0,0,3667*-3.0*3.14159/180, 60*256,90*256]#55*256,70*256]
        #         # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #         # time.sleep(0.01)
        #         # modeSignal = [0]
        #         # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #         # time.sleep(0.01)

        #         # Hop once and stop
        #         viconTest = [0,0,0, 0,0,0, 40*256,20*256]
        #         xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #         time.sleep(0.01)

        # time.sleep(0.2)
        # modeSignal = [0]
        # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        # time.sleep(0.01)
        # viconTest = [0,0,0, 0,0,-1.5*3667, 40*256,20*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.01)


        # tiltCmd = [0, 0, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.02)
        
        # if toHop:
        #     # Make a few bounces, then stop
        #     time.sleep(0.7)
        #     # modeSignal = [23]
        #     # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #     # time.sleep(0.01)
        #     # viconTest = [0,0,0, 0,0,0, 45*256,25*256]
        #     # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     # time.sleep(0.01)

        #     # # Enable if using higher gains
        #     # time.sleep(0.1)
        #     # modeSignal = [23]
        #     # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #     # time.sleep(0.01)
            
        #     # # Sit down
        #     # time.sleep(1.5)
        #     # viconTest = [0,0,0, 0,0,3667*-1*3.14159/180, 20*256,20*256]
        #     # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     # time.sleep(1.0)
        #     # viconTest = [0,0,0, 0,0,3667*-0*3.14159/180, 15*256,15*256]
        #     # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     # time.sleep(1.0)
        #     # viconTest = [0,0,0, 0,0,3667*-0*3.14159/180, 0*256,0*256]
        #     # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     # time.sleep(2.0)


        # Slow extension
        tiltCmd = [0, 0.02*938.7, 0, 0]
        xb_send(0, command.TILT, pack('4h', *tiltCmd))
        time.sleep(0.01)
        viconTest = [0,0,0, 0,0,0, 25*256,40*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.07)
        tiltCmd = [0, 0.02*938.7, 0, 0]
        xb_send(0, command.TILT, pack('4h', *tiltCmd))
        time.sleep(0.08)
        viconTest = [0,0,0, 0,0,0, 90*256,90*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.1)
        viconTest = [0,0,0, 0,0,0, 50*256,25*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.02)
        modeSignal = [23+32]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.1)

        time.sleep(1.5)
        viconTest = [0,0,0, 0,0,0, 15*256,15*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1.0)
        viconTest = [0,0,0, 0,0,0, 0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.02)


        # # Extend leg, then do other things
        # viconTest = [0,0,0, 0,3667*2.0*3.14159/180,0, 3*256,0*256]#55*256,70*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.5)
        # viconTest = [0,0,0,0,0,-0.0*3667*3.14159/180,25*256,37*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.1)
        # viconTest = [0,0,0,0,0,-0.0*3667*3.14159/180,25*256,25*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.5)
        # tiltCmd = [0, -1*3.14159/180*938.7, 0, 0]
        # xb_send(0, command.TILT, pack('4h', *tiltCmd))
        # time.sleep(0.01)

        # # # Balance controller sinusoidal tilt
        # # tEnd = 10 # duration (s)
        # # a = 3*3.14159/180*938.7 # amplitude (rad)
        # # w = 1*2*3.14159 # angular velocity (rad/s)
        # # t0 = time.time()
        # # t = 0.0
        # # while t < tEnd:
        # #     # ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
        # #     t = time.time() - t0
        # #     tiltCmd = [a/w*np.sin(w*t), a*np.cos(w*t), -a*w*np.sin(w*t)]
        # #     xb_send(0, command.TILT, pack('3h', *tiltCmd))
        # #     print tiltCmd
        # #     time.sleep(0.02)
        # # tiltCmd = [0, 0, 0]
        # # xb_send(0, command.TILT, pack('3h', *tiltCmd))
        # # time.sleep(0.02)
        # # tiltCmd = [0, 0, 0]
        # # xb_send(0, command.TILT, pack('3h', *tiltCmd))
        # # time.sleep(0.02)

        # # Jump
        # modeSignal = [7]
        # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        # time.sleep(0.01)
        # viconTest = [0,0,0,0,0,0,90*256,90*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.2)
        # modeSignal = [23+32]
        # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        # time.sleep(0.01)
        # viconTest = [0,0,0,0,0,0,45*256,25*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)


        # # Extend and retract leg up and down
        # viconTest = [0,0,0,0,0,0,30*256,60*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.1)
        # viconTest = [0,0,0,0,0,0,25*256,25*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)
        # viconTest = [0,0,0,0,0,0,20*256,20*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.25)
        # for x in range(10):
        #     viconTest = [0,0,0,0,0,0,30*256,30*256]
        #     xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     time.sleep(0.25)
        #     viconTest = [0,0,0,0,0,0,20*256,20*256]
        #     xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     time.sleep(0.25)
        # viconTest = [0,0,0,0,0,0,15*256,15*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)
        # viconTest = [0,0,0,0,0,0,0*256,0*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)


        # # Turn in place
        # viconTest = [0,0,0, 0,3667*-1.0*3.14159/180,0, 3*256,0*256]#55*256,70*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.02)
        # motorgains = [30,15,0, 70,50,0, 100,14,0,0] #[50,25,0, 180,140,0, 160,12,0,12]
        # xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        # time.sleep(0.02)

        # fCmd = 4;
        # degPerS = 60
        # duration_s = 6
        # for x in range(duration_s*fCmd):
        #     viconTest = viconTest = [0,0,0, degPerS*64*x/fCmd,0,0, 0*256,0*256]
        #     xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #     time.sleep(1.0/fCmd)
        # time.sleep(2.0)

        '''

        '''
        # Foot buzzing (new balance control)
        tEnd = 2 # duration (s)
        a = 10.0*3.14159/180*938.7 # amplitude (rad)
        f = 12.0 # frequency in Hz

        if f > 20.0:
            f = 20.0
        t = 0.0
        t0 = time.time()
        cntr = 0
        while t < tEnd:
            # ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
            t = time.time() - t0
            if cntr:
                legPosition = [0*256, 0.03*65535, 0.00*65535]
                tiltCmd = [0, a, 0, 0]
                cntr = 0
            else:
                legPosition = [12*256, 0.03*65535, 0.00*65535]
                tiltCmd = [0, -a, 0, 0]
                cntr = 1
            xb_send(0, command.TILT, pack('4h', *tiltCmd))
            time.sleep(0.15/f-0.001)
            xb_send(0, command.SET_MOTOR_POS, pack('3h', *legPosition))
            time.sleep(0.35/f-0.001)
        tiltCmd = [0, 0, 0, 0]
        xb_send(0, command.TILT, pack('4h', *tiltCmd))
        time.sleep(0.02)
        tiltCmd = [0, 0, 0, 0]
        xb_send(0, command.TILT, pack('4h', *tiltCmd))
        time.sleep(0.02)
        '''
        '''
        # Foot buzzing (old)
        for x in range(50):
            legPosition = [11*256, 0.03*65535, 0.00*65535]
            xb_send(0, command.SET_MOTOR_POS, pack('3h', *legPosition))
            time.sleep(0.01)
            viconTest = [0,0,0, 0,0,3667*0.02*3, 0*256,0*256]#55*256,70*256]
            #xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.03)

            legPosition = [0*256, 0.03*65535, 0.00*65535]
            xb_send(0, command.SET_MOTOR_POS, pack('3h', *legPosition))
            time.sleep(0.01)
            viconTest = [0,0,0, 0,0,-3667*0.02*3, 0*256,0*256]#55*256,70*256]
            #xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.03)

        viconTest = [0,0,0, 0,0,0, 0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)
        '''
        '''
        # Short vertical hop
        #modeSignal = [0]#[19] # mode 0 for full power (instead of GAINS_STAND)
        #xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        #time.sleep(0.01)
        viconTest = [0,0,0,0,0,0,90*256,90*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.2)
        modeSignal = [23]#[19]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        viconTest = [0,0,0, 0,0,0, 65*256,25*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        time.sleep(2.0)
        viconTest = [0,0,0, 0,0,0, 20*256,20*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.25)
        # viconTest = [0,0,0, 0,0,0, 15*256,15*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.25)
        # viconTest = [0,0,0, 0,0,0, 0*256,0*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.01)
        '''

        '''
        # Short jump
        motorgains = [100,50,0, 350,170,0, 120,20,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(0.05)

        #viconTest = [0,0,0,0,0,0,25*256,50*256]#55*256,70*256]
        #viconTest = [0,0,0,0,0,0,35*256,50*256]
        viconTest = [0,0,0,0,0,3667.0*3.14159/180.0*10.0, 0,0]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.05)

        #viconTest = [0,0,0,0,0,3667.0*3.14159/180.0*10.0, 35*256, 50*256]
        viconTest = [0,0,0,0,0,3667.0*3.14159/180.0*10.0, 70*256, 70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.1)

        motorgains = [90,40,0, 130,110,0, 90,13,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        time.sleep(0.6)
        # End balance on toe test
        '''

        '''
        # small step calibration for crank
        exp = [2]
        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)
        for x in np.hstack((np.linspace(0,80,17),np.linspace(75,0,16))):
            viconTest = [0,0,0,0,0,0, x*256, x*256]
            xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.3)
        '''


        '''
        # small step calibration for crank 2
        # leg extension test with variable motor gains
        arbitrary = [0]
        # motor deflection [radians * 256], P gain [65536 * duty cyle/rad], D gain [65536 * duty cyle/(rad/s)]
        for x in np.hstack((np.linspace(0,90,46),np.linspace(88,0,45))):
            legPosition = [x*256, 0.03*65536, 0.005*65536]
            xb_send(0, command.SET_MOTOR_POS, pack('3h', *legPosition))
            time.sleep(0.02)
        '''

        '''
        # Toe pull-ups
        # leg extension test with variable motor gains
        arbitrary = [0]
        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.02)
        # motor deflection [radians * 256], P gain [65536 * duty cyle/rad], D gain [65536 * duty cyle/(rad/s)]
        # for x in np.hstack((np.linspace(80,20,31),np.linspace(22,80,30))):
        for x in 50+30*np.cos(np.linspace(0,np.sqrt(4*2*3.14159),360)**2):#np.cos(np.linspace(0,6.2832,60)):
            legPosition = [x*256, 0.03*65536, 0.001*65536]
            xb_send(0, command.SET_MOTOR_POS, pack('3h', *legPosition))
            time.sleep(0.01)
        '''


        '''
        # five leg extension points
        exp = [2]
        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(1)
        viconTest = [0,0,0,0,0,0,20*256,20*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1)
        viconTest = [0,0,0,0,0,0,40*256,40*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1)
        viconTest = [0,0,0,0,0,0,60*256,60*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1)
        viconTest = [0,0,0,0,0,0,80*256,80*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1)
        '''


        time.sleep(params.duration / 500.0)
        time.sleep(0.5)
        
        #time.sleep(10)
        stopSignal = [0]
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.01)
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.02)
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))


        # temp = [0]
        # xb_send(0, command.RESET_BODY_ANG, "0")
        # xb_send(0, command.PID_START_MOTORS, "0")
        # xb_send(0, command.SET_PITCH_SET, pack('l', *temp))
        # time.sleep(params.duration/1000.0)
        # xb_send(0, command.PID_STOP_MOTORS, "0")

        if params.telemetry and query_yes_no("Save Data?"):
            flashReadback(numSamples, params, manParams)

        repeatMenu(params)

    print "Done"
    
    
#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
#TODO: provide a more informative exit here; stack trace, exception type, etc
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_stack()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
