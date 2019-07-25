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
    motorgains = [40,20,0, 120,80,0, 140,15,0,0]

    launch = 70
    t_off = -0.08#-0.02
    a = -35
    tilt = -3
    retract = 70

    tau = 0.05

    duration = 1500
    rightFreq = 0
    leftFreq = 0
    phase = -3
    telemetry = True
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


        #'''
        # Balance on toe test
        #Start robot 0: wall jump, 1: single jump, 2: vicon jumps
        exp = [2]
        arbitrary = [0]

        thrusterGains = [20,15,0, 40,40,0, 0,0,0,0] #[50,25,0, 180,140,0, 0,0,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*thrusterGains))
        time.sleep(0.02)

        viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.01)

        adjust = [0,-32,-256]#[0,192,-256]# 3667 ticks per radian, yaw, roll, pitch (64 ticks per degree)
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.01)

        modeSignal = [23]#[19]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)
        time.sleep(2.0)

        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(0.01)

        time.sleep(1.5)
        startTelemetrySave(numSamples)
        time.sleep(0.5)

        # Balance control tilt once to 9/4*a*tau^2 rad and 1/2*a*tau rad/s
        toHop = 1 # make a small jump (1) or not (0)

        t0 = time.time()
        t = 0.0
        tEnd = 13.0*tau
        while t < tEnd:
            # Md is in 2^15/(2000*pi/180)~=938.7 ticks/rad
            t = time.time() - t0

            if t < 0.0: # balance
                Mdd = 0.0
                Md = 0.0
                M = 0.0
            elif t < tau: # begin lean back
                Mdd = -a*t
                Md = -1.0/2.0*a*t**2.0
                M = -1.0/6.0*a*t**3.0
            elif t < 5.0*tau: # reverse lean toward forward
                tr = t - tau
                Mdd = -a*tau + 1.0/2.0*a*tr
                Md = -1.0/2.0*a*tau**2.0 - a*tau*tr + 1.0/4.0*a*tr**2.0
                M = -1.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr - 1.0/2.0*a*tau*tr**2.0 + 1.0/12.0*a*tr**3.0
            elif t < 6.0*tau: # follow through forward tilt
                tr = t - 5.0*tau
                Mdd = a*tau
                Md = -1.0/2.0*a*tau**2.0 + a*tau*tr
                M = -29.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0
            elif t < 7.0*tau: # slow forward tilt
                tr = t - 6.0*tau
                Mdd = a*tau - 1.0/2.0*a*tr
                Md = 1.0/2.0*a*tau**2.0 + a*tau*tr - 1.0/4.0*a*tr**2.0
                M = -29.0/6.0*a*tau**3.0 + 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0 - 1.0/12.0*a*tr**3.0;
            elif t < (7.0+2.217+2.0)*tau: # hold forward tilt
                tr = t - 7.0*tau
                Mdd = 1.0/2.0*a*tau
                Md = 5.0/4.0*a*tau**2.0 + 1.0/2.0*a*tau*tr
                M = -24/6*a*tau**3.0 + 5.0/4.0*a*tau**2.0*tr + 1.0/4.0*tau*tr**2.0
            else:
                Mdd = 0.0
                Md = 0.0
                M = 0.0

            t_launchStart = 9.217*tau - (0.16+t_off)

            # Send tilt command
            tiltCmd = [M*938.7, Md*938.7, Mdd*938.7]
            xb_send(0, command.TILT, pack('3h', *tiltCmd))
            print tiltCmd
            time.sleep(0.01)

            if t > t_launchStart and toHop == 1: # begin launch
                # # Higher gains
                # modeSignal = [7]
                # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
                # time.sleep(0.01)
                
                # Normal
                viconTest = [0,0,0,0,0,0,launch*256,launch*256]
                xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                time.sleep(0.01)
                toHop = 2
            if t > 11.0*tau and toHop == 2: # prepare for landing
                # # Make a few bounces, then stop
                # modeSignal = [6]
                # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
                # time.sleep(0.01)
                # toSend = [-2000, 0, 6000, 0]
                # xb_send(0, command.SET_VELOCITY, pack('4h',*toSend))
                # time.sleep(0.01)

                # Set angle bounce
                viconTest = [0,0,0, 0,0,3667*tilt*3.14159/180, retract*256,90*256]#55*256,70*256]
                xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                time.sleep(0.1)
                modeSignal = [0]
                xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
                time.sleep(0.01)

                # # Hop once and stop
                # viconTest = [0,0,0, 0,0,0, 50*256,30*256]
                # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                # time.sleep(0.01)

        tiltCmd = [0, 0, 0]
        xb_send(0, command.TILT, pack('3h', *tiltCmd))
        time.sleep(0.02)
        
        if toHop:
            # # Make a few bounces, then stop
            # time.sleep(0.7)
            # modeSignal = [23]
            # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            # time.sleep(0.01)
            # viconTest = [0,0,0, 0,0,0, 45*256,25*256]
            # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            # time.sleep(0.01)

            time.sleep(0.7)
            # Enable if using higher gains
            time.sleep(0.1)
            #modeSignal = [23]
            #xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            #time.sleep(0.01)
            viconTest = [0,0,0, 0,0,3667*-2.0*3.14159/180, retract*256,30*256]#55*256,70*256]
            xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.1)
            
            # # Sit down
            # time.sleep(1.5)
            # viconTest = [0,0,0, 0,0,3667*-1*3.14159/180, 20*256,20*256]
            # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            # time.sleep(1.0)
            # viconTest = [0,0,0, 0,0,3667*-0*3.14159/180, 15*256,15*256]
            # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            # time.sleep(1.0)
            # viconTest = [0,0,0, 0,0,3667*-0*3.14159/180, 0*256,0*256]
            # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            # time.sleep(2.0)


        # # Extend leg, then do other things
        # viconTest = [0,0,0,0,0,0,30*256,60*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.1)
        # viconTest = [0,0,0,0,0,0,25*256,25*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)

        # # Balance controller sinusoidal tilt
        # tEnd = 10 # duration (s)
        # a = 3*3.14159/180*938.7 # amplitude (rad)
        # w = 1*2*3.14159 # angular velocity (rad/s)
        # t0 = time.time()
        # t = 0.0
        # while t < tEnd:
        #     # ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
        #     t = time.time() - t0
        #     tiltCmd = [a/w*np.sin(w*t), a*np.cos(w*t), -a*w*np.sin(w*t)]
        #     xb_send(0, command.TILT, pack('3h', *tiltCmd))
        #     print tiltCmd
        #     time.sleep(0.02)
        # tiltCmd = [0, 0, 0]
        # xb_send(0, command.TILT, pack('3h', *tiltCmd))
        # time.sleep(0.02)
        # tiltCmd = [0, 0, 0]
        # xb_send(0, command.TILT, pack('3h', *tiltCmd))
        # time.sleep(0.02)

        # # Jump
        # modeSignal = [7]
        # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        # time.sleep(0.01)
        # viconTest = [0,0,0,0,0,0,90*256,90*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(0.2)
        # modeSignal = [23]
        # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        # time.sleep(0.01)
        # viconTest = [0,0,0,0,0,0,45*256,25*256]
        # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        # time.sleep(1.0)


        #time.sleep(params.duration / 500.0)
        time.sleep(0.2)
        
        #time.sleep(10)
        stopSignal = [0]
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.01)
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.02)
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))


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
