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
    motorgains = [0,0,0, 0,0,0, 0,0,0,0]

    thrusterGains = [50,30,0, 80,50,0, 0,0,0,0]
    standGains = [50,30,0, 80,50,0, 100,13,0,0]
    airGains = [50,30,0, 80,40,0, 100,13,0,0]

    duration = 4000#15000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True#False#
    repeat = False

    # Balance control tilt once to 9/4*a*tau^2 rad and 1/2*a*tau rad/s
    a = 25#-25.0 # angular acceleration (rad/s^2)
    tau = 0.05#0.08#0.08# # time scale (s)
    toHop = 1 # make a small jump (1) or not (0)
    v = 3.5 # launch velocity in m/s
    roll = -0.02 # roll in rad

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)
    setMotorGains(motorgains)

    sj_params = sjParams(300, 800)
    wj_params = wjParams(-551287, -40000, 80000, 5353068, 411774)
    wjParams.set(wj_params)


    t_launch = (7.0+2.1815)*tau
    t_energize = t_launch - 0.17


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


        # Balance on toe test
        #Start robot 0: wall jump, 1: single jump, 2: vicon jumps
        exp = [2]
        arbitrary = [0]

        xb_send(0, command.SET_PID_GAINS, pack('10h',*thrusterGains))
        time.sleep(0.02)

        viconTest = [0,0,0, 0,3667*roll,0, 45*256,25*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)

        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.01)

        adjust = [0,-128,-256]#[0,192,-256]# 3667 ticks per radian, yaw, roll, pitch (64 ticks per degree)
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.01)

        #modeSignal = [3]
        modeSignal = [17]#[19]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)
        time.sleep(1.5)#2.0)
        xb_send(0, command.SET_PID_GAINS, pack('10h',*standGains))

        time.sleep(1.5)

        t0 = time.time()
        t = 0.0
        tEnd = 13.0*tau
        while t < tEnd:
            # Md is in 2^15/(2000*pi/180)~=938.7 ticks/rad
            t = time.time() - t0

            if t < 0.0: # balance
                Mddd = 0.0
                Mdd = 0.0
                Md = 0.0
                M = 0.0
            elif t < tau: # begin lean back
                Mddd = -a
                Mdd = -a*t
                Md = -1.0/2.0*a*t**2.0
                M = -1.0/6.0*a*t**3.0
            elif t < 5.0*tau: # reverse lean toward forward
                tr = t - tau
                Mddd = 1.0/2.0*a
                Mdd = -a*tau + 1.0/2.0*a*tr
                Md = -1.0/2.0*a*tau**2.0 - a*tau*tr + 1.0/4.0*a*tr**2.0
                M = -1.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr - 1.0/2.0*a*tau*tr**2.0 + 1.0/12.0*a*tr**3.0
            elif t < 6.0*tau: # follow through forward tilt
                tr = t - 5.0*tau
                Mddd = 0.0
                Mdd = a*tau
                Md = -1.0/2.0*a*tau**2.0 + a*tau*tr
                M = -29.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0
            elif t < 7.0*tau: # slow forward tilt
                tr = t - 6.0*tau
                Mddd = -1.0/2.0*a
                Mdd = a*tau - 1.0/2.0*a*tr
                Md = 1.0/2.0*a*tau**2.0 + a*tau*tr - 1.0/4.0*a*tr**2.0
                M = -29.0/6.0*a*tau**3.0 + 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0 - 1.0/12.0*a*tr**3.0;
            elif t < (7.0+2.1815+2.0)*tau: # hold forward tilt
                tr = t - 7.0*tau
                Mddd = 0.0
                Mdd = 1.0/2.0*a*tau
                Md = 5.0/4.0*a*tau**2.0 + 1.0/2.0*a*tau*tr
                M = -24.0/6.0*a*tau**3.0 + 5.0/4.0*a*tau**2.0*tr + 1.0/4.0*tau*tr**2.0
            else:
                Mddd = 0.0
                Mdd = 0.0
                Md = 0.0
                M = 0.0


            if t > t_energize and toHop == 1: # begin launch
                rd = v
            else:
                rd = 0.0

            # Send tilt command
            cmd = [M*938.7, Md*938.7, Mdd*938.7, Mddd*938.7,\
            3667*roll, rd*2000]
            xb_send(0, command.LAUNCH, pack('6h', *cmd))
            print cmd
            time.sleep(0.01)
                
            if t > 11.0*tau and toHop == 1: # prepare for landing
                # # Make a few bounces, then stop
                # modeSignal = [6]
                # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
                # time.sleep(0.01)
                # toSend = [-2000, 0, 6000, 0]
                # xb_send(0, command.SET_VELOCITY, pack('4h',*toSend))
                # time.sleep(0.01)

                # # Set angle bounce
                # viconTest = [0,0,0, 0,0,3667*-3.0*3.14159/180, 60*256,90*256]#55*256,70*256]
                # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                # time.sleep(0.01)
                # modeSignal = [0]
                # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
                # time.sleep(0.01)

                # Hop once and stop
                viconTest = [0,0,0, 0,0,0, 50*256,25*256]
                xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                time.sleep(0.01)
                xb_send(0, command.SET_PID_GAINS, pack('10h',*airGains))
                time.sleep(0.05)
                toHop = 2

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
        
        if toHop:
            # # Make a few bounces, then stop
            # modeSignal = [23]
            # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            # time.sleep(0.01)
            # viconTest = [0,0,0, 0,0,0, 45*256,25*256]
            # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            # time.sleep(0.01)

            # # Enable if using higher gains
            # time.sleep(0.1)
            # modeSignal = [23]
            # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            # time.sleep(0.01)

            # # New leg control
            time.sleep(0.3)
            cmd = [0,0,0,0,\
            (0.12)*2**16, 0.0*2000, (0+9.81)*1024,\
            -0, -20]
            xb_send(0, command.STANCE, pack('9h', *cmd))
            time.sleep(0.01)

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
