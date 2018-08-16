#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))      # Some projects have a single-directory structure
import shared
import pygame

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    runTailGains = [160,0,30,0,0, 0,0,0,0,0]
    runThrusterGains = [170,0,120, 170,0,120]

    standTailGains = [200,0,25,0,0, 0,0,0,0,0]
    standThrusterGains = [250,0,170, 100,0,150]

    duration = 1000#15000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True#False
    repeat = False

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    params = hallParams(runTailGains, duration, rightFreq, leftFreq, phase, telemetry, repeat)
    xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h', *standThrusterGains))


    sj_params = sjParams(300, 800)
    # wj_params = wjParams(-551287, -50000, 1000000, 4941297, 411774)
    wj_params = wjParams(-551287, -40000, 80000, 5353068, 411774)
    wjParams.set(wj_params)

    # Joystick --------------------
    pygame.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()
    joyNAxes = joy.get_numaxes() # should be 6
    joyNButtons = joy.get_numbuttons() # should be 11
    joyAxes = [0]*joyNAxes
    joyButtons = [0]*joyNButtons
    joyYaw = 0

    started = 0
    stopped = False

    AngleScaling = 3667; # rad to 15b 2000deg/s integrated 1000Hz


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

    # STAND UP ---------------------------------

    print "Hit Y to start"
    while started==0:
        pygame.event.pump() # joystick

        if joy.get_button(3):
            started = 1
        time.sleep(0.01)

    startTelemetrySave(numSamples)
    print "START"

    exp = [2]
    arbitrary = [0]

    #viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
    viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
    time.sleep(0.01)

    xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
    time.sleep(0.01)

    xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
    time.sleep(0.01)

    xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
    time.sleep(0.01)

    modeSignal = [3]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
    
    time.sleep(0.01)
    xb_send(0, command.START_EXPERIMENT, pack('h', *exp))

    time.sleep(3.0)
    motorgains = [350,0,30,0,0, 0,0,0,0,0]
    xb_send(0, command.SET_PID_GAINS, pack('10h',*standTailGains))

    time.sleep(2.0)

    # JUMP ---------------------------------

    print "Hit left trigger to jump"
    while started==1:
        pygame.event.pump() # joystick

        if joy.get_button(4):
            started = 2

        stopped = joy.get_button(5)
        if stopped:
            stopSignal = [0]
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
            time.sleep(0.01)
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))

        time.sleep(0.01)

    modeSignal = [8]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))

    time.sleep(0.01)

    xb_send(0, command.SET_PID_GAINS, pack('10h',*runTailGains))

    time.sleep(0.01)
    xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h',*runThrusterGains))


    while not stopped:
        pygame.event.pump()

        for i in range(joyNAxes):
            joyAxes[i] = joy.get_axis(i)
        joyYaw = joyYaw# -joyAxes[0]/100

        vx1 = int(-joyAxes[4]*2000)
        vy1 = int(-joyAxes[3]*1000)
        vz1 = int(-joyAxes[1]*2000+6000)
        Cyaw = int(joyYaw*AngleScaling)

        toSend = [vx1,vy1,vz1, Cyaw]
        for i in range(4):
            if toSend[i] > 32767:
                toSend[i] = 32767
            elif toSend[i] < -32767:
                toSend[i] = -32767
        xb_send(0, command.SET_VELOCITY, pack('4h',*toSend))

        stopped = joy.get_button(5)

        time.sleep(0.01)




    '''
    # Balance on toe test
    #Start robot 0: wall jump, 1: single jump, 2: vicon jumps
    exp = [2]
    arbitrary = [0]

    #viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
    viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
    time.sleep(0.01)

    xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
    time.sleep(0.01)

    xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
    time.sleep(0.01)

    xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
    time.sleep(0.01)

    modeSignal = [3]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
    time.sleep(0.01)

    xb_send(0, command.START_EXPERIMENT, pack('h', *exp))

    time.sleep(3.0)
    motorgains = [350,0,30,0,0, 0,0,0,0,0]
    xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
    '''
    '''
    time.sleep(5.0)
    viconTest = [0,0,0,0,0,0,58*256,58*256]#55*256,70*256]
    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))

    time.sleep(0.4)
    viconTest = [0,0,0,0,0,0,30*256,30*256]#55*256,70*256]
    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))

    time.sleep(0.05)
    modeSignal = [7]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))

    params.duration = params.duration - 8
    '''

    stopSignal = [0]
    xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
    time.sleep(0.01)
    xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))

    print("STOPPED")

    if params.telemetry and query_yes_no("Save Data?"):
        flashReadback(numSamples, params, manParams)

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
