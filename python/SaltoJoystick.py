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
import csv

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    #runTailGains = [110,0,20,0,0, 0,0,0,0,0]
    #runThrusterGains = [170,0,140, 100,0,140]

    #standTailGains = [180,0,17,0,0, 0,0,0,0,0]
    #standThrusterGains = [250,0,180, 100,0,150]

    zeroGains = [50,30,0, 80,50,0, 0,0,0,0]
    runTailGains = [50,30,0, 120,60,0, 80,13,0,0]#90,13,0,0]
    runThrusterGains = [0,0,0, 0,0,0]

    standTailGains = [50,30,0, 80,50,0, 80,13,0,0]
    standThrusterGains = [0,0,0, 0,0,0]

    k1 = 0
    k2 = -15
    airRetract = 55

    duration = 5000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True#False
    repeat = False

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    params = hallParams(runTailGains, duration, rightFreq, leftFreq, phase, telemetry, repeat)

    # Joystick --------------------
    pygame.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()
    joyNAxes = joy.get_numaxes() # should be 6
    joyNButtons = joy.get_numbuttons() # should be 11
    joyAxes = [0.0]*joyNAxes
    joyButtons = [0]*joyNButtons
    joyYaw = 0.0

    started = 0
    stopped = False

    screen = pygame.display.set_mode((500, 500))

    AngleScaling = 3667; # rad to 15b 2000deg/s integrated 1000Hz

    # Command writing -------------
    path     = 'Data/'
    name     = 'cmd'
    datetime = time.localtime()
    dt_str   = time.strftime('%Y-%m-%d_%H-%M-%S', datetime)
    root     = path + dt_str + '_' + name
    cmdFileName = root + '.txt'
    f = open(cmdFileName,'w')
    cmdwrite = csv.writer(f,delimiter=',')
    start_time = time.time()


    if params.telemetry:
        # Construct filename
        # path     = '/home/duncan/Data/'
        path     = 'Data/'
        name     = 'trial'
        datetime = time.localtime()
        dt_str   = time.strftime('%Y-%m-%d_%H-%M-%S', datetime)
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
        time.sleep(0.02)

    print "START"

    exp = [2]
    arbitrary = [0]

    modeSignal = [0]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
    time.sleep(0.02)

    xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h', *standThrusterGains))
    time.sleep(0.02)

    #zeroGains = [0,0,0,0,0, 0,0,0,0,0]
    xb_send(0, command.SET_PID_GAINS, pack('10h',*zeroGains))
    time.sleep(0.02)

    #viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
    viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
    xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
    time.sleep(0.02)

    xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
    time.sleep(0.02)

    xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
    time.sleep(0.02)

    xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
    time.sleep(0.02)

    adjust = [0,0,-192]
    xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
    time.sleep(0.02)

    modeSignal = [1]
    xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
    time.sleep(0.02)

    xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
    time.sleep(3.0)
    
    xb_send(0, command.SET_PID_GAINS, pack('10h',*standTailGains))
    time.sleep(0.5)

    # JUMP ---------------------------------

    print "Hit left trigger to jump"
    while started==1 and stopped == 0:
        pygame.event.pump() # joystick

        for i in range(joyNAxes):
            joyAxes[i] = joy.get_axis(i)
        if joyAxes[0] > 0.1 or joyAxes[0] < -0.1:
            joyYaw = joyYaw - joyAxes[0]/20.0
        if joyYaw > 3.14159:
            joyYaw = joyYaw - 2*3.14159
        elif joyYaw < -3.14159:
            joyYaw = joyYaw + 2*3.14159
        Cyaw = int(joyYaw*AngleScaling)

        toSend = [0,0,0, Cyaw,0,0, 0,0]
        xb_send(0, command.INTEGRATED_VICON, pack('8h',*toSend))
        time.sleep(0.02)

        if joy.get_button(4):
            started = 2

        stopped = joy.get_button(5)
        if stopped:
            stopSignal = [0]
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
            time.sleep(0.02)
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))

        time.sleep(0.03)

    if not stopped:
        startTelemetrySave(numSamples)

        modeSignal = [6]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.03)
        xb_send(0, command.SET_PID_GAINS, pack('10h',*runTailGains))
        #time.sleep(0.03)
        #xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h',*runThrusterGains))

    crouch = 0
    tailGains = 1

    pygame.event.pump()

    while not stopped:
        pygame.event.pump()

        stopped = joy.get_button(5) or joy.get_hat(0) == (0,-1)

        if joy.get_button(1) and crouch == 0:
            # Land
            crouch = 1
        if joy.get_button(4) and crouch == 2 and tailGains == 1:
            # Jump again
            modeSignal = [6]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)
            crouch = 0

        for i in range(joyNAxes):
            joyAxes[i] = joy.get_axis(i)

        if crouch == 0:
            # Normal jumping
            if joyAxes[0] > 0.1 or joyAxes[0] < -0.1:
                joyYaw = joyYaw - joyAxes[0]/20.0
            if joyYaw > 3.14159:
                joyYaw = joyYaw - 2*3.14159
            elif joyYaw < -3.14159:
                joyYaw = joyYaw + 2*3.14159

            vz1 = int(np.sqrt(joyAxes[2]*1.4+2.4)*4000)
            vx1 = int(-joyAxes[3]*4000*(vz1-2000)/6000)
            vy1 = int(-joyAxes[4]*2000*(vz1-2000)/6000)
            Cyaw = int(joyYaw*AngleScaling)
            #print Cyaw

            if joy.get_button(0):
                screen.fill((50,200,100))
                key = pygame.key.get_pressed()
                if key[pygame.K_LEFT]:
                    vy1 = 1000
                elif key[pygame.K_RIGHT]:
                    vy1 = -1000
                if key[pygame.K_UP]:
                    vx1 = 2000
                elif key[pygame.K_DOWN]:
                    vx1 = -2000
            else:
                vz1 = vz1 # do nothing
                screen.fill((0,0,0))

            pygame.draw.line(screen, (0,0,255), (250, 250), (250-vy1/20, 250-vx1/20), vz1/500)
            pygame.display.flip()

            toSend = [vx1,vy1,vz1, Cyaw]
            for i in range(4):
                if toSend[i] > 32767:
                    toSend[i] = 32767
                elif toSend[i] < -32767:
                    toSend[i] = -32767
            xb_send(0, command.SET_VELOCITY, pack('4h',*toSend))
            time.sleep(0.02)
        elif crouch == 1:
            # Landing
            modeSignal = [23]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)

            viconTest = [0,0,0, 0,0,0, airRetract*256, 25*256]
            xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.02)

            cmd = [0,0,0,0,\
            (0.12)*2**16, 0.0*2000, (0.0)*1024,\
            k1, k2]
            xb_send(0, command.STANCE, pack('9h', *cmd))
            time.sleep(0.01)
            xb_send(0, command.SET_PID_GAINS, pack('10h', *standTailGains))
            time.sleep(0.02)
            crouch = 2
        elif crouch == 2:
            # Balancing on the ground
            if joyAxes[0] > 0.1 or joyAxes[0] < -0.1:
                joyYaw = joyYaw - joyAxes[0]/20.0
            if joyYaw > 3.14159:
                joyYaw = joyYaw - 2*3.14159
            elif joyYaw < -3.14159:
                joyYaw = joyYaw + 2*3.14159
            Cyaw = int(joyYaw*AngleScaling)

            toSend = [0,0,0, Cyaw,0,0, 0,0]
            xb_send(0, command.INTEGRATED_VICON, pack('8h',*toSend))
            time.sleep(0.02)

            if joy.get_button(2):
                # Turn off the tail and let it wind down
                xb_send(0, command.SET_PID_GAINS, pack('10h',*zeroGains))
                time.sleep(0.02)
                tailGains = 0
            if joy.get_button(3):
                # Turn the tail back on
                adjust = [0,0,-512]
                xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
                time.sleep(0.02)
                xb_send(0, command.SET_PID_GAINS, pack('10h',*runTailGains))
                time.sleep(0.02)
                tailGains = 1

        cmdwrite.writerow([time.time()-start_time, vx1, vy1, vz1, Cyaw])

        time.sleep(0.03)

    stopSignal = [0]
    xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
    time.sleep(0.02)
    xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))

    print("STOPPED")

    if params.telemetry and query_yes_no("Save Data?"):
        flashReadback(numSamples, params, manParams)

    print "Done"

    raw_input("Press enter to exit")

    shared.xb.halt()
    shared.ser.close()
    sys.exit()
    
    
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
