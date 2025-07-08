#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys,os,traceback
import serial
import numpy as np

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))      # Some projects have a single-directory structure
import shared

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()

    motorgains = [0,0,0,0,0, 0,0,0,0,0] # disable thrusters and tail
    thrustGains = [0,0,0, 0,0,0]

    duration = 2000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True
    repeat = False

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    sj_params = sjParams(300, 800)
    wj_params = wjParams(-551287, -40000, 80000, 5353068, 411774)
    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)


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

        stopSignal = [0]

        arbitrary = [0]
        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.02)
        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.02)

        # Calibrate --------------------------------------
        # This should be between 0 and 2^14/6 = 2730.7
        # The calibration is good if the leg moves quickly outwards.
        calibPoint = 1012
        toSend = [calibPoint,int((2**11))] # reterminated motor 1
        xb_send(0, command.CALIBRATE_MOTOR, pack('2h', *toSend))
        time.sleep(params.duration / 1000.0)

        toSend = [calibPoint+2730.7/2,int((2**11))] # reterminated motor 1
        xb_send(0, command.CALIBRATE_MOTOR, pack('2h', *toSend))
        time.sleep(params.duration / 1000.0)

        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.1)
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
