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

    duration = 5000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = False#True
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

        # Calibrate --------------------------------------
        '''
        # Check eight points
        n_steps = 8
        ispdf = [0 for i in range(n_steps)]
        #ispdb = [0 for i in range(n_steps)]
        for i in range(n_steps):
            toSend = [int(2730.7*i/n_steps),int(2**13)]
            xb_send(0, command.CALIBRATE_MOTOR, pack('2h', *toSend))
            time.sleep(1.0)
            ispdf[i] = float(shared.bytesIn)/2**16
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
            time.sleep(0.2)

        # Find the octant for higher resolution search
        vel1 =  max(ispdf)
        for i in range(n_steps):
            if ispdf[i] == vel1:
                ang1 = i
                break
        if ang1 == 0:
            ang1 = n_steps-1
            ang2 = n_steps+1
        elif ang1 == n_steps-1:
            ang1 = n_steps-2
            ang2 = n_steps
        else:
            ang2 = ang1+1
            ang1 = ang1-1
        
        print ispdf
        #print ispdb

        n_search = int(360/6/8/0.3+1)
        angles = 2730.7*np.linspace(float(ang1)/n_steps,float(ang2)/n_steps,num=n_search)
        spdf = [0 for i in range(n_search)]
        for i in range(n_search):
            toSend = [int(angles[i]),int(2**13)]
            xb_send(0, command.CALIBRATE_MOTOR, pack('2h', *toSend))
            time.sleep(1.0)
            spdf[i] = float(shared.bytesIn)/2**16
            xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
            time.sleep(0.2)
        
        print(angles.tolist())
        print(spdf)
        '''


        # Evaulate accuracy of calibration --------------
        #'''
        # This should be between 0 and 2^14/6 = 2730.7
        # The calibration is good if the leg moves quickly outwards.
        toSend = [1900,int((2**11))] # reterminated motor 1
        #toSend = [2018,int(2**13)] # stock motor 1
        xb_send(0, command.CALIBRATE_MOTOR, pack('2h', *toSend))
        time.sleep(params.duration / 1000.0)
        #'''

                
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
