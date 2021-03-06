#!/usr/bin/env python
"""
authors: Justin Yim
testing

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach import *

####### Wait at exit? #######
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x20\x52', xb)
    R1.SAVE_DATA = False
                            
    #R1.RESET = False       #current roach code does not support software reset
    
    shared.ROBOTS.append(R1) #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    # Send resets
    for r in shared.ROBOTS:
        if r.RESET:
            r.reset()
            time.sleep(0.35)
    # Repeat this for other robots
    # TODO: move reset / telem flags inside robot class? (pullin)
    
    # Send robot a WHO_AM_I command, verify communications
    for r in shared.ROBOTS:
        r.query(retries = 3)
    
    #Verify all robots can be queried
    verifyAllQueried()  # exits on failure
    
    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    #motorgains = [1800,0,100,0,0, 1800,0,100,0,0]
    #motorgains = [200,200,0,0,0 , 200,200,0,0,0]
    motorgains = [0,0,200,0,100, 0,0,200,0,100] # JY edits
    
    freq = 4
    simpleAltTripod = GaitConfig(motorgains, rightFreq=freq, leftFreq=freq) # Parameters can be passed into object upon construction, as done here.
    # Puffer operates at frequencies from 3 to 9
    simpleAltTripod.phase = PHASE_180_DEG                             # Or set individually, as here
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    #simpleAltTripod.deltasTime  = [0.25, 0.25, 0.25] # Not current supported by firmware; time deltas are always exactly [0.25, 0.25, 0.25, 0.25]
    
    # Configure intra-stride control
    R1.setGait(simpleAltTripod)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME_MS     = 8000*4/freq #ms
    EXPERIMENT_LEADIN_TIME_MS  = 100  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 100  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_LEADIN_TIME_MS + EXPERIMENT_RUN_TIME_MS + EXPERIMENT_LEADOUT_TIME_MS)
            r.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()
    
    ######## Motion is initiated here! ########
    R1.startTimedRun( EXPERIMENT_RUN_TIME_MS ) #Faked for now, since pullin doesn't have a working VR+AMS to test with
    ######## End of motion commands   ########
   
    #R1.setServo(-0.3) # minimum physical
    #R1.setServo(-0.95) # maximum physical
    #R1.setServo(-0.45) # minimum sprawl
    R1.setServo(-0.6)

    """
    time.sleep(1)
    R1.setServo(-0.45)
    time.sleep(1)
    R1.setServo(-0.95)
    time.sleep(1)
    R1.setServo(-0.6)
    time.sleep(1)
    R1.setServo(-0.8)
    """


    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            raw_input("Press Enter to start telemetry read-back ...")
            r.downloadTelemetry()
    
    if EXIT_WAIT:  #Pause for a Ctrl + C , if desired
        while True:
            time.sleep(0.1)

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
    except Exception as args:
        print "\nGeneral exception from main:\n",args,'\n'
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_exc()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
    finally:
        xb_safe_exit(shared.xb)
