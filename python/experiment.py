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

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    #motorgains = [450,0,20,0,100, 0,0,0,0,0] #[600,0,20,0,0, 100,0,0,0,0]
    #thrustGains = [300,100,300,30,0,40]
    motorgains = [110,0,25,0,0, 0,0,0,0,0]
    thrustGains = [160,0,190, 100,0,150]
    motorgains = [0,0,0,0,0, 0,0,0,0,0]# disable thrusters and tail
    thrustGains = [0,0,0, 0,0,0]

    #motorgains = [150,0,13,0,5, 0,0,0,0,0]
    thrustGains = [250,0,180, 0,50,50]

    xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h', *thrustGains))
    time.sleep(0.01)
    xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('6h', *thrustGains))
    time.sleep(0.01)

    duration = 5000#15000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True#False#
    repeat = False

    manParams = manueverParams(0, 0, 0, 0, 0, 0) # JY edits: added for compatibility
    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)
    setMotorGains(motorgains)

    sj_params = sjParams(300, 800)
    # wj_params = wjParams(-551287, -50000, 1000000, 4941297, 411774)
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

        viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.02)
        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        '''


        '''
        # leg extension test with variable motor gains
        arbitrary = [0]
        legPosition = [30*256, 0.1*65535, 0.002*65535]
        # motor deflection [radians * 256], P gain [65535 * duty cyle/rad], D gain [65535 * duty cyle/(rad/s)]
        xb_send(0, command.SET_MOTOR_POS, pack('3H', *legPosition))
        '''


        #'''
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

        #modeSignal = [7]
        modeSignal = [3]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))

        time.sleep(3.0)
        #motorgains = [200,0,22,0,0, 0,0,0,0,0]
        #motorgains = [130,0,13,0,5, 0,0,0,0,0]
        #motorgains = [110,0,12,0,5, 0,0,0,0,0]
        motorgains = [180,0,20,0,0, 0,0,0,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        #time.sleep(15.0)
        for x in range(10):
            print 10-x
            time.sleep(1.0)

        #'''
        #'''
        #viconTest = [0,0,0,0,0,0,25*256,50*256]#55*256,70*256]
        viconTest = [0,0,0,0,0,0,60*256,90*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        #'''



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
        #'''

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


        #time.sleep(params.duration / 1000.0)
        time.sleep(0.5)
        
        #time.sleep(10)
        stopSignal = [0]
        xb_send(0, command.STOP_EXPERIMENT, pack('h', *stopSignal))
        time.sleep(0.01)
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
