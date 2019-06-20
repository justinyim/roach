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
    #  [ Kp , Kd , other , Kp , Kd , other , Kp , Kd , other , other ]
    motorgains = [100,80,0, 150,120,0, 120,15,0,0]
    motorgains = [0,0,0, 0,0,0, 0,0,0,0]

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

        viconTest = [0,0,0,0,0,0,50*256,80*256]#55*256,70*256]
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


        '''
        # Swing-up pendulum test
        exp = [2]
        arbitrary = [0]

        angle = [3667*3.14159]

        #motorgains = [0,0,0, 0,0,0, 200,12,0,12]
        motorgains = [0,0,0, 0,0,0, 200,12,0,12]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        viconTest = [0,0,0,0,0,0,0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *angle))
        time.sleep(0.01)

        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.01)

        #modeSignal = [7]
        modeSignal = [32]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.1)
        '''


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


        #'''
        # Balance on toe test
        #Start robot 0: wall jump, 1: single jump, 2: vicon jumps
        exp = [2]
        arbitrary = [0]

        motorgains = [40,20,0, 70,120,0, 0,0,0,0] #[50,25,0, 180,140,0, 0,0,0,0]
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

        adjust = [0,64,-256] # 3667 ticks per radian, yaw, roll, pitch (64 ticks per degree)
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.01)

        #modeSignal = [3]
        modeSignal = [23]#[19]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.01)

        viconTest = [0,0,0, 0,3667*-1*3.14159/180,0, 3*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.01)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        time.sleep(0.01)
        time.sleep(3.0)

        #motorgains = [200,0,22,0,0, 0,0,0,0,0]
        #motorgains = [130,0,13,0,5, 0,0,0,0,0]
        #motorgains = [110,0,12,0,5, 0,0,0,0,0]
        motorgains = [40,20,0, 70,120,0, 160,12,0,0] #[50,25,0, 180,140,0, 160,12,0,12]
        #motorgains = [0,0,0, 0,0,0, 160,12,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))

        #time.sleep(15.0)

        countDown = 3
        for x in range(countDown):
            print countDown-x
            time.sleep(1.0)
            #viconTest = [0,0,0, 0,3667*(3-6*(x%2))*3.14159/360,0, 3*256,0*256]#55*256,70*256]
            #xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))

        '''
        # Balance controller sinusoidal tilt
        fiftiethsOfASecond = 500
        a = 2*3.14159/180*938.7
        w = 0.5*2*3.14159
        for x in range(fiftiethsOfASecond):
            # ud is in 2^15/(2000*pi/180)~=938.7 ticks/rad
            tiltCmd = [a/w*np.sin(w*x/50), a*np.cos(w*x/50), -a*w*np.sin(w*x/50)]
            xb_send(0, command.TILT, pack('3h', *tiltCmd))
            print tiltCmd
            time.sleep(0.02)
        tiltCmd = [0, 0, 0]
        xb_send(0, command.TILT, pack('3h', *tiltCmd))
        time.sleep(0.02)
        tiltCmd = [0, 0, 0]
        xb_send(0, command.TILT, pack('3h', *tiltCmd))
        '''

        #'''
        # Balance control tilt once to a*tau**2 radians
        a = 10.0#23.4375#
        tau = 0.1#0.08#
        fiftiethsOfASecond = 100
        for x in range(fiftiethsOfASecond):
            t = x/50.0

            if t > 6.0*tau:
                t = 12.0*tau-t;
                flip = -1.0
            else:
                flip = 1.0

            if t < 0.0:
                Mdd = 0.0
                Md = 0.0
                M = 0.0
            elif t < tau:
                Mdd = -a*t
                Md = -1.0/2.0*a*t**2.0
                M = -1.0/6.0*a*t**3.0
            elif t < 3.0*tau:
                tr = t - tau
                Mdd = -a*tau + a*tr
                Md = -1.0/2.0*a*tau**2.0 - a*tau*tr + 1.0/2.0*a*tr**2.0
                M = -1.0/6.0*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr - 1.0/2.0*a*tau*tr**2.0 + 1.0/6.0*a*tr**3.0
            elif t < 4.0*tau:
                tr = t - 3*tau
                Mdd = a*tau
                Md = -1.0/2.0*a*tau**2.0 + a*tau*tr
                M = (-2.0+1.0/6.0)*a*tau**3.0 - 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0
            elif t < 5.0*tau:
                tr = t - 4.0*tau
                Mdd = a*tau - a*tr
                Md = 1.0/2.0*a*tau**2.0 + a*tau*tr - 1.0/2.0*a*tr**2.0
                M = (-2.0+1.0/6.0)*a*tau**3.0 + 1.0/2.0*a*tau**2.0*tr + 1.0/2.0*a*tau*tr**2.0 - 1.0/6.0*a*tr**3.0;
            else:
                tr = t - 5.0*tau
                Mdd = 0.0
                Md = a*tau**2.0
                M = -a*tau**3.0 + a*tau**2.0*tr

            tiltCmd = [flip*M*938.7, Md*938.7, flip*Mdd*938.7]
            xb_send(0, command.TILT, pack('3h', *tiltCmd))
            print tiltCmd
            time.sleep(0.02)

        tiltCmd = [0, 0, 0]
        xb_send(0, command.TILT, pack('3h', *tiltCmd))
        time.sleep(0.02)
        tiltCmd = [0, 0, 0]
        xb_send(0, command.TILT, pack('3h', *tiltCmd))
        #'''

        #'''

        '''
        # Foot buzzing
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


        time.sleep(params.duration / 1000.0)
        #time.sleep(0.5)
        
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
