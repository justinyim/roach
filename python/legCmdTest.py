#!/usr/bin/env python

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

      # BEGIN -----------------------
      setupSerial()
      queryRobot()

      duration = 2000
      telemetry = True
      repeat = False

      motorgains = [0,0,0, 0,0,0, 0,0,0,0]
      leftFreq = [0.16, 0.2, 0.5, .16, 0.12, 0.25]
      phase = [65, 80] # Raibert leg extension
      rightFreq = motorgains # thruster gains

      manParams = manueverParams(0,0,0,0,0,0)
      params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)

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


        if True:
            exp = [2]
            arbitrary = [0]

            modeSignal = [0]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)

            zeroGains = [0,0,0,0,0, 0,0,0,0,0]
            xb_send(0, command.SET_PID_GAINS, pack('10h',*zeroGains))
            time.sleep(0.02)

            xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
            time.sleep(0.02)

            xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
            time.sleep(0.02)

            xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
            time.sleep(0.02)

            modeSignal = [49]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)

            xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
            time.sleep(0.02)
            

            k1 = -2000#-2000
            k2 = -89#-89
            tEnd = 8
            t0 = time.time()
            t = 0.0
            while t < tEnd:
                t = time.time() - t0
                l = 0.05*(t%2>1) + 0.12
                cmd = [0, 0, 0, 0, \
                l*2**16, 0, (0+9.81)*1024,\
                k1, k2]
                xb_send(0, command.STANCE, pack('9h', *cmd))
                time.sleep(0.02)

            '''
            k1 = -2000
            k2 = -89
            tEnd = 1
            t0 = time.time()
            t = 0.0
            while t < tEnd:
                t = time.time() - t0
                cmd = [0,0,0,0,\
                (t*0.05+0.09)*2**16, 0.0*2000, (0+9.81)*1024,\
                k1, k2]
                xb_send(0, command.STANCE, pack('9h', *cmd))
                time.sleep(0.02)

            tEnd = 8
            l = 0.15
            al = 0.02
            wl = 0.5*2*3.14159

            ap = 2.5*3.14159/180*938.7
            wp = 1*2*3.14159
            t0 = time.time()
            t = 0.0
            while t < tEnd:
                t = time.time() - t0
                cmd = [ap/wp*np.cos(wp*t), -ap*np.sin(wp*t), -ap*wp*np.cos(wp*t), ap*wp*wp*np.sin(wp*t),\
                (l+al*np.sin(wl*t))*2**16, (al*wl*np.cos(wl*t))*2000, (-al*wl*wl*np.sin(wl*t) - 9.81)*1024,\
                k1, k2]
                xb_send(0, command.STANCE, pack('9h', *cmd))
                time.sleep(0.02)
            '''
        else:
            exp = [2]
            arbitrary = [0]

            modeSignal = [0]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)

            zeroGains = [0,0,0,0,0, 0,0,0,0,0]
            xb_send(0, command.SET_PID_GAINS, pack('10h',*zeroGains))
            time.sleep(0.02)

            xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
            time.sleep(0.02)

            xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
            time.sleep(0.02)

            xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
            time.sleep(0.02)

            tEnd = 6
            t0 = time.time()
            t = 0.0
            while t < tEnd:
                t = time.time() - t0
                viconTest = [0,0,0,0,0,0,60*256,60*256]#55*256,70*256]
                xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                time.sleep(1.0)
                viconTest = [0,0,0,0,0,0,30*256,30*256]#55*256,70*256]
                xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
                time.sleep(1.0)



        time.sleep(4.0)
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



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception from main:\n",args,'\n'
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_exc()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
