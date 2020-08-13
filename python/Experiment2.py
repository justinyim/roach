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

      # # Balance control tilt once to 9/4*a*T^2 rad and 1/2*a*T rad/s
      a = 0.001#20#-25.0# angular acceleration (rad/s^2) 4 for vertical jump on branch
      b = 0.1 # for falling forward
      T = 0.63#0.55#0.65#0.07#0.05# # time scale (s)
      motorExtend = 48#50 #76 # radians
      extraPushOff = 5
      t_motor = 0.17# seconds

      k1 = 0
      k2 = -15
      airRetract = (motorExtend-80.0)*0.5 + 60.0
      
      toHop = 1 # make a small jump (1) or not (0)
      toBranch = 1 # jump to branch (1) or not (0)

      rollOff = 0.01#-0.01 0 for jump back onto branch


      # BEGIN -----------------------
      setupSerial()
      queryRobot()

      duration = 2500
      telemetry = True
      repeat = False

      zeroGains = [50,30,0, 80,70,0, 0,0,0,0]

      runTailGains = [50,30,0, 80,40,0, 100,15,0,0]

      standTailGains = [50,30,0, 80,50,0, 100,13,0,0]

      motorgains = [50,30,0, 80,40,0, 100,13,0,0]
      leftFreq = [0.16, 0.2, 0.5, .16, 0.12, 0.25]
      phase = [65, 80] # Raibert leg extension
      rightFreq = runTailGains # thruster gains

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

        exp = [2]
        arbitrary = [0]

        modeSignal = [0]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.02)

        #zeroGains = [0,0,0,0,0, 0,0,0,0,0]
        xb_send(0, command.SET_PID_GAINS, pack('10h',*zeroGains))
        time.sleep(0.02)

        #viconTest = [0,0,0,0,0,0,60*256,80*256]#55*256,70*256]
        viconTest = [0,0,0, 0,3667*rollOff,0, 0*256,0*256]#55*256,70*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(0.02)

        xb_send(0, command.RESET_BODY_ANG, pack('h', *arbitrary))
        time.sleep(0.02)

        xb_send(0, command.GYRO_BIAS, pack('h', *arbitrary))
        time.sleep(0.02)

        xb_send(0, command.G_VECT_ATT, pack('h', *arbitrary))
        time.sleep(0.02)

        adjust = [0,-64,-192]
        xb_send(0, command.ADJUST_BODY_ANG, pack('3h', *adjust))
        time.sleep(0.02)

        modeSignal = [17]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.02)

        xb_send(0, command.START_EXPERIMENT, pack('h', *exp))
        #time.sleep(1.5)
        
        xb_send(0, command.SET_PID_GAINS, pack('10h',*standTailGains))
        time.sleep(0.2) # CCC Added 8/10/2020 was originally 1.0

        viconTest = [0,0,0, 0,0,0, 30*256,30*256]
        xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
        time.sleep(1.0 + 1.5) #CCC Added 8/11/2020 + few seconds before offset estimator disabled

        modeSignal = [16]
        xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
        time.sleep(0.02)

        t0 = time.time()
        t = 0.0
        tEnd = 2.5*T #13.0*T CCC for testing, should be 6.0*T 8/3 or 3.0*T 8/10
        t_hasLaunched = tEnd
        while t < tEnd:
          # Md is in 2^15/(2000*pi/180)~=938.7 ticks/rad
          t = time.time() - t0
		  
		  # Fall forward
          if t < 0.0:
		     Mddd = 0.0
		     Mdd = 0.0
		     Md = 0.0
		     M = 0.0
          elif t < T+0.05:
             tr = t - 0.0
             Mddd = (a*np.exp(tr/b))/b**2
             Mdd = 1/(200*b) + (a*(np.exp(tr/b) - 1))/b
             Md = (tr/200 - a*tr)/b - a + a*np.exp(tr/b) + 1/200
             M = tr/200 - a*tr + a*b*(np.exp(tr/b) - 1) - (tr**2*(a - 1/200))/(2*b)
          else:
             Mddd = 0.0
             Mdd = 0.0
             Md = 0.0
             M = 0.0
          t_launchStart = (T - t_motor)

		  # Main Trajectory
          # if t < 0.0:
            # Mddd = 0.0
            # Mdd = 0.0
            # Md = 0.0
            # M = 0.0
          # elif t < T:
            # tr = t - 0.0
            # Mddd = a*(tr/(2*T) - 1)
            # Mdd = -(a*tr*(4*T - tr))/(4*T)
            # Md = -(a*tr**2*(6*T - tr))/(12*T)
            # M = -(a*tr**3*(8*T - tr))/(48*T)
          # elif t < 3*T:
            # tr = t - T
            # Mddd = -a*(tr/(2*T) - 1)
            # Mdd = (a*tr*(4*T - tr))/(4*T) - (3*T*a)/4
            # Md = - (5*T**2*a)/12 - (a*tr*(3*T - tr)**2)/(12*T)
            # M = - (7*T**3*a)/48 - (a*tr*(9*T*tr + 10*T**2 - 4*tr**2))/24 - (a*tr**4)/(48*T)
          # elif t < (11*T)/2:
            # tr = t - 3*T
            # Mddd = 0
            # Mdd = (T*a)/4
            # Md = (T*a*tr)/4 - (7*T**2*a)/12
            # M = - (71*T**3*a)/48 - (T*a*tr*(14*T - 3*tr))/24
          # elif t < (15*T)/2:
            # tr = t - (11*T)/2
            # Mddd = (a*tr)/(4*T)
            # Mdd = (T*a)/4 + (a*tr**2)/(8*T)
            # Md = (T**2*a)/24 + (a*tr*(6*T**2 + tr**2))/(24*T)
            # M = (a*tr**4)/(96*T) - (69*T**3*a)/32 + (T*a*tr*(T + 3*tr))/24
          # elif t < (17*T)/2:
            # tr = t - (15*T)/2
            # Mddd = (3*a*(tr/T - 1))/2
            # Mdd = (3*T*a)/4 - (3*a*tr*(2*T - tr))/(4*T)
            # Md = (7*T**2*a)/8 + (a*tr**3)/(4*T) + (3*a*tr*(T - tr))/4
            # M = (a*tr*(3*T*tr + 7*T**2 - 2*tr**2))/8 - (45*T**3*a)/32 + (a*tr**4)/(16*T)
          # elif t < (317/36+1)*T:
            # tr = t - (17*T)/2
            # Mddd = 0
            # Mdd = 0
            # Md = (9*T**2*a)/8
            # M = (9*T**2*a*tr)/8 - (11*T**3*a)/32
          # else:
            # Mddd = 0.0
            # Mdd = 0.0
            # Md = 0.0
            # M = 0.0
          # t_launchStart = (317*T/36 - t_motor)

          # if t < 0.0: # balance
          #   Mddd = 0.0
          #   Mdd = 0.0
          #   Md = 0.0
          #   M = 0.0
          # elif t < T: # begin lean back
          #   Mddd = -a
          #   Mdd = -a*t
          #   Md = -1.0/2.0*a*t**2.0
          #   M = -1.0/6.0*a*t**3.0
          # elif t < 5.0*T: # reverse lean toward forward
          #   tr = t - T
          #   Mddd = 1.0/2.0*a
          #   Mdd = -a*T + 1.0/2.0*a*tr
          #   Md = -1.0/2.0*a*T**2.0 - a*T*tr + 1.0/4.0*a*tr**2.0
          #   M = -1.0/6.0*a*T**3.0 - 1.0/2.0*a*T**2.0*tr - 1.0/2.0*a*T*tr**2.0 + 1.0/12.0*a*tr**3.0
          # elif t < 6.0*T: # follow through forward tilt
          #   tr = t - 5.0*T
          #   Mddd = 0.0
          #   Mdd = a*T
          #   Md = -1.0/2.0*a*T**2.0 + a*T*tr
          #   M = -29.0/6.0*a*T**3.0 - 1.0/2.0*a*T**2.0*tr + 1.0/2.0*a*T*tr**2.0
          # elif t < 7.0*T: # slow forward tilt
          #   tr = t - 6.0*T
          #   Mddd = -1.0/2.0*a
          #   Mdd = a*T - 1.0/2.0*a*tr
          #   Md = 1.0/2.0*a*T**2.0 + a*T*tr - 1.0/4.0*a*tr**2.0
          #   M = -29.0/6.0*a*T**3.0 + 1.0/2.0*a*T**2.0*tr + 1.0/2.0*a*T*tr**2.0 - 1.0/12.0*a*tr**3.0;
          # elif t < (7.0+2.1815+2.0)*T: # hold forward tilt
          #   tr = t - 7.0*T
          #   Mddd = 0.0
          #   Mdd = 1.0/2.0*a*T
          #   Md = 5.0/4.0*a*T**2.0 + 1.0/2.0*a*T*tr
          #   M = -24/6*a*T**3.0 + 5.0/4.0*a*T**2.0*tr + 1.0/4.0*T*tr**2.0
          # else:
          #   Mddd = 0.0
          #   Mdd = 0.0
          #   Md = 0.0
          #   M = 0.0

          # t_launchStart = 9.1815*T - t_motor

          if toHop == 1 and t > t_launchStart and t < (t_launchStart + 0.08):
            Mdd = Mdd - 0.5

          # Send tilt command
          tiltCmd = [M*938.7, Md*938.7, Mdd*938.7, Mddd*938.7/2.0]
          for ind in range(4):
            if tiltCmd[ind] > 32767:
              tiltCmd[ind] = 32767
              print 'TOO LARGE'
            elif tiltCmd[ind] < -32768:
              tiltCmd[ind] = -32768
              print 'TOO SMALL'
          xb_send(0, command.TILT, pack('4h', *tiltCmd))
          print tiltCmd
          time.sleep(0.01)

          if t > t_launchStart and toHop == 1: # begin launch
            # Normal
            viconTest = [0,0,0, 0,3667*rollOff,0, motorExtend*256,(motorExtend+extraPushOff)*256]
            xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.01)
            xb_send(0, command.SET_PID_GAINS, pack('10h',*runTailGains))
            time.sleep(0.02)
            toHop = 2
            t_hasLaunched = t

            # # Higher gains
            # modeSignal = [1]#[7]
            # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            # time.sleep(0.01)
              
          if t > (t_hasLaunched + t_motor + 0.12) and toHop == 2: # prepare for landing # CCC 8/11/2020 removed 1.0*T + 0.05 + 0.14
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
            #modeSignal = [32]
            #xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            #time.sleep(0.02)
            #viconTest = [0,0,0, 0,0,-0.52*3667, 35*256,35*256] #airRetract*256,25*256]
            #xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            #time.sleep(0.01)

            # Grab branch with closed loop leg control
            modeSignal = [16 + 32]
            xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
            time.sleep(0.02)
            viconTest = [0,0,0, 0,0,-0.5*3667, 35*256,0*256] #airRetract*256,25*256] #CCC was -0.44*3667
            xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
            time.sleep(0.01)
            cmd = [0,0,0,0,\
            (0.12)*2**16, 0.0*2000, (0)*1024,\
            k1, k2]
            xb_send(0, command.STANCE, pack('9h', *cmd))
            time.sleep(0.01)

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
        
        if toHop and not toBranch:
          # # Make a few bounces, then stop
          # modeSignal = [23]
          # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
          # time.sleep(0.2)
          # viconTest = [0,0,0, 0,0,0, 55*256,15*256]
          # xb_send(0, command.INTEGRATED_VICON, pack('8h', *viconTest))
          # time.sleep(0.01)

          # # Enable if using higher gains
          # time.sleep(0.1)
          # modeSignal = [23]
          # xb_send(0, command.ONBOARD_MODE, pack('h', *modeSignal))
          # time.sleep(0.01)

          # # New leg control
          time.sleep(0.2)
          cmd = [0,0,0,0,\
          (0.12)*2**16, 0.0*2000, (0)*1024,\
          k1, k2]
          xb_send(0, command.STANCE, pack('9h', *cmd))
          time.sleep(0.01)
          xb_send(0, command.SET_PID_GAINS, pack('10h',*standTailGains))
          time.sleep(0.02)
          
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

        time.sleep(0.2) #3.0
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
