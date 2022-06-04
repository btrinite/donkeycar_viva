from datetime import datetime
import donkeycar as dk
import re
import time
import logging
import numpy as np
from donkeycar.parts.actuator import RobocarsHat
from donkeycar.utilities.logger import init_special_logger

mylogger = init_special_logger ("Rx")
mylogger.setLevel(logging.INFO)


class RobocarsHatIn:
    CH3_FEATURE_RECORDandPILOT=0
    CH3_FEATURE_THROTTLEEXP=1
    CH3_FEATURE_STEERINGEXP=2

    def __init__(self, cfg):

        self.cfg = cfg
        self.inSteering = 0.0
        self.inThrottle = 0.0
        self.fixThrottle = 0.0
        self.fixSteering = 0.0
        self.inAux1 = 0.0
        self.inAux2 = 0.0
        self.lastAux1 = -1.0
        self.lastAux2 = -1.0
        self.recording=False
        self.mode = 'user'
        self.lastMode = self.mode
        self.applyBrake = 0

        #CH3 feature
        self.ch3Feature = self.CH3_FEATURE_RECORDandPILOT
        if self.cfg.ROBOCARSHAT_CH3_FEATURE == 'throttle_exploration':
            self.ch3Feature = self.CH3_FEATURE_THROTTLEEXP
        elif self.cfg.ROBOCARSHAT_CH3_FEATURE == 'steering_exploration':
            self.ch3Feature = self.CH3_FEATURE_STEERINGEXP

        if self.cfg.ROBOCARSHAT_THROTTLE_DISCRET != None:
            self.discretesThrottle = np.arange(0.0,1.0001,1.0/len(self.cfg.ROBOCARSHAT_THROTTLE_DISCRET))
            mylogger.info("CtrlIn Discrete throttle thresholds set to {}".format(self.discretesThrottle))

        self.sensor = RobocarsHat(self.cfg)
        self.on = True

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        '''
        Linear mapping between two ranges of values
        '''
        X_range = X_max - X_min
        Y_range = Y_max - Y_min
        XY_ratio = X_range/Y_range

        return ((x-X_min) / XY_ratio + Y_min)

    def getCommand(self):
        l = self.sensor.readline()
        if l != None:
            params = l.split(',')
            if len(params) == 5 and int(params[0])==1 :
                if params[0].isnumeric():
                    self.inThrottle = self.map_range(int(params[1]),
                            self.cfg.ROBOCARSHAT_PWM_IN_THROTTLE_MIN, self.cfg.ROBOCARSHAT_PWM_IN_THROTTLE_MAX,
                        -1, 1)
                if params[2].isnumeric():
                    self.inSteering = self.map_range(int(params[2]),
                        self.cfg.ROBOCARSHAT_PWM_IN_STEERING_MIN, self.cfg.ROBOCARSHAT_PWM_IN_STEERING_MAX,
                        -1, 1)
                if params[3].isnumeric():
                    self.inAux1 = self.map_range(int(params[3]),
                        self.cfg.ROBOCARSHAT_PWM_IN_AUX_MIN, self.cfg.ROBOCARSHAT_PWM_IN_AUX_MAX,
                        -1, 1)
                if params[4].isnumeric():
                    self.inAux2 = self.map_range(int(params[4]),
                        self.cfg.ROBOCARSHAT_PWM_IN_AUX_MIN, self.cfg.ROBOCARSHAT_PWM_IN_AUX_MAX,
                        -1, 1)
                mylogger.debug("CtrlIn {} {} {} {}".format(int(params[1]), int(params[2]), int(params[3]), int(params[4])))

    def processAUxCh(self):
        self.recording=False
        self.mode='user'
        user_throttle = self.inThrottle
        user_steering = self.inSteering

        if self.ch3Feature == self.CH3_FEATURE_RECORDandPILOT :

            if (self.inAux1<-0.5):
                self.recording=True
            if (self.inAux1>0.5):
                self.mode='local_angle'
                user_throttle = self.cfg.ROBOCARSHAT_LOCAL_ANGLE_FIX_THROTTLE

        elif self.ch3Feature == self.CH3_FEATURE_THROTTLEEXP :
            if (abs(self.lastAux1 - self.inAux1)>0.5) :
                if self.inAux1 > 0.5:
                    self.fixThrottle = min(self.fixThrottle+self.cfg.ROBOCARSHAT_THROTTLE_EXP_INC,1.0)
                    mylogger.info("CtrlIn Fixed throttle set to {}".format(self.fixThrottle))
                if self.inAux1 < -0.5:
                    self.fixThrottle = max(self.fixThrottle-self.cfg.ROBOCARSHAT_THROTTLE_EXP_INC,0.0)
                    mylogger.info("CtrlIn Fixed throttle set to {}".format(self.fixThrottle))
            user_throttle = self.fixThrottle

        elif self.ch3Feature == self.CH3_FEATURE_STEERINGEXP :
            if (abs(self.lastAux1 - self.inAux1)>0.5) :
                if self.inAux1 > 0.5:
                    self.fixSteering = min(self.fixSteering+self.cfg.ROBOCARSHAT_STEERING_EXP_INC,1.0)
                    mylogger.info("CtrlIn Fixed steering set to {}".format(self.fixSteering))
                if self.inAux1 < -0.5:
                    self.fixSteering = max(self.fixSteering-self.cfg.ROBOCARSHAT_STEERING_EXP_INC,-1.0)
                    mylogger.info("CtrlIn Fixed steering set to {}".format(self.fixSteering))
            user_steering = self.fixSteering
            
        if self.cfg.ROBOCARSHAT_STEERING_FIX != None:
            user_steering = self.cfg.ROBOCARSHAT_STEERING_FIX

        if self.mode=='user' and self.cfg.ROBOCARSHAT_THROTTLE_DISCRET != None:
            inds = np.digitize(user_throttle, self.discretesThrottle)
            inds = max(inds,1)
            user_throttle = self.cfg.ROBOCARSHAT_THROTTLE_DISCRET[inds-1]

        #if switching back to user, then apply brake
        if self.mode=='user' and self.lastMode != 'user' :
            self.applyBrake=10 #brake duration

        self.lastMode = self.mode
        self.lastAux1 = self.inAux1
        self.lastAux2 = self.inAux2
        
        if self.applyBrake>0:
            user_throttle = self.cfg.ROBOCARSHAT_LOCAL_ANGLE_BRAKE_THROTTLE
            self.applyBrake-=1

        return user_throttle, user_steering

    def update(self):

        while self.on:
            start = datetime.now()
            self.getCommand()
            stop = datetime.now()
            s = 0.01 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        user_throttle, user_steering = self.processAUxCh ()
        return user_steering, user_throttle, self.mode, self.recording

    def run (self):
        self.getCommand()
        user_throttle, user_steering = self.processAUxCh ()
        return user_steering, user_throttle, self.mode, self.recording
    

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping Robocars Hat Controller')
        time.sleep(.5)
