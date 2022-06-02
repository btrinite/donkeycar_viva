from datetime import datetime
import donkeycar as dk
import re
import time
import logging
from donkeycar.parts.actuator import RobocarsHat
from donkeycar.utilities.logger import init_special_logger

mylogger = init_special_logger ("Rx")
mylogger.setLevel(logging.INFO)


class RobocarsHatIn:
    CH3_FEATURE_RECORDandPILOT=0
    CH3_FEATURE_THROTTLEEXP=1

    def __init__(self, cfg):

        self.cfg = cfg
        self.inSteering = 0.0
        self.inThrottle = 0.0
        self.fixThrottle = 0.0
        self.inAux1 = 0.0
        self.inAux2 = 0.0
        self.lastAux1 = -1.0
        self.lastAux2 = -1.0
        self.recording=False
        self.mode = 'user'
        self.lastMode = self.mode
        self.applyBrake = 0

        #Default CH3 feature
        self.ch3Feature = self.CH3_FEATURE_RECORDandPILOT
        if self.cfg.ROBOCARSHAT_CH3_FEATURE == 'throttle_exploration':
            self.ch3Feature = self.CH3_FEATURE_THROTTLEEXP
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

        if self.ch3Feature == self.CH3_FEATURE_RECORDandPILOT :

            if (self.inAux1<-0.5):
                self.recording=True
            if (self.inAux1>0.5):
                self.mode='local_angle'
                user_throttle = self.cfg.ROBOCARSHAT_LOCAL_ANGLE_FIX_THROTTLE

        elif self.ch3Feature == self.CH3_FEATURE_THROTTLEEXP :
            if (self.lastAux1 != self.inAux1) :
                if self.inAux1 > 0.5:
                    self.fixThrottle = min(self.fixThrottle+self.cfg.ROBOCARSHAT_THROTTLE_EXP_INC,self.cfg.ROBOCARSHAT_PWM_IN_THROTTLE_MAX)
                    mylogger.info("CtrlIn Fixed throttle set to {}".format(self.fixThrottle))
                if self.inAux1 < -0.5:
                    self.fixThrottle = max(self.fixThrottle-self.cfg.ROBOCARSHAT_THROTTLE_EXP_INC,self.cfg.ROBOCARSHAT_PWM_IN_THROTTLE_MIN)
                    mylogger.info("CtrlIn Fixed throttle set to {}".format(self.fixThrottle))
            user_throttle = self.fixThrottle
            
        #if switching back to user, then apply brake
        if self.mode=='user' and self.lastMode != 'user' :
            self.applyBrake=10 #brake duration

        self.lastMode = self.mode
        self.lastAux1 = self.inAux1
        self.lastAux2 = self.inAux2
        
        if self.applyBrake>0:
            user_throttle = self.cfg.ROBOCARSHAT_LOCAL_ANGLE_BRAKE_THROTTLE
            self.applyBrake-=1

        return user_throttle

    def update(self):

        while self.on:
            start = datetime.now()
            self.getCommand()
            stop = datetime.now()
            s = 0.01 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        user_throttle = self.processAUxCh ()
        return self.inSteering, user_throttle, self.mode, self.recording

    def run (self):
        self.getCommand()
        user_throttle = self.processAUxCh ()
        return self.inSteering, user_throttle, self.mode, self.recording
    

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping Robocars Hat Controller')
        time.sleep(.5)
