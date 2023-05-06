from pybricks.pupdevices import Motor
from pybricks.parameters import Direction, Port, Stop
from pybricks.tools import wait, StopWatch


class Grabber(Motor):

    def __init__(self, port=Port.A):
        super().__init__(port)

    def calibrate(self):
        #print("grabber default settings: " ,end="")
        #print(self.control.limits())
        self.control.limits(speed=800, acceleration=600) # default max speed is 1000 deg/s, acc is 4000 deg/s/s
        self.run_time(speed=100, time=200)
        self.run_until_stalled(-250, then=Stop.HOLD, duty_limit=40)
        #wait(100)
        self.stop()
        wait(100)
        self.reset_angle(angle = 0)

    def prepareForGrabbing(self, waitForCompletion = True):
        #self.control.limits(speed=800, acceleration=800)
        self.run_target(speed=500,target_angle=290, then=Stop.BRAKE, wait=waitForCompletion)

    def lift(self, waitForCompletion=True):
        #self.control.limits(speed=800, acceleration=800)
        self.run_target(speed=400,target_angle=90, then=Stop.BRAKE, wait=waitForCompletion)

    def unloadOnRamp(self, waitForCompletion=True):
        self.control.limits(speed=1000, acceleration=700)
        self.run_target(speed=150,target_angle=30, then=Stop.BRAKE, wait=True) # TODO speed was 170
        self.run_target(speed=150,target_angle=100, then=Stop.BRAKE, wait=waitForCompletion)
        self.control.limits(speed=800, acceleration=800)

    def retract(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=1, then=Stop.BRAKE, wait=waitForCompletion)        

    def unloadBuffer(self, waitForCompletion=True):
        self.run_until_stalled(300, then=Stop.HOLD, duty_limit=100)
        #self.run_target(speed=300,target_angle=380, then=Stop.BRAKE, wait=waitForCompletion)  
