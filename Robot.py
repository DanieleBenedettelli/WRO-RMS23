""" Line Following Gyro navigator

Author  : Daniele Benedettelli
Date    : May  2023
Version : 1.0

"""
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.parameters import Port, Color, Stop, Axis
from pybricks.robotics import DriveBase, GyroDriveBase
from pybricks.tools import wait, StopWatch
from pybricks.hubs import PrimeHub
from Grabber import Grabber

from umath import atan2

class Side:
    LEFT = 0
    RIGHT = 1

class Robot(GyroDriveBase):
    """ This class contains all functionality of DriveBase + our line-following routines

    """
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track, colorSensor1, colorSensor2, blockSensor, grabber ):
        self.sensorRight = colorSensor1 # for line following
        self.sensorLeft = colorSensor2 # for intersections
        self.sensorLine = self.sensorRight
        self.sensorIntersection = self.sensorLeft
        self.blockSensor = blockSensor
        self.target = 50
        self.gain = -0.7
        self.blackThreshold = 20
        self.integralError = 0
        self.travelSpeed = 100 # mm/s
        self.lineFollowerBasePower = self.travelSpeed
        self.turnSpeed = 90
        self.wheelbase = axle_track
        self.wheelDiameter = wheel_diameter
        self.left_motor = left_motor
        self.right_motor = right_motor
        # container logic
        self.slot1 = 0
        self.slot2 = 0
        self.whiteContainerInStock = 0
        self.coloredContainerStock = 0
        self.gotWhiteContainer = 0
        self.grabber = grabber
        # match the orientation of the hub on the robot
        # positive X is forward, positive Y is left, Z is up
        self.hub = PrimeHub(top_side=-Axis.X, front_side=Axis.Y)
        #initialize super class DriveBase
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)        
        self.hub.speaker.volume(100)
        self.hub.system.set_stop_button(None) # avoid to stop everything with a slight touch
        #self.resetGyro()

    def settings(self, straight_speed, straight_acceleration, turn_rate, turn_acceleration) :
        self.travelSpeed = straight_speed
        self.turnSpeed = turn_rate
        super().settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration) 

    def __subang(self,a,b):
        d = a-b
        while d>180:
            d -= 360
        while d<-180:
            d += 360
        return d

    def saturate(self, speed, maxSpeed=1000, minSpeed = 0):
        """ Limit speed in range [-1000,1000] """
        if speed > 0 :
            if speed > maxSpeed:
                speed = maxSpeed
            if speed < minSpeed:
                speed = minSpeed
        elif speed < 0 :
            if speed < -maxSpeed:
                speed = -maxSpeed
            if speed > -minSpeed:
                speed = -minSpeed       
       
        return speed

    def resetGyro(self):
        self.hub.imu.reset_heading(0)
        wait(100)

    def readGyro(self):
        a = self.hub.imu.heading()
        heading = ((a+180)%360-180)
        return heading

    def headTo(self, angle=0, wait=True) :
        h0 = self.hub.imu.heading()
        delta = self.__subang(angle, h0)
        print("heading 0: ", h0)
        print("delta angle: ", delta)        
        self.turn(angle = delta, wait = wait)

    def __sign(self,n):
        if n>0:
            return 1
        elif n<0:
            return -1
        else :
            return 0

    # curve forward to the right, pivot on the right side of the center of rotation
    #robot.curve(radius=WHEEL_DIST, angle=90)

    # curve forward to the left, pivot on the left side of the center of rotation
    #robot.curve(radius=WHEEL_DIST, angle=-90)

    # curve backward to the left, pivot on the right side of the center of rotation
    #robot.curve(radius=-WHEEL_DIST, angle=90)

    # curve backward to the right, pivot on the left side of the center of rotation
    #robot.curve(radius=-WHEEL_DIST, angle=-90)

    def Scurve(self, dx, dy, speed=100):
        # Wolfram Alpha: solve [ y/2=R*sin(a), x/2 = R*(1-cos(a)) ] for R,a
        angle = 2*atan2(dx, dy)*57.296
        radius = (dx*dx+dy*dy)/(4*dx)
        #print("angle: %.2f deg"%angle)
        #print("radius: %.2f mm"%radius)
        self.curve(radius=radius, angle= -angle )
        self.curve(radius= radius, angle= angle )

    def spin(self, angle, stopType=Stop.BRAKE, waitForCompletion=True) :
        self.turn(-angle)# = -angle, then = stopType, wait= waitForCompletion)

    def lineFollowerSettings(self, basePower, target, gain, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT) :
        if whichSensor is Side.LEFT:
            self.sensorLine = self.sensorLeft
            self.sensorIntersection = self.sensorRight
        else :
            self.sensorLine = self.sensorRight
            self.sensorIntersection = self.sensorLeft
        self.lineFollowerBasePower = basePower     

        self.gain = gain
        if whichBorder is Side.LEFT:#whichSensor : #  if same side
            self.gain = -gain
            #print("invert gain", self.gain)
        
        self.blackThreshold = darkThreshold     
        self.target = target       

    def __lineFollowCore(self, basePower=60) :
        e = self.target - self.sensorLine.reflection()
        #print("lnfl e:", e)
        steering = self.gain * e
        self.left_motor.dc( basePower - steering )
        self.right_motor.dc(basePower + steering )
        #self.drive(speed,steer)

    def followLineUntilIntersection(self, thr = 24, basePower = None, brake=True):
        self.integralError = 0
        if basePower is None:
            pwr = self.lineFollowerBasePower 
        else:
            pwr = basePower
        self.stop()
        while self.sensorIntersection.reflection() > thr:
            #print(self.sensorIntersection.reflection())
            self.__lineFollowCore(pwr)
            #wait()
        if brake:    
            self.straight(0) # stop con frenata
        else:
            self.stop()
        
    def followLineForDistance(self, distance, basePower = None, brake=True):
        self.integralError = 0
        if basePower is None:
            pwr = self.lineFollowerBasePower 
        else:
            pwr = basePower        
        self.reset() # reset distance
        while self.distance() < distance :
            self.__lineFollowCore(pwr)
        if brake:
            self.straight(0) # stop con frenata
        else :
            self.stop()

    def straightUntilLine(self, white_thr = 70, black_thr=15, maxSpeed=None, blackLine = True):
        if maxSpeed is None:
            maxSpeed, _ , _ , _ = self.settings()
        self.drive(maxSpeed , 0 )
        if blackLine:
            while self.sensorRight.reflection() < white_thr:
                #print(self.sensorRight.reflection())
                wait(2)
            #self.speaker.beep()    
            while self.sensorRight.reflection() > black_thr:
                #print(self.sensorRight.reflection())
                wait(2)
        else:
            print("look for white")
            while self.sensorLeft.reflection() < white_thr:
                print(self.sensorRight.reflection())
                wait(1)              
            while self.sensorRight.reflection() < white_thr:
                print(self.sensorRight.reflection())
                wait(1)            
        self.straight(0)      

    def straightGyroForDistance(self, distance, maxSpeed = None, steerGain=4.0, driveGain = 3.5, absoluteHeading = True, headingOffset = 0):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        self.reset()
        togo = distance - self.distance()

        if absoluteHeading:
            headingNow = headingOffset
        else:
            headingNow = self.hub.imu.heading()

        #print("heading now: ", headingNow)

        while abs(togo)>1:
            heading = self.hub.imu.heading()
            #print("gyro ", heading)
            gyro_error = heading- headingNow
            togo = distance - self.distance()
            steer = steerGain * gyro_error
            speed = abs(maxSpeed)*self.__sign(distance)
            self.drive(speed,-steer)                
        self.straight(0)
        self.stop()        

    def straightGyroUntilContainer(self,maxSpeed = None, steerGain=4.5, colors =[Color.BLUE, Color.GREEN], longRange = False, headingToKeep=-90, maxDistance=400):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        pos0 = self.distance()
        event = 0
        while event is 0 :
            if self.blockSensor.getColor(longRange) in colors:
                event = 1
            if (self.distance()-pos0) > maxDistance:
                event = -1
            heading = self.readGyro()
            
            #print("gyro ", heading)
            #steerGain = 3
            gyro_error = heading - headingToKeep  # maintain always same heading w.r.t 0
            steer = steerGain * gyro_error
            self.drive(maxSpeed,-steer)                
        self.straight(0)
        self.stop() 
        return event
    
    def resetContainerLogic(self):
        self.slot1 = 0
        self.slot2 = 0
        self.whiteContainerInStock = 0
        self.coloredContainerStock = 0
        self.gotWhiteContainer = 0
    
    def grabContainer(self, offset = 0, heading=-90):
        self.grabber.lift()
        #wait(200)
        #self.straightGyroForDistance(-offset,maxSpeed = 120, headingOffset=heading)
        self.straight(-offset)
        self.grabber.unloadOnRamp()
        self.grabber.prepareForGrabbing()
        wait(200) # wait for container to slide onto the boat
        #self.straightGyroForDistance(offset,maxSpeed = 120,  headingOffset=heading)
        if offset != 0 :
            self.straight(-40) # evita che il rallentatore si blocchi sul container
            self.straight(offset)

    def manageContainer(self, orderColor1, orderColor2, containerColorSeen,headingToKeep):
        DISTANZA_CARICO = 43
        DISTANZA_CONSERVO = 80
        TRAVEL_SPEED = 70
                
        #global slot1, slot2, coloredContainerStock, daParteBianchi, biancoPreso
        if containerColorSeen in [Color.BLUE, Color.GREEN]:
            if orderColor1 == containerColorSeen and self.slot1 == 0:
                print("Slot 1")
                self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED_SLOW, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.slot1 = 1
            elif orderColor2 == containerColorSeen and self.slot2 == 0:
                print("Slot 3")
                self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED_SLOW, headingOffset=headingToKeep)
                self.grabContainer(offset=12*8, heading=headingToKeep)
                self.slot2 = 1
            elif self.coloredContainerStock == 0:
                print("Stock")
                self.straightGyroForDistance(distance=DISTANZA_CONSERVO, maxSpeed=TRAVEL_SPEED_SLOW, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.straightGyroForDistance(-24, maxSpeed=TRAVEL_SPEED_SLOW, headingOffset=headingToKeep)
                self.coloredContainerStock = 1
            else : # skip container 
                self.straightGyroForDistance(distance=26, maxSpeed=TRAVEL_SPEED_SLOW, headingOffset=headingToKeep)

        if self.slot1 == 1 and self.slot2 == 1 and self.coloredContainerStock == 1 :
            return True
        else:
            return False
        
    def manageWhiteContainers(self,headingToKeep):
        DISTANZA_CARICO = 53
        DISTANZA_CONSERVO = 93 
        TRAVEL_SPEED = 70

        if self.gotWhiteContainer == 0:
            self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=18*8,heading=headingToKeep)
            print("Slot 4")
            self.gotWhiteContainer = 1
        else:
            self.straightGyroForDistance(distance=DISTANZA_CONSERVO,maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=0,heading=headingToKeep)
            print("Stock")
                    

