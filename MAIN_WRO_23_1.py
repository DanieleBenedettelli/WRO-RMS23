from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase, GyroDriveBase
from pybricks.tools import wait, StopWatch

from Grabber import Grabber
from HSVColorSensor import HSVColorSensor
from Robot import Robot

WHEEL_DIAM = 56 # mm/s
WHEEL_DIST = 96 # mm/s
TRAVEL_SPEED = 600 # mm/s
TRAVEL_ACC = 1200 # mm/s/s
SPIN_SPEED = 360 # deg/s
SPIN_ACC = 360 # deg/s/s

TRAVEL_SPEED_SLOW = 300 # mm / s
TRAVEL_ACC_SLOW = 600 
SPIN_SPEED_SLOW = 180
SPIN_ACC_SLOW = 180

sensorBlocks = HSVColorSensor(Port.A)
sensorLine = ColorSensor(Port.E) # right
sensorIntersections = ColorSensor(Port.C) # left

motorRight = Motor(Port.F)
motorLeft = Motor(Port.B, Direction.COUNTERCLOCKWISE)
grabber = Grabber(Port.D)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections, sensorBlocks, grabber)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)

# settings for line following
#robot.lineFollowerSettings(speed=90, target=45, gain=0.55, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )
colorTones = {Color.BLUE:2000, Color.GREEN:1000, Color.WHITE:4000, None: 500}
colorChars = {Color.BLUE:'B', Color.GREEN:'G', Color.WHITE:'W', None: 'X'}
timer = StopWatch()

def aspettaIlVia():
    robot.hub.display.char(">")
    robot.hub.light.blink(Color.GREEN, [300, 500])
    pressed = []
    while not any(pressed):
        pressed = robot.hub.buttons.pressed()
        wait(10)
    robot.hub.light.on(Color.BLUE)  
    robot.hub.display.off()

# esci da base, leggi ordine, prendi barca grande e riforniscila
def leggiOrdineRifornisciBarca():
  global order1, order2
  robot.settings(TRAVEL_SPEED_SLOW, TRAVEL_ACC_SLOW, SPIN_SPEED_SLOW, SPIN_ACC_SLOW)
  
  robot.straightUntilLine(black_thr=20, maxSpeed=50) 
  robot.straight(20)
  robot.lineFollowerSettings(basePower=40, target=60, gain=0.1, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )
  robot.followLineForDistance(DISTANZA_ORDINE)
  
  # leggi blocco ordine 1
  order1 = sensorBlocks.getRobustColor()
  robot.hub.light.on(order1)
  robot.hub.display.char(colorChars[order1])
  print("color1: "+str(order1))
  robot.hub.speaker.beep(frequency=colorTones[order1])

  robot.followLineForDistance(48)

  order2 = sensorBlocks.getRobustColor()
  robot.hub.light.on(order1)
  robot.hub.display.char(colorChars[order1])
  print("color2: "+str(order2))
  robot.hub.speaker.beep(frequency=colorTones[order2])

  #print("follow until X")
  robot.followLineUntilIntersection(thr=20, basePower = 50)
  wait(100)
    
  #print("reset gyro")
  robot.hub.imu.reset_heading(0) # UNICO RESET DA FARE

  #print("vai indietro")
  robot.straight(distance=-180)

  #print("curva S")
  robot.Scurve(dx=65, dy=280,speed=120)
  
  #robot.straightGyroForDistance(distance=DISTANZA_PER_RIFORNIMENTO, maxSpeed=150)
  robot.straight(DISTANZA_PER_RIFORNIMENTO)
  robot.straight(10) 
  robot.straight(-10) 

# curva con la barca verso i container
def curvaVersoContainer():
  robot.settings(TRAVEL_SPEED_SLOW, TRAVEL_ACC_SLOW, SPIN_SPEED_SLOW, SPIN_ACC_SLOW)
  robot.turnSpeed = 90

  robot.curve(angle=45, radius=WHEEL_DIST/2)
  robot.headTo(angle=45)
 
  robot.straight(DISTANZA_AVVICINAMENTO_CONTAINER) # determina avvicinamento ai container
  robot.curve(angle=45, radius=WHEEL_DIST/2)  
  robot.headTo(angle=HEADING_CONTAINER)

  robot.straightGyroForDistance(distance=50,maxSpeed=80, absoluteHeading=True,headingOffset=HEADING_CONTAINER) 
  #robot.turnSpeed = 270

def testGrabber():
  robot.grabber.prepareForGrabbing()
  while True:
    if Button.RIGHT in robot.hub.buttons.pressed():
        robot.grabContainer()
        while Button.RIGHT in robot.hub.buttons.pressed():
          wait(5)

def testContainerColor() :
  while Button.RIGHT not in robot.hub.buttons.pressed():
    color=sensorBlocks.getColor(longRange=False, printHSV=True)
    print("************* ", color)
    wait(500)

def prendiTuttiContainer(c1=Color.GREEN, c2=Color.BLUE):
    robot.settings(TRAVEL_SPEED_SLOW, TRAVEL_ACC_SLOW, SPIN_SPEED_SLOW, SPIN_ACC_SLOW)
    grabber.prepareForGrabbing()
    done = False

    robot.resetContainerLogic()
    heading0 = robot.readGyro() # should be -90 deg
    #heading0 = HEADING_CONTAINER
    print("heading to keep:", heading0)

    pos0 = robot.distance() # per misurare distanza massima percorsa e fermarsi
    MAX_DISTANCE = 400
    while not done and (robot.distance()-pos0)<MAX_DISTANCE: # 55cm per trovare i container colorati
        #avanza finché non vedi un colore
        print("searching...")
        distanceLeft = MAX_DISTANCE - (robot.distance()-pos0)
        robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.BLUE, Color.GREEN], headingToKeep=heading0, maxDistance = distanceLeft)
        robot.hub.speaker.beep()
        #wait(100)
        seenColor = sensorBlocks.getRobustColor(longRange=False)
        print("container color:", seenColor)
        done = robot.manageContainer(orderColor1=c1, orderColor2=c2, containerColorSeen=seenColor, headingToKeep=heading0)

    print("presi tutti colorati:", done)
    grabber.retract() # rilascia eventuali container incastrati sotto
    robot.straightGyroForDistance(distance=DISTANZA_PRIMA_CONTAINER_BIANCHI, maxSpeed = 120, steerGain = 4, absoluteHeading=True, headingOffset=heading0) 
    grabber.prepareForGrabbing(False)

    found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE], headingToKeep=heading0, maxDistance=270, longRange=True)
    if found == 1:
        robot.hub.speaker.beep()
        robot.manageWhiteContainers(heading0)
        found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE],headingToKeep=heading0,  maxDistance=80, longRange=True)
    if found == 1:
        robot.hub.speaker.beep()
        robot.manageWhiteContainers(heading0)

def barcaGrandeFuori():
  robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)
  robot.hub.imu.reset_heading(HEADING_CONTAINER) # TODO remove, it's just for debug from here
  robot.grabber.retract()
  robot.headTo2(angle=50, radius=-WHEEL_DIST/2, speedMax=100) #angle was 45

  # TODO rendi più robusto andare verso la linea, indipendentemente dall'aver preso i container bianchi o meno
  robot.straight(100)
  robot.straightUntilLine()
  robot.straight(35) # was 50

  # sterza imperniato a sx finché non vede la linea
  robot.stop() 
  motorLeft.hold()
  motorRight.run(130)
  print("prima: ",motorRight.angle())
  while sensorLine.reflection()<60:
    wait(10)
  while sensorLine.reflection()>20:
    wait(10)
  while sensorLine.reflection()<60:
    wait(100)    
  motorRight.brake()
  print("dopo: ",motorRight.angle())
  #motorRight.run_angle(speed=100,rotation_angle=60)
  motorLeft.stop()
  
  #robot.headTo(angle=HEADING_CONTAINER)

  #robot.headTo2(radius=140, angle=0, speedMax=90,absoluteHeading=False) # TODO manovra critica

  robot.lineFollowerSettings(basePower=60, target=35, gain=0.1, darkThreshold = 15)
  robot.followLineForDistance(distance=280,basePower=70, brake=False)

  #robot.lineFollowerSettings(basePower=100, target=35, gain=0.1, darkThreshold = 15)
  robot.followLineUntilIntersection(basePower=40)
  robot.gyro.reset_angle(0)
  robot.straight(73)# allinea anello container rosso con gancio gru
  robot.straight(-220)
  robot.spin(45)
  robot.straight(220)
  robot.headTo(0)
  robot.straight(150)
  robot.headTo(90)
  
  robot.lineFollowerSettings(basePower=60, target=40, gain=0.1, darkThreshold = 15, whichSensor=Side.LEFT, whichBorder=Side.RIGHT)
  robot.followLineForDistance(distance = DISTANZA_GRU,basePower=50)
  #robot.straightGyroForDistance(distance=DISTANZA_GRU,maxSpeed=200,absoluteHeading=False) # avvicinamento alla gru

  #print("indietro")
  robot.drive(-180,0)
  while (sensorIntersections.reflection()>30 or sensorLine.reflection()>30):
    #print("R: ", sensorIntersections.reflection())
    #print("L: ", sensorLine.reflection())
    wait(5)
  robot.straight(0)
  #print("avanti")  
  robot.straightGyroForDistance(distance=DISTANZA_DOPO_GRU, maxSpeed = 200, absoluteHeading=False) # regola posizione sensore su bordo linea
  robot.headTo(180)

def barcaPiccola():
  robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)

  robot.lineFollowerSettings(basePower=70, target=30, gain=0.1, darkThreshold = 15, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineForDistance(distance=480 , basePower=40, brake=False)

  robot.lineFollowerSettings(speed=70, target=30, gain=0.1, darkThreshold = 15, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineUntilIntersection(basePower=40,brake=False)

  robot.followLineForDistance(distance=480 , basePower=70, brake=False)  
  robot.followLineUntilIntersection(basePower=40, brake=True)

  wait(100)
  robot.gyro.reset_angle(180)

  robot.followLineForDistance(430,basePower=60)
  wait(100)

  # arco per prendere barca
  robot.curve(radius=93, angle=-180)
  #robot.arc(radius=93, angle=180, speed=100 )  # radius era 95
  
  robot.straightGyroForDistance(600, maxSpeed=200, absoluteHeading=True)

  robot.straightUntilLine(maxSpeed=150)
  
  robot.headTo(-45)# TODO debug

  robot.straight(150)
  robot.straightUntilLine(maxSpeed=150)
  robot.straight(140)
  
  robot.curve(angle=45 radius=WHEEL_DIST/2)
  #robot.arc(angle=-45,radius=-WHEEL_DIST/2,speed=90)

  robot.lineFollowerSettings(basePower=50, target=40, gain=0.1,darkThreshold=10,whichSensor=Side.LEFT,whichBorder=Side.RIGHT)
  robot.followLineForDistance(distance=300, basePower= 70, brake=False) 
  robot.followLineUntilIntersection(basePower=50)
  #robot.arc(radius=100, angle=90, speed=140)
  robot.curve(radius=100, angle=-90)

  robot.straight(20)
  # carica barca piccola
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)
  robot.straight(-48)
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)

  robot.straight(230) # park, end of mission

"""
  __  __    _    ___ _   _ 
 |  \/  |  / \  |_ _| \ | |
 | |\/| | / _ \  | ||  \| |
 | |  | |/ ___ \ | || |\  |
 |_|  |_/_/   \_\___|_| \_|
                           
"""
# DATA: 05/05/2023
# Dopo la vittoria con la versione MINDSTORMS EV3 PyBricks a Romecup, 
# eseguito porting a SPIKE Prime
# VERSIONE 1.1

# avoid stopping everything with a slight touch of the CENTER button
#robot.hub.system.set_stop_button(None) 

order1 = Color.BLUE
order2 = Color.GREEN

HEADING_CONTAINER = 90 #gradi
# tutte distanze in millimetri

DISTANZA_ORDINE = 144
DISTANZA_PER_RIFORNIMENTO = 185 
DISTANZA_AVVICINAMENTO_CONTAINER = 275 #275 # senza rilevamento banchina
DISTANZA_PRIMA_CONTAINER_BIANCHI = 290 #250
DISTANZA_GRU = 390 
DISTANZA_DOPO_GRU = 200 

robot.grabber.calibrate()

#testContainerColor() # se sbaglia a leggere i container
#testGrabber() # per testare la pinza

robot.hub.speaker.play_notes(notes=['C5/8_', 'E5/8_', 'G5/4'],tempo=200)
robot.resetGyro()

aspettaIlVia()

timer.reset()

grabber.unloadBuffer()

grabber.retract(False)

leggiOrdineRifornisciBarca() 
print("BARCA RIFORNITA @ "+str(timer.time()/1000))

curvaVersoContainer()
print("ATTRACCO @ "+str(timer.time()/1000))

prendiTuttiContainer(order1, order2)
print("PRESI CONTAINER @ "+str(timer.time()/1000))

barcaGrandeFuori()
print("BARCA GRANDE FUORI @ "+str(timer.time()/1000))

barcaPiccola()
print("MISSIONE COMPLETATA @ "+str(timer.time()/1000))

