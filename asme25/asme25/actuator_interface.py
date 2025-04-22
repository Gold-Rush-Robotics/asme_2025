import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import JointState
from asme25_msgs.msg import Servo as ServoMsg, Motor as MotorMsg, SorterServo as SorterServoMsg

#raspberry pi imports
import adafruit_pca9685
import adafruit_tca9548a
import board
import busio

from time import sleep
import signal
from enum import Enum
from gpiozero import Button, OutputDevice

I2C = busio.I2C(board.SCL, board.SDA)
TCA = adafruit_tca9548a.TCA9548A(I2C)
PCA = adafruit_pca9685.PCA9685(TCA[0])
PCA.frequency = 60

SOLENOID_OUT_TIME = 0.2

class Servo:
    allServos = []
    step = 10
    speedDelay = {
        "fast": .001,
        "medium": .005,
        "slow": .01
    }

    def __init__(self, pin: int, namedPositions):
        self.pin = pin
        self.namedPositions = namedPositions
        Servo.allServos.append(self)
        self.lastPosition = self.getPos("home")

    def getPos(self, name: str) -> int:
        pos = self.namedPositions[name]
        if type(pos) is int:
            return pos
        else:
            return self.getPos(pos)
        
    def moveManual(self, pos: int):
        self.lastPosition = pos
        PCA.channels[self.pin].duty_cycle = pos
    
    def move(self, position: str):
        self.moveManual(self.getPos(position))

    def moveSmooth(self, position: str, speed: str):
        goal = self.getPos(position)
        step = Servo.step
        if goal < self.lastPosition:
            step = -step
        
        for pos in range(self.lastPosition + step, goal + step, step):
            self.moveManual(pos)
            sleep(Servo.speedDelay[speed])

    def moveAll(position: str):
        for servo in Servo.allServos:
            servo.move(position)

class Motor:
    allMotors = []

    def __init__(self, pwmPin: int, drivePin: int):
        self.pwmPin = pwmPin
        self.drivePin = drivePin
        self.lastDirection = -1

        Motor.allMotors.append(self)
        
    def setSpeed(self, speed: int, direction: int):
        PCA.channels[self.drivePin].duty_cycle = direction
        self.lastDirection = direction
        PCA.channels[self.pwmPin].duty_cycle = speed

    def stop(self):
        PCA.channels[self.pwmPin].duty_cycle = 0

    def stopAll():
        for motor in Motor.allMotors:
            PCA.channels[motor.pwmPin].duty_cycle = 0

class LimitSwitch:
    # wire one side of limit switch to ground, other side to gpioPin
    def __init__(self, gpioPin: int):
        self.button = Button(gpioPin)

    def isPressed(self):
        return self.button.is_pressed

class Sorter:
    # For both servos on the motor:
    # - pin is the pin it's connected to on the PCA
    # - left is the degrees to move the servo to send marbles left
    # - right is the degrees to move the servo to send marbles right
    def __init__(self, pin1: int, left1: int, right1: int, pin2: int, left2: int, right2: int):
        self.one = Servo(pin1, {"home": "right", "left": left1, "right": right1})
        self.two = Servo(pin2, {"home": "right", "left": left2, "right": right2})

    # Move the sorter to a bin between 1 and 3. One is the leftmost, 3 is the rightmost.
    def setBin(self, binNum: int):
        match binNum:
            case 1:
                self.one.move("left")
            case 2:
                self.one.move("right")
                self.two.move("left")
            case 3:
                self.one.move("right")
                self.two.move("right")

class Solenoid:
    # outTime: How long (in seconds) to extend the solenoid for when `activate` is called
    def __init__(self, gpioPin):
        self.outputDevice = OutputDevice(gpioPin)

    def activate(self):
        self.outputDevice.on()
        
    def deactivate(self):
        self.outputDevice.off()

bottomLimit = LimitSwitch(22)
topLimit = LimitSwitch(23)

# Vertical: The motor that moves the marbles up/down the elevator
# Horizontal: The servo that pulls the marble tray from its starting position to the elevator
# Angle: The servo that dumps the marble tray into the engineezy funnel
wwerrVertical = Motor(0, 1)
wwerrHorizontal = Servo(11, {"home": "min", "min": 7250, "max": 9700, "shakingForward": 8300, "shakingBack": 7600}) #tuned 4/10
wwerrAngle = Servo(15, {"home": "holding", "holding": 4100, "dumping": 5000, "shaking": 4700, "tiltUp":3800}) #tuned 4/10

funnel = Motor(4, 5)

barrier = Servo(10, {"home": 0x8888, "forward": 0xFFFF, "backward": 100})

halfInchSolenoid = Solenoid(24)
quarterInchSolenoid = Solenoid(25)

halfInchSorter = Sorter(12, 5500, 6800, 13, 5100, 6200)
# TODO: Add quarter-inch sorter when we have a quarter-inch track



"""
read sensor i2c data
publish joint state messages
publish joint trajectory messages
"""
class ActuatorInterface(Node):
    def __init__(self):
        super().__init__('actuator_interface')

        self.servoCommandsSub = self.create_subscription(ServoMsg, 'robot_joints/servo_commands', self.onServoMsg, 10)
        self.servoStatesPub = self.create_publisher(ServoMsg, 'robot_joints/servo_states', 10)

        self.sorterServoCommandsSub = self.create_subscription(SorterServoMsg, 'robot_joints/sorter_servos', self.onSorterServoMsg, 10)

        self.motorCommandsSub = self.create_subscription(MotorMsg, 'robot_joints/motors', self.onMotorMsg, 10)
        self.motorStatesPub = self.create_publisher(MotorMsg, 'robot_joints/motor_states', 10)


        # Receives a "reset" command that moves everything in the robot back to its initial state
        self.resetSub = self.create_subscription(Empty, "robot_joints/reset", self.onResetMsg, 1)

        # Receives commands for solenoids. Solenoid control uses a sleep, so we put it on a separate
        # topic that will run concurrently and not block the rest of the code.
        self.solenoidCommandsSub = self.create_subscription(Int8, "robot_joints/solenoid_commands", self.onSolenoidCommand, 10)
        self.halfSolenoidTimer = self.create_timer(SOLENOID_OUT_TIME, self.halfSolenoidDeactivate, autostart=False)
        self.quarterSolenoidTimer = self.create_timer(SOLENOID_OUT_TIME, self.quarterSolenoidDeactivate, autostart=False)

        self.checkLimitSwitchesTimer = self.create_timer(.001, self.checkLimitSwitches)

    def onMotorMsg(self, msg):
        if msg.name == "wwerrVertical":
            #double triple check that we don't move while bin is extended
            if wwerrHorizontal.lastPosition == wwerrHorizontal.getPos("min") or wwerrHorizontal.lastPosition == wwerrHorizontal.getPos("home"):
                wwerrVertical.setSpeed(msg.speed, msg.direction)
            else:
                print("Attempted to move wwerrVertical while bin was extended")
        elif msg.name == "funnel":
            funnel.setSpeed(msg.speed, msg.direction)
        else:
            print(f"Sent motor command for nonexistant motor {msg.name}")

    def onServoMsg(self, msg):
        #publish a message to update the states of the servo
        stateMsg = ServoMsg()
        stateMsg.position = msg.position


        if msg.name == "wwerrAngle":
            wwerrAngle.moveSmooth(msg.position, msg.speed)
            stateMsg.name = "wwerrAngle"
            self.servoStatesPub.publish(stateMsg)

        elif msg.name == "wwerrHorizontal":
            wwerrHorizontal.moveSmooth(msg.position, msg.speed)
            stateMsg.name = "wwerrHorizontal"
            self.servoStatesPub.publish(stateMsg)


        #barrier servo is a special case because it is a continuous servo
        elif msg.name == "barrier":
            print("moving barrier")
            PCA.channels[10].duty_cycle = 10000
            #change back to 2.8 once we're actually running
            sleep(0.5)
            PCA.channels[10].duty_cycle = 0
        else:
            print(f"Sent servo command for nonexistant servo {msg.name}")

    def onSorterServoMsg(self, msg):
        if msg.name == "halfInch":
            halfInchSorter.setBin(msg.bin)
        elif msg.name == "quarterInch":
            pass # TODO
        else:
            print(f"Sent sorter servo command for nonexistant sorter servo {msg.name}")

    def onSolenoidCommand(self, msg: Int8):
        solenoid = msg.data
        if solenoid == 0:
            halfInchSolenoid.activate()
            self.halfSolenoidTimer.reset()
        elif solenoid == 1:
            quarterInchSolenoid.activate()
            self.quarterSolenoidTimer.reset()
        else:
            print("WARNING sent command to activate nonexistant solenoid. Half=0, Quarter=1")

    def halfSolenoidDeactivate(self):
        halfInchSolenoid.deactivate()
    def quarterSolenoidDeactivate(self):
        quarterInchSolenoid.deactivate()


    def onResetMsg(self, _: Empty):
        print("-- RESETTING ---")
        Motor.stopAll()


        if(not bottomLimit.isPressed()):
            print("limit switch isn't pressed")
            wwerrAngle.move("tiltUp")
            wwerrAngle.move("holding")
            wwerrHorizontal.move("min")
            sleep(3) # Let the horizontal box move back in before we start moving down
            wwerrVertical.setSpeed(0x8000, MotorMsg.DOWN)
        
        while(not bottomLimit.isPressed()):
            # print("waiting on limit switch")
            sleep(0.001)
        
        print("at bottom limit")
        wwerrVertical.stop()

        wwerrHorizontal.moveSmooth("max", "slow")

        print("--- RESET COMPLETE ---")

    def checkLimitSwitches(self):
        msg = MotorMsg()
        msg.name = "wwerrVertical"

        if bottomLimit.isPressed():
            if wwerrVertical.lastDirection == MotorMsg.DOWN:
                wwerrVertical.stop()
            msg.direction = MotorMsg.DOWN
            self.motorStatesPub.publish(msg)
        elif topLimit.isPressed():
            if wwerrVertical.lastDirection == MotorMsg.UP:
                wwerrVertical.stop()
            msg.direction = MotorMsg.UP
            self.motorStatesPub.publish(msg)



def main(args=None):
    try:
        rclpy.init(args=args)
        node = ActuatorInterface()
        rclpy.spin(node)
        
        rclpy.shutdown()

    except(KeyboardInterrupt):
        print("Stopping All")
        TCA[0].unlock()

        Motor.stopAll()

        #stop barrier puller
        PCA.channels[10].duty_cycle = 0



if __name__ == '__main__':
    main()