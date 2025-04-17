import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Int8
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

#raspberry pi imports
import board
import busio
import adafruit_PCA9685
from time import sleep

I2C = busio.I2C(board.SCL, board.SDA)
PCA = adafruit_PCA9685.PCA9685(I2C)
PCA.frequency = 60

class Servo:
    allServos = []
    step = 10

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

    def moveSmooth(self, position: str, delay: int):
        print(f"Moving servo to {position}")

        goal = self.getPos(position)
        step = Servo.step
        if goal < self.lastPosition:
            step = -step
        
        for pos in range(self.lastPosition + step, goal, step):
            self.moveManual(pos)
            sleep(delay)

    def moveAll(position: str):
        for servo in Servo.allServos:
            servo.move(position)

class Motor:
    allMotors = []

    def __init__(self, pwmPin: int, drivePin: int, speeds, directions):
        self.pwmPin = pwmPin
        self.drivePin = drivePin
        self.speeds = speeds
        self.directions = directions

        Motor.allMotors.append(self)
        
    def setSpeed(self, speed: str, direction: str):
        PCA.channels[self.drivePin].duty_cycle = self.directions[direction]
        PCA.channels[self.pwmPin].duty_cycle = self.speeds[speed]

    def stop(self):
        PCA.channels[self.pwmPin].duty_cycle = 0

    def stopAll(self):
        for motor in Motor.allMotors:
            PCA.channels[motor.pwmPin].duty_cycle = 0

class LimitSwitch:
    # wire one side of limit switch to ground, other side to gpioPin
    def __init__(self, gpioPin: int):
        self.button = Button(gpioPin)

    def isPressed(self):
        return self.button.is_pressed

class Solenoid(Motor):
    # outTime: How long (in seconds) to extend the solenoid for when `activate` is called
    def __init__(self, pin1: int, pin2: int, outTime: int):
        Motor(pin1, pin2, {"on": 0xFFFF, "off": 0}, {"on": 0})
        self.outTime = outTime

    def activate(self):
        self.setSpeed("on", "on")
        sleep(self.outTime)
        self.setSpeed("off", "on")

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




"""
read sensor i2c data
publish joint state messages
publish joint trajectory messages
"""
class ActuatorInterface(Node):
    def __init__(self):
        super().__init__('actuator_interface')

        # Receives commands from game_controller
        self.jointCommandsSub = self.create_subscription(
            JointTrajectory,
            'robot_joints/commands',
            self.onJointCommand,
            10
        )
        # Publishes states of motors that we have access to
        self.jointStatePub = self.create_publisher(JointState, "robot_joints/state", 10)
        # Receives a "reset" command that moves everything in the robot back to its initial state
        self.resetSub = self.create_subscription(Empty, "robot_joints/reset", self.onResetMsg, 1)
        # Receives commands for solenoids. Solenoid control uses a sleep, so we put it on a separate
        # publisher that will run concurrently and not block the rest of the code.
        self.solenoidCommandsSub = self.create_subscription(Int8, "robot_joints/solenoid_commands", 10);
        
    def onResetMsg(self, _: Empty):
        pass
    
    def onJointCommand(self, msg):
        #translate joint trajectory messages into pwm controls to PCA9685
        names = msg.joint_names
        joints = msg.points

        #set frequency example: PCA.frequency = 60

        pass

    def onSolenoidCommand(self, msg: Int8):
        solenoid = msg.data

        


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorInterface()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()