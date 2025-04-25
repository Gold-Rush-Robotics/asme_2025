import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Int8
from asme25_msgs.msg import Servo, Motor, SorterServo

from enum import Enum
import datetime

class Testing(Node):
    def __init__(self):
        super().__init__('testing')
        self.servoCommandsPub = self.create_publisher(Servo, 'robot_joints/servo_commands',10)
        self.servoStatesSub = self.create_subscription(Servo, 'robot_joints/servo_states', self.OnServoState, 10)

        self.motorCommandsPub = self.create_publisher(Motor, 'robot_joints/motors',10)
        self.motorStatesSub = self.create_subscription(Motor, 'robot_joints/motor_states', self.OnMotorState, 10)
        self.resetPub = self.create_publisher(Empty, "robot_joints/reset", 10)

        self.solenoidCommandsPub = self.create_publisher(Int8, 'robot_joints/solenoid_commands', 10)
        #shake nonsense
        self.shakeTimer = self.create_timer(2.0, self.shake, autostart=False)
        self.shakerOut = True
        self.shakerCount = 0
        self.funnelDirection = Motor.DOWN

        self.wwerrAnglePos = ""
        self.wwerrHorizontalPos = ""
        self.wwerrVerticalPos = None

        self.nodeMain()

    def moveServo(self, name, position, speed):
        msg = Servo()
        msg.name = name
        msg.position = position
        msg.speed = speed
        self.servoCommandsPub.publish(msg)

    def moveMotor(self, name, speed, direction):
        msg = Motor()
        msg.name = name
        msg.speed = speed
        msg.direction = direction
        self.motorCommandsPub.publish(msg)

    def moveSolenoid(self, solenoid):
        if(solenoid == 0 or solenoid == 1):
            msg = Int8()
            msg.data = solenoid
            self.solenoidCommandsPub.publish(msg)
        else:
            print("WARNING sent command to activate nonexistant solenoid. Half=0, Quarter=1")

    def OnServoState(self, msg):
        #update the current position of the servos
        if msg.name == "wwerrAngle":
            self.wwerrAnglePos = msg.position
        elif msg.name == "wwerrHorizontal":
            self.wwerrHorizontalPos = msg.position

    def OnMotorState(self, msg):
        if msg.name == "wwerrVertical":
            self.wwerrVerticalPos = msg.direction



    def shake(self):
        if self.shakerOut:
            self.moveServo("wwerrHorizontal", "shakingForward", "fast")
        else:
            self.moveServo("wwerrHorizontal", "shakingBack", "slow")
        
        self.shakerOut = not self.shakerOut
        self.shakerCount += 1

        if self.shakerCount == 3:
            self.shakerCount = 0
            self.moveMotor("funnel", 0x4000, self.funnelDirection)

            if self.funnelDirection == Motor.DOWN:
                self.funnelDirection = Motor.UP
            else:
                self.funnelDirection = Motor.DOWN
            


    def nodeMain(self):
        self.moveSolenoid(0)
        self.moveSolenoid(1)
        # self.resetPub.publish(Empty())

        # #TODO: continue execution once hmi switch is flipped
        # print("waiting for hmi switch")
        # input("press enter once reset sequence is complete")

        # self.moveServo("barrier", "", "")
        # self.moveServo("wwerrHorizontal", "home", "slow")

        # #wait for bin to be in
        # print("waiting for wwerrHorizontal to go home")
        # while(not self.wwerrHorizontalPos == "home"):
        #     #spin once to allow ros to still receive messages
        #     rclpy.spin_once(self)

        # self.moveMotor("wwerrVertical", 0x8000, Motor.UP)

        # #wait for wwerr to be up
        # print("waiting for wwerrVertical to be up")
        # while(not self.wwerrVerticalPos == Motor.UP):
        #     rclpy.spin_once(self)

        # self.moveServo("wwerrAngle", "shaking", "slow")
        # self.moveMotor("funnel", 0x4000, Motor.UP)

        # self.shakeTimer.reset() #this starts the shaking



def main(args=None):
    rclpy.init(args=args)

    node = Testing()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()