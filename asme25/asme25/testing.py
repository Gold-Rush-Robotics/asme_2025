import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from asme25_msgs.msg import Servo, Motor, SorterServo

from enum import Enum

#


class Testing(Node):
    def __init__(self):
        super().__init__('testing')
        self.servoCommandsPub = self.create_publisher(Servo, 'robot_joints/servos',10)
        self.motorCommandsPub = self.create_publisher(Motor, 'robot_joints/motors',10)
        self.resetPub = self.create_publisher(Empty, "robot_joints/reset", 10)


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

    def nodeMain(self):
        #self.resetPub.publish(Empty())

        self.moveServo("barrier", "", "")


        # self.moveServo("wwerrHorizontal", "home", "slow")
        # self.moveMotor("wwerrVertical", 0xFFFF, Motor.DOWN)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Testing()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()