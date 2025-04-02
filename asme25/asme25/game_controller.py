import rclpy
import sensor_msgs 
from rclpy.node import Node

from std_msgs.msg import String, Int64
from sensor_msgs.msg import JointState

from enum import Enum

class Marble(Enum):
    BRASS = 1
    NYLON = 2
    STEEL = 3

class Servo(Enum):
    LEFT = 1
    RIGHT = 2

SERVO_PATHS = {
    # paths are temp right now
    Marble.BRASS: [Servo.LEFT, Servo.LEFT, Servo.LEFT],
    Marble.NYLON: [Servo.LEFT, Servo.LEFT, Servo.LEFT],
    Marble.STEEL: [Servo.LEFT, Servo.LEFT, Servo.LEFT]
}


# state of marbles
# find marble path
# communicate to serrvos - correct position
class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')
        self.servo_commands_publisher = self.create_publisher(Servo, 'servo_commands', 10)
        self.joint_state_subscriber = self.create_subscription(JointState, 'robot_joints/states', 10)
        self.marble_subscriber = self.create_subscription(Marbles, 'marbles', 10)
        self.hmi_start_stop = self.create_subscription(String, 'hmi_start_stop', self.on_start, 10)
        self.solenoid_command_publsiher = self.create_publisher(Int64, 'solenoid_commands', 10)
    
    def marble_callback(self, msg):
        #when we get marbles we need to check for new marbles
        marble = msg[0]
        path = SERVO_PATHS[marble]
        self.servo_commands_publisher.publish(path)
        # TODO: Change 1 to the ID of the solenoid to move
        self.solenoid_command_publsiher.publish(1)

    def joint_state_callback(self, msg):
        pass


    # Processes st messages sent by the HMI

    def on_start(self, msg: String) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)

    node = GameController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()