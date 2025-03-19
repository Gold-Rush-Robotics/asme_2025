import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

#raspberry pi imports
import board
import busio
import adafruit_pca9685

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)


"""
read sensor i2c data
publish joint state messages
publish joint trajectory messages
"""
class ActuatorInterface(Node):

    def __init__(self):
        super().__init__('actuator_interface')

        #receives commands from game_controller
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            'robot_joints/commands',
            self.joint_trajectory_callback,
            10)
        
        #publish states of motors that we have access to
        self.joint_state_pub = self.create_publisher(JointState, "robot_joints/state", 10)
        
    def joint_trajectory_callback(self, msg):
        #translate joint trajectory messages into pwm controls to pca9685
        names = msg.joint_names
        joints = msg.points

        #set frequency example: pca.frequency = 60

        pass

        


def main(args=None):
    rclpy.init(args=args)

    node = ActuatorInterface()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()