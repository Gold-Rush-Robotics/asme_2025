import rclpy
import sensor_msgs 
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

# state of marbles
# find marble path
# communicate to serrvos - correct position
class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')
        self.joint_commands_publisher = self.create_publisher(JointState, 'robot_joints/commands', 10)
        
        self.joint_state_subscriber = self.create_subscription(JointState, 'robot_joints/states', 10)
        self.hmi_start_stop = self.create_subscription(String, 'hmi_start_stop', self.on_start, 10)
    

    """
    Processes st messages sent by the HMI
    """
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