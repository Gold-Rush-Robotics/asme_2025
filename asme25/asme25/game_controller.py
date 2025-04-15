import rclpy
import sensor_msgs
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# state of marbles
# find marble path
# communicate to serrvos - correct position
class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')
        self.joint_commands_publisher = self.create_publisher(JointState, 'robot_joints/commands', 10)
        
        self.joint_state_subscriber = self.create_subscription(JointState, 'robot_joints/states', 10)
        self.hmi_start_stop = self.create_subscription(String, 'hmi_start_stop', self.on_start, 10)
    
    def on_start(self, msg: str) -> None:
        """
        Processes messages from the `hmi_start_stop` topic.
        
        Args:
            msg (str): The message sent by the HMI. Can currently be either "start" or "stop".
        """
        
        if msg == "start":
            # Run entrypoint for the ASME program
            pass
        if msg == "stop":
            # Stop all tasks happening in the robot right now
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