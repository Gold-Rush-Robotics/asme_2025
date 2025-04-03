import rclpy
import sensor_msgs 
from rclpy.node import Node

from std_msgs.msg import String, Int64
from sensor_msgs.msg import JointState
from asme25_msgs.msg import Servo , Marbles

from enum import Enum



SERVO_PATHS = {
    #paths are temp right now
    Marbles.BRASS: [Servo.LEFT, Servo.LEFT, Servo.LEFT],
    Marbles.NYLON: [Servo.LEFT, Servo.LEFT, Servo.LEFT],
    Marbles.STEEL: [Servo.LEFT, Servo.LEFT, Servo.LEFT]
}


# state of marbles
# find marble path
# communicate to serrvos - correct position
class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')
        self.servo_commands_publisher = self.create_publisher(Servo, 'servo_commands', 10)
        self.joint_state_subscriber = self.create_subscription(JointState, 'robot_joints/states', self.joint_state_callback, 10)
        self.marble_subscriber = self.create_subscription(Marbles, 'marbles', self.marble_callback,  10)
        self.hmi_start_stop = self.create_subscription(String, 'hmi_start_stop', self.on_start, 10)
        self.solenoid_command_publsiher = self.create_publisher(Int64, 'solenoid_commands', 10)
    
    def marble_callback(self, msg):
        """"
        when we get marbles we need to check for new marbles
        """
        marble = msg[0]
        path = SERVO_PATHS[marble]
        self.servo_commands_publisher.publish(path.value)
        # TODO: Change 1 to the ID of the solenoid to move
        self.solenoid_command_publsiher.publish(1)
        

    def joint_state_callback(self, msg):
    
    
        pass
    

    def on_start(self, msg: String) -> None:
        """
        Processes start messages sent by the HMI    
        """
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