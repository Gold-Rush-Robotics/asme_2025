import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')


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