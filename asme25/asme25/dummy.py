# Runs dummy services so we can verify that the game controller works without
# running code on the actual robot.

import rclpy
from rclpy.node import Node

from asme25_msgs.srv import Reset as ResetSrv, Servo as ServoSrv

class DummyHardwareInterface(Node):
    def __init__(self):
        super().__init__("dummy_hardware_interface")

        self.dummyServices = []
        for (ty, service) in [
            (ResetSrv, "reset"),
            (ServoSrv, "servo_commands")
        ]:
            self.dummyServices.append(self.create_service(ty, f"robot_joints/{service}", self.make_dummy_service_handler(service)))

    def make_dummy_service_handler(self, name: str):
        def handler(req, res):
            print(f"Dummy handler {name} called")
            return res
        return handler

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DummyHardwareInterface())
    rclpy.shutdown()
