import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from std_msgs.msg import String as StringMsg, Empty as EmptyMsg, Int8 as Int8Msg, Bool as BoolMsg
from asme25_msgs.msg import Servo as ServoMsg, Marble as MarbleMsg, Motor as MotorMsg, SorterServo as SorterServoMsg
from asme25_msgs.srv import Reset as ResetSrv



HALF_INCH_SOLENOID_ID = 0
QUARTER_INCH_SOLENOID_ID = 1
MARBLE_MSG_TO_BIN = {
    MarbleMsg.BRASS: 3,
    MarbleMsg.NYLON: 1,
    MarbleMsg.STEEL: 2,
}
MARBLE_MSG_TO_NAME = {
    MarbleMsg.BRASS: "brass",
    MarbleMsg.NYLON: "nylon",
    MarbleMsg.STEEL: "steel",
}



class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')

        self.hmi_start_stop_sub = self.create_subscription(StringMsg, 'hmi_start_stop', self.on_start_stop, 10)
        self.servo_states_sub = self.create_subscription(ServoMsg, 'robot_joints/servo_states', self.on_servo_move, 10)
        self.motor_states_sub = self.create_subscription(MotorMsg, 'robot_joints/motor_states', self.on_motor_move, 10)
        self.quarter_inch_sub = self.create_subscription(MarbleMsg, "quarter/new_marble", self.on_new_quarter_inch_marble, 10)
        self.half_inch_sub = self.create_subscription(MarbleMsg, "half/new_marble", self.on_new_half_inch_marble, 10)
        self.top_limit_switch_sub = self.create_subscription(BoolMsg, "robot_joints/top_limit_switch", self.on_top_limit_switch, 10)
        self.bottom_limit_switch_sub = self.create_subscription(BoolMsg, "robot_joints/bottom_limit_switch", self.on_bottom_limit_switch, 10)

        self.servo_commands = self.create_publisher(ServoMsg, 'robot_joints/servo_commands',10)
        self.motor_commands = self.create_publisher(MotorMsg, 'robot_joints/motors',10)
        self.solenoid_commands = self.create_publisher(Int8Msg, 'robot_joints/solenoid_commands', 10)
        self.sorter_servo_commands = self.create_publisher(SorterServoMsg, 'robot_joints/sorter_servos', 10)

        self.reset = self.create_client(ResetSrv, "robot_joints/reset")

        self.wwerr_horizontal_pos = ""
        self.wwerr_angle_pos = ""
        self.wwerr_vertical_pos = ""

        self.top_limit_switch_pressed = False
        self.bottom_limit_switch_pressed = False
    
    def on_top_limit_switch(self, msg):
        self.top_limit_switch_pressed = msg.data
    def on_bottom_limit_switch(self, msg):
        self.bottom_limit_switch_pressed = msg.data
    
    def move_solenoid(self, solenoid):
        if(solenoid == 0 or solenoid == 1):
            msg = Int8Msg()
            msg.data = solenoid
            self.solenoid_commands.publish(msg)
        else:
            print("WARNING: sent command to activate nonexistant solenoid. Half=0, Quarter=1")
    def move_servo(self, name, position, speed):
        msg = ServoMsg()
        msg.name = name
        msg.position = position
        msg.speed = speed
        self.servo_commands.publish(msg)
    
    def on_new_quarter_inch_marble(self, msg):
        print(f"Detected quarter-inch marble {MARBLE_MSG_TO_NAME[msg.kind]}")

        send_msg = SorterServoMsg()
        send_msg.name = "quarterInch"
        send_msg.bin = MARBLE_MSG_TO_BIN[msg.kind]
        self.sorter_servo_commands.publish(send_msg)

        self.move_solenoid(QUARTER_INCH_SOLENOID_ID)

    def on_new_half_inch_marble(self, msg):
        print(f"Detected half-inch marble {MARBLE_MSG_TO_NAME[msg.kind]}")
        
        send_msg = SorterServoMsg()
        send_msg.name = "halfInch"
        send_msg.bin = MARBLE_MSG_TO_BIN[msg.kind]
        self.sorter_servo_commands.publish(send_msg)

        self.move_solenoid(HALF_INCH_SOLENOID_ID)
        print("Sent commands")

    def on_servo_move(self, msg):
        match msg.name:
            case "wwerrHorizontal":
                self.wwerr_horizontal_pos = msg.position
            case "wwerrAngle":
                self.wwerr_angle_pos = msg.position
    
    def on_motor_move(self, msg):
        match msg.name:
            case "wwerrVertical":
                self.wwerr_vertical_pos = msg.direction

    def on_start_stop(self, msg):
        self.move_servo("barrier", "", "")
        # Wait for barrier to pull out
        start = self.get_clock().now()
        while self.get_clock().now() < start + Duration(seconds=2):
            rclpy.spin_once(self, timeout_sec=0)
        pass



def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()