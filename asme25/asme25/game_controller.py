import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from std_msgs.msg import String as StringMsg, Empty as EmptyMsg, Int8 as Int8Msg, Bool as BoolMsg, UInt16 as UInt16Msg
from asme25_msgs.msg import Marble as MarbleMsg, Motor as MotorMsg, SorterServo as SorterServoMsg
from asme25_msgs.srv import Reset as ResetSrv, Servo as ServoSrv

from asyncio import Future



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


class Job:
    def __init__(self, cb):
        self.cb = cb

    def subscribe(self, msg):
        self.cb(msg)



class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')

        self.hmi_start_stop_sub = self.create_subscription(StringMsg, 'hmi_start_stop', self.on_start_stop, 10)
        self.quarter_inch_sub = self.create_subscription(MarbleMsg, "quarter/new_marble", self.on_new_quarter_inch_marble, 10)
        self.half_inch_sub = self.create_subscription(MarbleMsg, "half/new_marble", self.on_new_half_inch_marble, 10)
        self.top_limit_switch_sub = self.create_subscription(BoolMsg, "robot_joints/top_limit_switch", self.on_top_limit_switch, 10)
        self.bottom_limit_switch_sub = self.create_subscription(BoolMsg, "robot_joints/bottom_limit_switch", self.on_bottom_limit_switch, 10)

        self.motor_commands = self.create_publisher(MotorMsg, 'robot_joints/motors',10)
        self.solenoid_commands = self.create_publisher(Int8Msg, 'robot_joints/solenoid_commands', 10)
        self.barrier = self.create_publisher(UInt16Msg, "robot_joints/barrier_speed", 10)

        self.reset = self.create_client(ResetSrv, "robot_joints/reset")
        self.servo_commands = self.create_client(ServoSrv, "robot_joints/servo_commands")

        self.game_loop_timer = self.create_timer(3, self.game_loop, autostart=False)

        self.top_limit_switch_pressed = False
        self.bottom_limit_switch_pressed = False
        self.game_loop_state = 0

        print("Started")
    
    def ros_sleep(self, secs: float):
        start = self.get_clock().now()
        while self.get_clock().now() < start + Duration(seconds=secs):
            rclpy.spin_once(self, timeout_sec=0)
    def wait_for_future(self, future: Future):
        while not future.done():
            print("Looping while future completes...")
            rclpy.spin_once(self, timeout_sec=0)

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
    def move_servo(self, name: str, position: str, speed: str) -> Future:
        req = ServoSrv.Request()
        req.name = name
        req.position = position
        req.speed = speed
        return self.servo_commands.call_async(req)
    def move_servo_async(self, name: str, position: str, speed: str) -> Future:
        req = ServoSrv.Request()
        req.name = name
        req.position = position
        req.speed = speed
        self.servo_commands.call_async(req)
    def move_motor(self, name: str, speed: int, direction: int):
        msg = MotorMsg()
        msg.name = name
        msg.speed = speed
        msg.direction = direction
        self.motor_commands.publish(msg)
    
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

    def on_start_stop(self, msg):
        """
        - Pull barrier out
        - Sleep for a bit to let barrier move & marbles fall
        - Pull wwerr-wwerr's Legally Distinct Metallic Object:tm: back
        - Wait for pullback to complete
        - Angle wwerr-wwerr up
        - Move wwerr-werr up
        - Stop at limit switch
        - Dump wwerr-wwerr
        - Start game loop
        """
        def stage1():
            print("hi")
            msg = UInt16Msg()
            msg.data = 10_000
            self.barrier.publish(msg)
            self.ros_sleep(2)
            msg = UInt16Msg()
            msg.data = 0
            self.barrier.publish(msg)
            print("wow")
            self.move_servo("wwerr_horizontal", "min", "slow").add_done_callback(stage2)
            print("bruh")

        def stage2():
            self.move_servo("wwerr_angle", "holding", "slow")
            self.move_motor("wwerr_vertical", 0x8000, MotorMsg.UP)
            while not self.top_limit_switch_pressed:
                rclpy.spin_once(self, timeout_sec=0)
            print("neat")

            self.move_motor("funnel", 0x4000, MotorMsg.UP)
            self.move_servo("wwerr_angle", "dumping", "slow")
            self.game_loop_timer.reset()
            print("cool")
    
    def game_loop(self):
        if self.game_loop_state % 2 == 0:
            pos = "max"
            speed = "fast"
        else:
            pos = "min"
            speed = "slow"

        self.move_servo_async("wwerr_horizontal", pos, speed)

        if self.game_loop_state == 2:
            self.move_motor("funnel", 0x4000, MotorMsg.DOWN)
        elif self.game_loop_state == 5:
            self.move_motor("funnel", 0x4000, MotorMsg.UP)
            self.game_loop_state = 0
            return
        
        self.game_loop_state += 1



def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()