import rclpy
from rclpy.node import Node

from std_msgs.msg import String as StringMsg, Empty as EmptyMsg, Int8 as Int8Msg
from asme25_msgs.msg import Servo as ServoMsg, Marble as MarbleMsg, Motor as MotorMsg, SorterServo as SorterServoMsg



HALF_INCH_SOLENOID_ID = 0
QUARTER_INCH_SOLENOID_ID = 1
MARBLE_MSG_TO_BIN = {
    MarbleMsg.BRASS: 0,
    MarbleMsg.NYLON: 1,
    MarbleMsg.STEEL: 1,
}
MARBLE_MSG_TO_NAME = {
    MarbleMsg.BRASS: "brass",
    MarbleMsg.NYLON: "nylon",
    MarbleMsg.STEEL: "steel",
}



class GameController(Node):
    def __init__(self):
        super().__init__('game_controller')

        self.hmiStartStopSub = self.create_subscription(StringMsg, 'hmi_start_stop', self.on_start, 10)
        # self.servoStatesSub = self.create_subscription(ServoMsg, 'robot_joints/servo_states', lambda: (), 10)
        # self.motorStatesSub = self.create_subscription(MotorMsg, 'robot_joints/motor_states', lambda: (), 10)
        self.quarterInchSub = self.create_subscription(MarbleMsg, "quarter/new_marble", self.onQuarterInchMarble, 10)
        self.halfInchSub = self.create_subscription(MarbleMsg, "half/new_marble", self.onHalfInchMarble, 10)

        self.resetPub = self.create_publisher(EmptyMsg, "robot_joints/reset", 10)
        self.servoCommandsPub = self.create_publisher(ServoMsg, 'robot_joints/servo_commands',10)
        self.motorCommandsPub = self.create_publisher(MotorMsg, 'robot_joints/motors',10)
        self.solenoidCommandsPub = self.create_publisher(Int8Msg, 'robot_joints/solenoid_commands', 10)
        self.sorterServoCommandsPub = self.create_publisher(SorterServoMsg, 'robot_joints/sorter_servos', 10)
    
    def moveSolenoid(self, solenoid):
        if(solenoid == 0 or solenoid == 1):
            msg = Int8Msg()
            msg.data = solenoid
            self.solenoidCommandsPub.publish(msg)
        else:
            print("WARNING sent command to activate nonexistant solenoid. Half=0, Quarter=1")
    
    def onQuarterInchMarble(self, msg):
        # print(f"Detected quarter-inch marble {MARBLE_MSG_TO_NAME[msg.kind]}")

        sendMsg = SorterServoMsg()
        sendMsg.name = "quarterInch"
        sendMsg.bin = MARBLE_MSG_TO_BIN[msg.kind]
        self.sorterServoCommandsPub.publish(sendMsg)

        # self.moveSolenoid(QUARTER_INCH_SOLENOID_ID)

    def onHalfInchMarble(self, msg):
        print(f"Detected half-inch marble {MARBLE_MSG_TO_NAME[msg.kind]}")
        
        sendMsg = SorterServoMsg()
        sendMsg.name = "halfInch"
        sendMsg.bin = MARBLE_MSG_TO_BIN[msg.kind]
        self.sorterServoCommandsPub.publish(sendMsg)

        self.moveSolenoid(HALF_INCH_SOLENOID_ID)
        print("Sent commands")

    def on_start(self, msg):
        self.resetPub.publish(EmptyMsg())



def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()