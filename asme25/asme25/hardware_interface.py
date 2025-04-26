import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from asme25_msgs.msg import Motor as MotorMsg, SorterServo as SorterServoMsg, Marble as MarbleMsg, TimeOfFlight as TimeOfFlightMsg
from asme25_msgs.srv import Reset as ResetSrv, Servo as ServoSrv
from std_msgs.msg import Int8 as Int8Msg, Bool as BoolMsg, UInt16 as UInt16Msg

from adafruit_blinka.board.raspberrypi import raspi_5

import board
import busio
import asyncio
from adafruit_tcs34725 import TCS34725
from adafruit_tca9548a import TCA9548A
from adafruit_pca9685 import PCA9685
from adafruit_vl53l4cd import VL53L4CD # Used on .5 inch, 345
from adafruit_vl6180x import VL6180X # Used on .25 inch, 012
from digitalio import DigitalInOut, Direction, Pin, Pull

import numpy as np
from time import sleep

class ColorSensor:
    def __init__(self, whiteLightPin: Pin, blueLightPin: Pin, offset: float, i2c: busio.I2C):
        self.sensor = TCS34725(i2c)
        self.sensor.gain = 16
        self.sensor.integration_time = 300

        self.whiteLight = DigitalInOut(whiteLightPin)
        self.blueLight = DigitalInOut(blueLightPin)

        self.whiteLight.direction = Direction.OUTPUT
        self.blueLight.direction = Direction.OUTPUT

        self.offset = offset

    def setBlueLightOn(self, on: bool):
        # We provide ground for the blue LED, so setting it to false turns it on
        self.blueLight.value = not on
    
    def setWhiteLightOn(self, on: bool):
        self.whiteLight.value = on
        
    def get_rgb(self):
        was_active = self.sensor.active
        self.sensor.active = True
        was_valid = self.sensor._valid()
        
        if not was_valid:
            return (0, 0, 0, -1)
        
        data = tuple(
            self.sensor._read_u16(reg)
            for reg in (
                0x16,
                0x18,
                0x1A,
                0x14,
            )
        )
        
        self.sensor.active = was_active
        
        r, g, b, clear = data
        if clear == 0:
            return (0, 0, 0, -2)

        # Each color value is normalized to clear, to obtain int values between 0 and 255.
        # A gamma correction of 2.5 is applied to each value as well, first dividing by 255,
        # since gamma is applied to values between 0 and 1
        red = int(pow((int((r / clear) * 256) / 255), 2.5) * 255)
        green = int(pow((int((g / clear) * 256) / 255), 2.5) * 255)
        blue = int(pow((int((b / clear) * 256) / 255), 2.5) * 255)

        # Handle possible 8-bit overflow
        red = min(red, 255)
        green = min(green, 255)
        blue = min(blue, 255)
        return (red, green, blue, 0)

class Servo:
    all_servos = []
    step = 10
    speed_delay = {
        "fast": .001,
        "medium": .005,
        "slow": .01
    }

    def __init__(self, pin: int, namedPositions, pca: PCA9685):
        self.pin = pin
        self.named_positions = namedPositions
        Servo.all_servos.append(self)
        self.last_position = self.get_pos("home")
        self.pca = pca

    def get_pos(self, name: str) -> int:
        pos = self.named_positions[name]
        if type(pos) is int:
            return pos
        else:
            return self.get_pos(pos)
        
    def move_manual(self, pos: int):
        self.last_position = pos
        print(f"Moving servo to {pos}")
        self.pca.channels[self.pin].duty_cycle = pos
    
    def move(self, position: str):
        self.move_manual(self.get_pos(position))

    def move_smooth(self, position: str, speed: str):
        goal = self.get_pos(position)
        print(f"servo.move_smooth, diff is {goal - self.last_position}")
        step = Servo.step
        if goal < self.last_position:
            step = -step
        
        for pos in range(self.last_position + step, goal + step, step):
            self.move_manual(pos)
            sleep(Servo.speed_delay[speed])

    def move_all(position: str):
        for servo in Servo.all_servos:
            servo.move(position)

class Motor:
    all_motors = []

    def __init__(self, pwm_pin: int, drive_pin: int, pca: PCA9685):
        self.pwm_pin = pwm_pin
        self.drive_pin = drive_pin
        self.last_direction = -1
        self.pca = pca

        Motor.all_motors.append(self)
        
    def set_speed(self, speed: int, direction: int):
        self.pca.channels[self.drive_pin].duty_cycle = direction
        self.last_direction = direction
        self.pca.channels[self.pwm_pin].duty_cycle = speed

    def stop(self):
        self.pca.channels[self.pwm_pin].duty_cycle = 0

    def stop_all(pca: PCA9685):
        for motor in Motor.all_motors:
            pca.channels[motor.pwm_pin].duty_cycle = 0

class LimitSwitch:
    # wire one side of limit switch to ground, other side to gpioPin
    def __init__(self, gpio_pin: int):
        self.button = DigitalInOut(gpio_pin)
        self.button.direction = Direction.INPUT
        self.button.pull = Pull.UP

    def is_pressed(self):
        return not self.button.value

class Sorter:
    # For both servos on the motor:
    # - pin is the pin it's connected to on the PCA
    # - left is the degrees to move the servo to send marbles left
    # - right is the degrees to move the servo to send marbles right
    def __init__(self, pin1: int, left1: int, right1: int, pin2: int, left2: int, right2: int, pca:PCA9685):
        self.one = Servo(pin1, {"home": "right", "left": left1, "right": right1}, pca)
        self.two = Servo(pin2, {"home": "right", "left": left2, "right": right2}, pca)

    # Move the sorter to a bin between 1 and 3. One is the leftmost, 3 is the rightmost.
    def setBin(self, bin_num: int):
        match bin_num:
            case 1:
                self.one.move("left")
            case 2:
                self.one.move("right")
                self.two.move("left")
            case 3:
                self.one.move("right")
                self.two.move("right")


#object creation
#wwerrVertical = Motor(0, 1)
#wwerrHorizontal = Servo(11, {"home": "min", "min": 7250, "max": 9700, "shakingForward": 8300, "shakingBack": 7600}) #tuned 4/10
#wwerrAngle = Servo(15, {"home": "holding", "holding": 4100, "dumping": 5000, "shaking": 4700, "tiltUp":3800}) #tuned 4/10
                  
class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        self.I2C = busio.I2C(board.SCL, board.SDA)
        self.TCA = TCA9548A(self.I2C)
        self.PCA = PCA9685(self.I2C)
        self.PCA.frequency = 60
        
        if True: # half inch sorting
            self.tmr_half_color_sensor = self.create_timer(1/30, self.half_color_sensor_tmr_callback)
            self.half_color_sensor_state = 0
            self.half_color_sensor_state_entered = self.get_clock().now()
            self.half_color = ColorSensor(whiteLightPin=raspi_5.D26, blueLightPin=raspi_5.D16, offset=33.331479324459327, i2c=self.TCA[7])
            self.half_pub = self.create_publisher(MarbleMsg, "half/new_marble", 10)
            self.half_sorter = Sorter(3, 5700, 6550, 6, 5000, 6500, self.PCA)
            self.half_tof = self.create_publisher(TimeOfFlightMsg, "half/tof", 10)

        if True: # quarter inch sorting
            self.tmr_quarter_color_sensor = self.create_timer(1/30, self.quarter_color_sensor_tmr_callback)
            self.quarter_color_sensor_state = 0
            self.quarter_color_sensor_state_entered = self.get_clock().now()
            self.quarter_color = ColorSensor(whiteLightPin=raspi_5.D6, blueLightPin=raspi_5.D5, offset=33.331479324459327, i2c=self.TCA[6])
            self.quarter_pub = self.create_publisher(MarbleMsg, "quarter/new_marble", 10)
            self.quarter_sorter = Sorter(14, 7450, 8200, 12, 6500, 7600, self.PCA)
            self.quarter_tof = self.create_publisher(TimeOfFlightMsg, "quarter/tof", 10)

        if True: # solenoid stuff 
            self.solenoid_sub = self.create_subscription(Int8Msg, "robot_joints/solenoid_commands", self.solenoid_msg_callback, 10)
            self.solenoid_control_tmr = self.create_timer(1/30, self.solenoid_tmr_callback)
            self.solenoid_states = [0, 0]
            self.solenoids = [DigitalInOut(raspi_5.D25), DigitalInOut(raspi_5.D24)]
            self.solenoid_times = [self.get_clock().now(), self.get_clock().now()]
            for sol in self.solenoids:
                sol.direction = Direction.OUTPUT
        
        if True: # wwerr stuff
            self.bottom_limit_switch = LimitSwitch(raspi_5.D22)
            self.top_limit_switch = LimitSwitch(raspi_5.D23)
            self.bottom_limit_switch_publisher = self.create_publisher(BoolMsg, "robot_joints/bottom_limit_switch", 10)
            self.top_limit_switch_publisher = self.create_publisher(BoolMsg, "robot_joints/top_limit_switch", 10)
            self.limit_switch_timer = self.create_timer(.001, self.limit_switch_timer_callback)
            self.wwerr_vertical = Motor(0, 1, self.PCA)
            self.wwerr_horizontal = Servo(9, {"home": "min", "min": 7500, "max": 9700, "shakingForward": 8300, "shakingBack": 7600}, self.PCA) #tuned 4/10
            self.wwerr_angle = Servo(15, {"home": "holding", "holding": 3500, "dumping": 5000, "flat":3900}, self.PCA) #tuned 4/10

        self.funnel = Motor(4, 5, self.PCA)

        self.sorting_servo_sub = self.create_subscription(SorterServoMsg, "robot_joints/sorter_servo_commands", self.sorter_servo_callback, 10)
        self.reset_service = self.create_service(ResetSrv, "robot_joints/reset", self.reset_service_handler)
        self.servo_service = self.create_service(ServoSrv, "robot_joints/servo_commands", self.servo_service_handler)
        self.barrier_sub = self.create_subscription(UInt16Msg, "robot_joints/barrier_speed", self.barrier_speed_callback, 10)

        print("Started")

    def servo_service_handler(self, req, res):
        print("Servo service handler called")
        match req.name:
            case "wwerr_angle":
                servo = self.wwerr_angle
            case "wwerr_horizontal":
                servo = self.wwerr_horizontal
            case _:
                print(f"Sent servo command to nonexistant servo {req.name}")
                return
        
        servo.move_smooth(req.position, req.speed)

        print("Servo service handler done")
        return res
    
    def motor_callback(self, msg):
        if msg.name == "wwerr_vertical":
            self.wwerr_vertical.set_speed(msg.speed, msg.direction)
            # #double triple check that we don't move while bin is extended
            # if wwerrHorizontal.lastPosition == wwerrHorizontal.getPos("min") or wwerrHorizontal.lastPosition == wwerrHorizontal.getPos("home"):
            #     wwerrVertical.setSpeed(msg.speed, msg.direction)
            # else:
            #     print("Attempted to move wwerrVertical while bin was extended")
        elif msg.name == "funnel":
            self.funnel.set_speed(msg.speed, msg.direction)
        else:
            print(f"Sent motor command for nonexistant motor {msg.name}")
        
    def sorter_servo_callback(self, msg: SorterServoMsg):
        if msg.name == "halfInch":
            self.half_sorter.setBin(msg.bin)
        elif msg.name == "quarterInch":
            self.quarter_sorter.setBin(msg.bin)
        else:
            print(f"Sent sorter servo command for nonexistant sorter servo {msg.name}")

    def half_color_sensor_tmr_callback(self):
        match self.half_color_sensor_state:
            case 0:
                self.half_color.setBlueLightOn(True)
                self.half_color.setWhiteLightOn(False)

                self.half_color_sensor_state += 1
                self.half_color_sensor_state_entered = self.get_clock().now()
            case 1:
                if self.get_clock().now() - self.half_color_sensor_state_entered > Duration(seconds=1):
                    self.half_color_sensor_state += 1
                    self.half_color_sensor_state_entered = self.get_clock().now()
            case 2:
                _, _, blue, valid = self.half_color.get_rgb()
                print(f"half - blue - {blue}")
                if valid < 0: return
                
                if blue >= 100:
                    self.get_logger().debug("Half - No Marble")
                    self.half_color_sensor_state = 0
                    self.half_color_sensor_state_entered = self.get_clock().now()
                elif blue >= 40:
                    # Publish Nylon somehow
                    self.get_logger().debug("Half - Nylon")
                    self.half_pub.publish(MarbleMsg(kind=MarbleMsg.NYLON))
                    self.half_color_sensor_state = 6
                else:
                    self.half_color_sensor_state += 1
                    self.half_color_sensor_state_entered = self.get_clock().now()
            case 3:
                self.half_color.setBlueLightOn(False)
                self.half_color.setWhiteLightOn(True)
                self.half_color_sensor_state += 1
                self.half_color_sensor_state_entered = self.get_clock().now()
            case 4:
                if self.get_clock().now() - self.half_color_sensor_state_entered > Duration(seconds=1):
                    self.half_color_sensor_state += 1
                    self.half_color_sensor_state_entered = self.get_clock().now()
            case 5: 
                rgb = self.half_color.get_rgb()
                
                if rgb[3] < 0: return
                
                Y = (0.299 * rgb[0]) + (0.587 * rgb[1]) + (0.114 * rgb[2])
                Pb = 0.564 * (rgb[2] - Y)
                Pr = 0.713 * (rgb[0] - Y)
                
                normal = np.array([ 0.31656525, -0.96532631,  0.81247134 ])
                
                decision = normal @ np.array([Y, Pr, Pb]) - self.half_color.offset
                print(f"Half - decision {decision}")
                if decision < 0:
                    self.half_pub.publish(MarbleMsg(kind=MarbleMsg.BRASS))
                else:
                    self.half_pub.publish(MarbleMsg(kind=MarbleMsg.STEEL))

                self.half_color_sensor_state = 6
                
            case 6:
                pass

    def quarter_color_sensor_tmr_callback(self):
        match self.quarter_color_sensor_state:
            case 0: #set lights/reset
                self.quarter_color.setBlueLightOn(True)
                self.quarter_color.setWhiteLightOn(False)

                self.quarter_color_sensor_state += 1
                self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 1: #wait 0.3 seconds
                if self.get_clock().now() - self.quarter_color_sensor_state_entered > Duration(seconds=1.5):
                    self.quarter_color_sensor_state += 1
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 2: #check for blue
                _, _, blue, valid = self.quarter_color.get_rgb()
                if valid < 0: return
                if blue >= 140:
                    self.get_logger().debug("Quarter - No Marble")
                    self.quarter_color_sensor_state = 0
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
                elif blue >= 65:
                    # Publish Nylon somehow
                    self.get_logger().debug("Quarter - Nylon")
                    self.quarter_pub.publish(MarbleMsg(kind=MarbleMsg.NYLON))
                    self.quarter_color_sensor_state = 6
                else:
                    self.quarter_color_sensor_state += 1
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 3: #set lights for sensing metals
                self.quarter_color.setBlueLightOn(False)
                self.quarter_color.setWhiteLightOn(True)
                self.quarter_color_sensor_state += 1
                self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 4: #wait 0.3 seconds
                if self.get_clock().now() - self.quarter_color_sensor_state_entered > Duration(seconds=1):
                    self.quarter_color_sensor_state += 1
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 5: #detect metals
                rgb = self.quarter_color.get_rgb()
                
                if rgb[3] < 0: return
                
                print("made it here")

                Y = (0.299 * rgb[0]) + (0.587 * rgb[1]) + (0.114 * rgb[2])
                Pb = 0.564 * (rgb[2] - Y)
                Pr = 0.713 * (rgb[0] - Y)
                
                normal = np.array([ 0.31656525, -0.96532631,  0.81247134 ])
                decision = normal @ np.array([Y, Pr, Pb]) - self.quarter_color.offset

                if decision < 0:
                    self.quarter_pub.publish(MarbleMsg(kind=MarbleMsg.BRASS))
                else:
                    self.quarter_pub.publish(MarbleMsg(kind=MarbleMsg.STEEL))

                self.quarter_color_sensor_state = 6
                
            case 6: #exit state
                pass
    
    def solenoid_msg_callback(self, msg: Int8Msg):
        sol_num = msg.data
        if sol_num not in [0, 1]: return
        if self.solenoid_states[sol_num] == 0:
            self.solenoid_states[sol_num] = 1  

    def solenoid_tmr_callback(self):
        for i, state in enumerate(self.solenoid_states):
            match state:
                case 0: pass
                case 1:
                    self.solenoids[i].value = True
                    self.solenoid_states[i] += 1
                    self.solenoid_times[i] = self.get_clock().now()
                case 2:
                    if self.get_clock().now() - self.solenoid_times[i] > Duration(seconds=0.3):
                        self.solenoid_states[i] += 1
                case 3:
                    self.solenoids[i].value = False
                    self.solenoid_states[i] += 1
                    self.solenoid_times[i] = self.get_clock().now()

                case 4:
                    if self.get_clock().now() - self.solenoid_times[i] > Duration(seconds=0.3):
                        self.solenoid_states[i] += 1
                case 5:
                    match i:
                        case 0:
                            self.half_color_sensor_state = 0
                            self.half_color_sensor_state_entered = self.get_clock().now()
                        case 1:
                            self.quarter_color_sensor_state = 0
                            self.quarter_color_sensor_state_entered = self.get_clock().now()
                    self.solenoid_states[i] = 0

    def barrier_speed_callback(self, msg: UInt16Msg):
        self.PCA.channels[10].duty_cycle = msg.data

    def reset_service_handler(self, _, response):
        print("-- RESETTING ---")
        Motor.stop_all(self.PCA)
        print("- Stopped motors")

        if (not self.bottom_limit_switch.is_pressed()):
            print("- Bottom limit switch isn't pressed, pulling in wwerr-wwerr...")
            self.wwerr_angle.move("holding")
            self.wwerr_horizontal.move("min")
            # Let the horizontal box move back in before we start moving down
            start = self.get_clock().now()
            while self.get_clock().now() < start + Duration(seconds=3):
                rclpy.spin_once(self, timeout_sec=0)
            self.wwerr_vertical.set_speed(0x8000, MotorMsg.DOWN)
        
        while (not self.bottom_limit_switch.is_pressed()):
            rclpy.spin_once(self, timeout_sec=0)
        print("- wwerr-wwerr at bottom")
        
        self.wwerr_vertical.stop()
        self.wwerr_angle.move("flat", "slow")
        self.wwerr_horizontal.move_smooth("max", "slow")

        print("--- RESET COMPLETE ---")
        return response
    
    def limit_switch_timer_callback(self):
        top = BoolMsg()
        top.data = self.top_limit_switch.is_pressed()
        self.top_limit_switch_publisher.publish(top)
        btm = BoolMsg()
        btm.data = self.bottom_limit_switch.is_pressed()
        self.bottom_limit_switch_publisher.publish(btm)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HardwareInterface())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
