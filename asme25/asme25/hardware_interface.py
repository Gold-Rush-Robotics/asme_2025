import rclpy
from rclpy.node import Node
from rclpy.time import Duration

from time import sleep

from adafruit_blinka.board.raspberrypi import raspi_5


from digitalio import DigitalInOut, Direction, Pin

import board
import busio
from adafruit_tcs34725 import TCS34725
from adafruit_tca9548a import TCA9548A
from adafruit_pca9685 import PCA9685

from asme25_msgs.msg import Servo as ServoMsg, Motor as MotorMsg, SorterServo as SorterServoMsg, Marble as MarbleMsg
from std_msgs.msg import Int8

import numpy as np



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
        


class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        I2C = busio.I2C(board.SCL, board.SDA)
        TCA = TCA9548A(I2C)
        PCA = PCA9685(TCA[0])
        PCA.frequency = 60
        
        if True: #half inch sorting
            self.tmr_half_color_sensor = self.create_timer(1/30, self.half_color_sensor_tmr_callback)
            self.half_color_sensor_state = 0
            self.half_color_sensor_state_entered = self.get_clock().now()
            self.half_color = ColorSensor(whiteLightPin=raspi_5.D26, blueLightPin=raspi_5.D16, offset=27.331479324459327, i2c=TCA[7])
            self.half_pub = self.create_publisher(MarbleMsg, "half/new_marble", 10)

        if True: #quarter inch sorting
            self.tmr_quarter_color_sensor = self.create_timer(1/30, self.quarter_color_sensor_tmr_callback)
            self.quarter_color_sensor_state = 0
            self.quarter_color_sensor_state_entered = self.get_clock().now()
            self.quarter_color = ColorSensor(whiteLightPin=raspi_5.D6, blueLightPin=raspi_5.D5, offset=35.331479324459327, i2c=TCA[6])
            self.quarter_pub = self.create_publisher(MarbleMsg, "quarter/new_marble", 10)
        
        if True: # Solenoid stuff 
            self.solinoid_sub = self.create_subscription(Int8, "robot_joints/solenoid_commands", self.solinoid_msg_callback, 10)
            self.solinoid_control_tmr = self.create_timer(1/30, self.solinoid_tmr_callback)
            self.solinoid_states = [0, 0]
            self.solinoids = [DigitalInOut(raspi_5.D25), DigitalInOut(raspi_5.D21)]
            self.solinoid_times = [self.get_clock().now(), self.get_clock().now()]
            for sol in self.solinoids:
                sol.direction = Direction.OUTPUT
        # Servo
        # Limit Switches
        # Motor
        # TOF 
        
    def half_color_sensor_tmr_callback(self):
        print(f"Half state: {self.half_color_sensor_state}")
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
                print(f"Half - {blue} {valid}")
                
                if valid < 0: return
                
                if blue >= 255:
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
                if self.get_clock().now() - self.quarter_color_sensor_state_entered > Duration(seconds=0.3):
                    self.quarter_color_sensor_state += 1
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
            case 2: #check for blue
                _, _, blue, valid = self.quarter_color.get_rgb()
                if valid < 0: return
                if blue >= 255:
                    self.get_logger().debug("Quarter - No Marble")
                    self.quarter_color_sensor_state = 0
                    self.quarter_color_sensor_state_entered = self.get_clock().now()
                elif blue >= 40:
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
                if self.get_clock().now() - self.quarter_color_sensor_state_entered > Duration(seconds=0.3):
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
    
    def solinoid_msg_callback(self, msg:Int8):
        sol_num = msg.data
        if sol_num not in [0, 1]: return
        if self.solinoid_states[sol_num] == 0:
            self.solinoid_states[sol_num] = 1  

    def solinoid_tmr_callback(self):
        for i, state in enumerate(self.solinoid_states):
            match state:
                case 0: pass
                case 1:
                    self.solinoids[i].value = True
                    self.solinoid_states[i] += 1
                    self.solinoid_times[i] = self.get_clock().now()
                case 2:
                    if self.get_clock().now() - self.solinoid_times[i] > Duration(seconds=0.3):
                        self.solinoid_states[i] += 1
                case 3:
                    self.solinoids[i].value = False
                    self.solinoid_states[i] += 1
                    self.solinoid_times[i] = self.get_clock().now()

                case 4:
                    if self.get_clock().now() - self.solinoid_times[i] > Duration(seconds=0.3):
                        self.solinoid_states[i] += 1
                case 5:
                    match i:
                        case 0:
                            self.half_color_sensor_state = 0
                            self.half_color_sensor_state_entered = self.get_clock().now()
                        case 1:
                            self.quarter_color_sensor_state = 0
                            self.quarter_color_sensor_state_entered = self.get_clock().now()
                    self.solinoid_states[i] = 0

                    

                
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HardwareInterface())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
