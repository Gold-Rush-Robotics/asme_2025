import busio
import board
import rclpy
import numpy as np

from adafruit_tcs34725 import TCS34725
from adafruit_tca9548a import TCA9548A
from digitalio import DigitalInOut, Direction
from adafruit_blinka.board.raspberrypi import raspi_5
from adafruit_blinka.microcontroller.generic_linux.lgpio_pin import Pin

from rclpy.node import Node
from asme25_msgs.msg import Marble as MarbleMsg

I2C = busio.I2C(board.SCL, board.SDA)
TCA = TCA9548A(I2C)

# How often the marble detectors run, in seconds
MARBLE_DETECT_RATE = 1



class ColorSensor:
    def __init__(self, sensorId: int, whiteLightPin: Pin, blueLightPin: Pin):
        self.sensor = TCS34725(TCA[sensorId])
        self.sensor.gain = 16
        self.sensor.integration_time = 300

        self.whiteLight = DigitalInOut(whiteLightPin)
        self.blueLight = DigitalInOut(blueLightPin)

        self.whiteLight.direction = Direction.OUTPUT
        self.blueLight.direction = Direction.OUTPUT

    def setBlueLightOn(self, on: bool):
        # We provide ground for the blue LED, so setting it to false turns it on
        self.blueLight.value = not on
    def setWhiteLightOn(self, on: bool):
        self.whiteLight.value = on

    def checkMarble(self) -> MarbleMsg:
        msg = MarbleMsg()

        self.setWhiteLightOn(False)
        self.setBlueLightOn(True)
        
        rgb = self.sensor.color_rgb_bytes
        blue = rgb[2]

        if blue >= 255:
            return None
        elif blue >= 40:
            msg.kind = MarbleMsg.NYLON
            return msg

        self.setWhiteLightOn(True)
        self.setBlueLightOn(False)

        rgb = self.sensor.color_rgb_bytes
        Y = (0.299 * rgb[0]) + (0.587 * rgb[1]) + (0.114 * rgb[2])
        Pb = 0.564 * (rgb[2] - Y)
        Pr = 0.713 * (rgb[0] - Y)
        
        normal = np.array([ 0.31656525, -0.96532631,  0.81247134 ])
        
        decision = normal @ np.array([Y, Pr, Pb]) - 27.331479324459327

        if decision < 0:
           msg.kind = MarbleMsg.BRASS
        elif decision > 0:
           msg.kind = MarbleMsg.STEEL
        else:
            print("On the plane! Defaulting to none...")
            return None

        return msg



class MarbleDetector(Node):
    def __init__(self):
        super().__init__("color_sensor")
        self.halfInchMarblePublisher = self.create_publisher(MarbleMsg, "detected_marbles/half_inch", 10)
        self.quarterInchMarblePublisher = self.create_publisher(MarbleMsg, "detected_marbles/quarter_inch", 10)

        self.halfInchSensor = ColorSensor(7, raspi_5.D26, raspi_5.D16)
        self.quarterInchSensor = ColorSensor(6, raspi_5.D6, raspi_5.D5)

        self.halfInchTimer = self.create_timer(MARBLE_DETECT_RATE, self.halfInchCallback)
        self.quarterInchTimer = self.create_timer(MARBLE_DETECT_RATE, self.quarterInchCallback)
    
    def halfInchCallback(self):
        marble = self.halfInchSensor.checkMarble()
        if marble is not None:
            self.halfInchMarblePublisher.publish(marble)
    def quarterInchCallback(self):
        marble = self.quarterInchSensor.checkMarble()
        if marble is not None:
            self.quarterInchMarblePublisher.publish(marble)



def main(args=None):
    rclpy.init(args=args)
    node = MarbleDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()