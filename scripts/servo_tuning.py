from adafruit_blinka.board.raspberrypi import raspi_5

import board
import busio
import asyncio
from adafruit_tcs34725 import TCS34725
from adafruit_tca9548a import TCA9548A
from adafruit_pca9685 import PCA9685
from digitalio import DigitalInOut, Direction, Pin, Pull

i2c = busio.i2c(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60
pin = int(input("Pin number: "))

while True:
	pos = int(input("Position: "))
	pca.channels[pin].duty_cycle = pos
