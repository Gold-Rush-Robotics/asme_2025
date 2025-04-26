import board
import busio
from time import sleep
from adafruit_tca9548a import TCA9548A
from adafruit_vl53l4cd import VL53L4CD # Used on .5 inch, 345
from adafruit_vl6180x import VL6180X # Used on .25 inch, 012

i2c = busio.i2c(board.SCL, board.SDA)
tca = TCA9548A(i2c)

pin = int(input("Pin number: "))
kind = int(input("1 for quarter-inch, 2 for half-inch: "))

match kind:
	case 1:
		tof = VL6180X(tca[pin])
        read = lambda: tof.range
    case 2:
        tof = VL53L4CD(tca[pin])
        read = lambda: tof.distance

while True:
    print(f"Distance: {read()}")
    sleep(.1)
