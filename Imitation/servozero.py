import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685

i2c = I2C(7)
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

now = time.time()

def move(servo, pulse):
    	pca.channels[servo].duty_cycle = pulse

def angle(pwm):
	return(int(1638 + pwm * 6553 / 270))

move(15, angle(20))
move(14, angle(20))
move(13, angle(20))
move(12, angle(20))