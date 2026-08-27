import time
from adafruit_pca9685 import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685

i2c = I2C(7)
pca = PCA9685(i2c, address = 0x40)
pca.frequency = 50

def move(servo, pulse): #max is 8191 min is 1638, DO NOT EXCEED 7000
    pca.channels[servo].duty_cycle = pulse

move(0, 7000)
time.sleep(1)
move(1, 7000)
time.sleep(1)
move(2, 7000)
time.sleep(1)
move(3, 7000)
time.sleep(1)
move(4, 7000)
