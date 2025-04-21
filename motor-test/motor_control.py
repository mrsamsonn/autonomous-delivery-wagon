from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time
from typing import Union

# Create I2C bus interface
i2c = busio.I2C(SCL, SDA)

# Create PCA9685 instance
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM
# Set motor direction
motor_speed = 0.1  # 0 to 1.0

# forward direction
pca.channels[0].duty_cycle = 0
pca.channels[1].duty_cycle = int(0xFFFF * motor_speed)
print("Motor running forward...")


time.sleep(3)

pca.channels[0].duty_cycle = int(0xFFFF * motor_speed)  # RPWM
pca.channels[1].duty_cycle = 0  # LPWM

print("Motor running backward...")

time.sleep(3)


# Stop
pca.channels[0].duty_cycle = 0
pca.channels[1].duty_cycle = 0

print("Motor stopped.")


