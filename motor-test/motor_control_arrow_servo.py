import time
import busio
from adafruit_pca9685 import PCA9685
from board import SCL, SDA

# Create I2C bus interface
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)

# Set the frequency to 50Hz for servo control
pca.frequency = 50

# Function to recenter the servo
def recenter_servo(channel):
    # Set the servo to the neutral position (1500μs pulse width)
    duty_cycle = int(0xFFFF * (1500 / 20000))  # 1500μs corresponds to neutral position
    pca.channels[channel].duty_cycle = duty_cycle
    print("Servo recentered.")

# Example usage: recenter the servo on channel 0
recenter_servo(0)

time.sleep(1)  # Keep the servo at the neutral position for a second
