import time
import busio
import curses
from adafruit_pca9685 import PCA9685
from board import SCL, SDA

# Create I2C bus interface
i2c = busio.I2C(SCL, SDA)

# Create PCA9685 instance
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM
# Set motor speed
motor_speed = 0.15  # 0 to 1.0

# Function to stop the motor
def stop_motor():
    pca.channels[0].duty_cycle = 0
    pca.channels[1].duty_cycle = 0
    print("Motor stopped.")

# Function to move the motor forward
def move_forward():
    pca.channels[0].duty_cycle = 0
    pca.channels[1].duty_cycle = int(0xFFFF * motor_speed)
    print("Motor running forward...")

# Function to move the motor backward
def move_backward():
    pca.channels[0].duty_cycle = int(0xFFFF * motor_speed)  # RPWM
    pca.channels[1].duty_cycle = 0  # LPWM
    print("Motor running backward...")

# Main function to capture key presses
def main(stdscr):
    # Clear the screen
    curses.curs_set(0)  # Hide the cursor
    stdscr.nodelay(1)  # Non-blocking input
    stdscr.clear()

    motor_direction = None  # To keep track of current direction

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:  # Up arrow key
            if motor_direction != "forward":
                move_forward()
                motor_direction = "forward"

        elif key == curses.KEY_DOWN:  # Down arrow key
            if motor_direction != "backward":
                move_backward()
                motor_direction = "backward"

        elif key == 27:  # ESC key to stop
            stop_motor()
            motor_direction = None  # Reset direction
            break

        elif key == -1:  # No key pressed
            if motor_direction is not None:  # Stop the motor when key is released
                stop_motor()
                motor_direction = None

        time.sleep(0.1)  # Small delay to prevent too fast polling

# Initialize curses and run the main loop
curses.wrapper(main)
