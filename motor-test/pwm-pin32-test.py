import time
import curses
import math
import busio
import board
from adafruit_pca9685 import PCA9685

### --- PCA9685 Setup --- ###
i2c = busio.I2C(board.SCL_1, board.SDA_1)  # GPIO 27 and 28 on Jetson Nano
pca = PCA9685(i2c)
pca.frequency = 50  # Standard for servo motors (50Hz = 20ms period)

# Servo pulse range in milliseconds (converted to 16-bit duty cycle)
# Assuming 1ms (left), 1.5ms (center), 2ms (right) over 20ms period
def pulse_width_to_duty_cycle(ms):
    return int((ms / 20.0) * 0xFFFF)

CENTER_DC = pulse_width_to_duty_cycle(1.5)
LEFT_DC = pulse_width_to_duty_cycle(1.2)
RIGHT_DC = pulse_width_to_duty_cycle(1.7)

def move_left():
    pca.channels[0].duty_cycle = LEFT_DC

def move_right():
    pca.channels[0].duty_cycle = RIGHT_DC

def center_servo(last_direction):
    if last_direction == "right":
        # Compensate slightly left if coming from right
        adjusted_dc = pulse_width_to_duty_cycle(1.50)  # adjust if needed
    else:
        adjusted_dc = pulse_width_to_duty_cycle(1.5)  # standard center

    pca.channels[0].duty_cycle = adjusted_dc


def stop_servo():
    pca.channels[0].duty_cycle = 0

### --- Curses-Based UI --- ###
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()
    stdscr.addstr(0, 0, "←→: Steer | SPACE: Center | ESC: Exit")
    stdscr.refresh()

    servo_direction = None
    center_servo(servo_direction)

    while True:
        key = stdscr.getch()

        if key == curses.KEY_LEFT:
            if servo_direction != "left":
                move_left()
                servo_direction = "left"
                stdscr.addstr(1, 0, "Steering: Left     ")

        elif key == curses.KEY_RIGHT:
            if servo_direction != "right":
                move_right()
                servo_direction = "right"
                stdscr.addstr(1, 0, "Steering: Right    ")

        elif key == ord(' '):  # Spacebar to center
            center_servo(servo_direction)
            servo_direction = "center"
            stdscr.addstr(1, 0, "Steering: Centered ")

        elif key == 27:  # ESC to stop
            center_servo()
            stop_servo()
            stdscr.addstr(2, 0, "Stopped. Exiting... ")
            stdscr.refresh()
            break

        stdscr.refresh()
        time.sleep(0.1)

### --- Launch --- ###
if __name__ == "__main__":
    center_servo("center")
    print("Servo ready on PCA9685. Launching UI...")
    time.sleep(1)
    curses.wrapper(main)
    stop_servo()
    print("Exited. Servo disabled.")
