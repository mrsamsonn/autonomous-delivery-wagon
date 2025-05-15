import time
import curses
import busio
import board
from adafruit_pca9685 import PCA9685

### --- Setup Motor PCA9685 (SCL, SDA) --- ###
i2c_motor = busio.I2C(board.SCL, board.SDA)
pca_motor = PCA9685(i2c_motor)
pca_motor.frequency = 1000  # 1kHz for motor
motor_speed = 0.15  # Adjust as needed

def stop_motor():
    pca_motor.channels[0].duty_cycle = 0
    pca_motor.channels[1].duty_cycle = 0

def move_forward():
    pca_motor.channels[0].duty_cycle = 0
    pca_motor.channels[1].duty_cycle = int(0xFFFF * motor_speed)

def move_backward():
    pca_motor.channels[0].duty_cycle = int(0xFFFF * motor_speed)
    pca_motor.channels[1].duty_cycle = 0

### --- Setup Servo PCA9685 (SCL_1, SDA_1) --- ###
i2c_servo = busio.I2C(board.SCL_1, board.SDA_1)
pca_servo = PCA9685(i2c_servo)
pca_servo.frequency = 50  # 50Hz for servo (20ms)

def pulse_width_to_duty_cycle(ms):
    return int((ms / 20.0) * 0xFFFF)

# Define pulse width limits
MIN_TURN_MS = 1.3
MAX_TURN_MS = 1.9
CENTER_MS = 1.55
TURN_STEP = 0.1

current_servo_ms = CENTER_MS

def update_servo(ms_value):
    global current_servo_ms
    current_servo_ms = max(MIN_TURN_MS, min(MAX_TURN_MS, ms_value))
    pca_servo.channels[0].duty_cycle = pulse_width_to_duty_cycle(current_servo_ms)

def move_left():
    update_servo(current_servo_ms - TURN_STEP)

def move_right():
    update_servo(current_servo_ms + TURN_STEP)

def center_servo(last_direction=None):
    global current_servo_ms
    if last_direction == "right":
        overshoot = 1.4
    elif last_direction == "left":
        overshoot = 1.7
    else:
        overshoot = CENTER_MS

    pca_servo.channels[0].duty_cycle = pulse_width_to_duty_cycle(overshoot)
    time.sleep(0.75)  # Give time for overshoot
    pca_servo.channels[0].duty_cycle = pulse_width_to_duty_cycle(CENTER_MS)
    current_servo_ms = CENTER_MS

def stop_motor():
    # Dynamic braking: both channels ON
    pca_motor.channels[0].duty_cycle = 0xFFFF
    pca_motor.channels[1].duty_cycle = 0xFFFF


### --- Main Control Loop --- ###
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()
    stdscr.addstr(0, 0, "↑↓: Motor | ←→: Steer | SPACE: Center | ESC: Exit")

    motor_direction = None
    servo_direction = None
    center_servo()

    while True:
        key = stdscr.getch()

        # --- Motor Controls ---
        if key == curses.KEY_UP:
            if motor_direction != "forward":
                move_forward()
                motor_direction = "forward"
                stdscr.addstr(2, 0, "Motor: Forward      ")

        elif key == curses.KEY_DOWN:
            if motor_direction != "backward":
                move_backward()
                motor_direction = "backward"
                stdscr.addstr(2, 0, "Motor: Backward     ")

        elif key == -1:
            if motor_direction is not None:
                stop_motor()
                motor_direction = None
                stdscr.addstr(2, 0, "Motor: Stopped      ")

        # --- Servo Controls ---
        elif key == curses.KEY_LEFT:
            move_left()
            servo_direction = "left"
            stdscr.addstr(3, 0, f"Steering: Left ({current_servo_ms:.1f}ms)   ")

        elif key == curses.KEY_RIGHT:
            move_right()
            servo_direction = "right"
            stdscr.addstr(3, 0, f"Steering: Right ({current_servo_ms:.1f}ms)  ")

        elif key == ord(' '):  # Space to center
            center_servo(servo_direction)
            servo_direction = "center"
            stdscr.addstr(3, 0, "Steering: Centered     ")

        # --- Exit ---
        elif key == 27:  # ESC
            stop_motor()
            center_servo()
            stop_servo()
            stdscr.addstr(5, 0, "Exiting...           ")
            stdscr.refresh()
            break

        stdscr.refresh()
        time.sleep(0.1)

### --- Launch --- ###
if __name__ == "__main__":
    print("Initializing motor and servo...")
    center_servo()  # Use default overshoot logic
    time.sleep(1)
    curses.wrapper(main)
    stop_motor()
    stop_servo()
    print("Exited. Motor and servo disabled.")
