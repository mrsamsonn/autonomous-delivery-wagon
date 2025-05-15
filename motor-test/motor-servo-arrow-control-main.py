import os
import time
import curses
import busio
import math
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

### --- PCA9685 Setup for Motor --- ###
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # 1kHz PWM
motor_speed = 0.15  # 0.0 to 1.0

def stop_motor():
    pca.channels[0].duty_cycle = 0
    pca.channels[1].duty_cycle = 0

def move_forward():
    pca.channels[0].duty_cycle = 0
    pca.channels[1].duty_cycle = int(0xFFFF * motor_speed)

def move_backward():
    pca.channels[0].duty_cycle = int(0xFFFF * motor_speed)
    pca.channels[1].duty_cycle = 0

### --- Jetson PWM Setup for Servo --- ###
PWM_BASE = "/sys/devices/7000a000.pwm/pwm/pwmchip0"
PWM0_PATH = os.path.join(PWM_BASE, "pwm0")

PERIOD_NS = 20000000
# Adjustable center pulse width (in nanoseconds)
CENTER_NS = 1600000  # Default center (you can tweak this)

# Range to stay away from hard limits (in microseconds)
RANGE_US = 600  # ±300 μs → 1200–2000 μs

#range: 500–2500μs

# Calculate left/right relative to center
LEFT_NS  = CENTER_NS - (RANGE_US * 1000) // 2
RIGHT_NS = CENTER_NS + (RANGE_US * 1000) // 2

print(LEFT_NS)
print(RIGHT_NS)

def export_pwm(channel):
    try:
        with open(os.path.join(PWM_BASE, "export"), "w") as f:
            f.write(str(channel))
        time.sleep(0.1)
    except Exception:
        pass  # Already exported

def set_pwm(path, period, duty, enable):
    with open(os.path.join(path, "period"), "w") as f:
        f.write(str(period))
    with open(os.path.join(path, "duty_cycle"), "w") as f:
        f.write(str(duty))
    with open(os.path.join(path, "enable"), "w") as f:
        f.write(str(enable))

def smooth_move(target_ns, max_steps=200, delay=0.01):
    """Smooth and continuous servo movement."""
    current_ns = get_current_pwm_value()
    current_ns = max(1000000, min(current_ns, 2000000))  # stay in safe range

    distance = abs(target_ns - current_ns)
    steps = min(max_steps, int(distance / 5))  # more steps for bigger moves

    for i in range(1, steps + 1):
        t = i / steps
        # Cosine easing
        eased_t = 0.5 * (1 - math.cos(math.pi * t))
        interp_ns = current_ns + (target_ns - current_ns) * eased_t
        set_pwm(PWM0_PATH, PERIOD_NS, int(interp_ns), 1)
        time.sleep(delay)

    set_pwm(PWM0_PATH, PERIOD_NS, target_ns, 1)
    set_current_pwm_value(target_ns)

# You need to maintain current PWM value globally
current_pwm_value = CENTER_NS

def get_current_pwm_value():
    global current_pwm_value
    return current_pwm_value

def set_current_pwm_value(value):
    global current_pwm_value
    current_pwm_value = value

def move_left():
    smooth_move(LEFT_NS)

def move_right():
    smooth_move(RIGHT_NS)

def center_servo():
    smooth_move(CENTER_NS)

def stop_servo():
    smooth_move(CENTER_NS)
    time.sleep(0.1)
    set_pwm(PWM0_PATH, PERIOD_NS, CENTER_NS, 0)

### --- Combined Main Control --- ###
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()
    stdscr.addstr(0, 0, "↑↓: Motor | ←→: Steer | SPACE: Center | ESC: Exit")
    stdscr.refresh()

    motor_direction = None
    servo_direction = None

    center_servo()

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            if motor_direction != "forward":
                move_forward()
                motor_direction = "forward"
                stdscr.addstr(1, 0, "Motor: Forward     ")

        elif key == curses.KEY_DOWN:
            if motor_direction != "backward":
                move_backward()
                motor_direction = "backward"
                stdscr.addstr(1, 0, "Motor: Backward    ")

        elif key == curses.KEY_LEFT:
            if servo_direction != "left":
                move_left()
                servo_direction = "left"
                stdscr.addstr(2, 0, "Steering: Left     ")

        elif key == curses.KEY_RIGHT:
            if servo_direction != "right":
                move_right()
                servo_direction = "right"
                stdscr.addstr(2, 0, "Steering: Right    ")

        elif key == ord(' '):  # Spacebar to center
            center_servo()
            servo_direction = "center"
            stdscr.addstr(2, 0, "Steering: Centered ")

        elif key == 27:  # ESC to stop all
            stop_motor()
            center_servo()
            time.sleep(1)
            stop_servo()
            stdscr.addstr(3, 0, "Stopped. Exiting... ")
            stdscr.refresh()
            break

        elif key == -1:
            if motor_direction is not None:
                stop_motor()
                motor_direction = None
                stdscr.addstr(1, 0, "Motor: Stopped     ")

        stdscr.refresh()
        time.sleep(0.1)

### --- Init and Launch --- ###
if __name__ == "__main__":
    export_pwm(0)
    center_servo()
    print("Motors and servo ready. Launching UI...")
    time.sleep(1)
    curses.wrapper(main)
    stop_servo()
    print("Exited. Servo disabled.")
