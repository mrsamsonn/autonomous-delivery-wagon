import time
import curses
import os
import time

print("Waiting for PWM setup...")
time.sleep(3)
print("Now safe to control motor.")


# Paths
PWM_BASE = "/sys/devices/7000a000.pwm/pwm/pwmchip0"
PWM0_PATH = os.path.join(PWM_BASE, "pwm0")  # Pin 32
PWM2_PATH = os.path.join(PWM_BASE, "pwm2")  # Pin 33

# Constants
PERIOD_NS = 1000000  # 1ms = 1kHz frequency
MOTOR_SPEED = 0.15  # 15% duty cycle
DUTY_NS = int(PERIOD_NS * MOTOR_SPEED)

# Export the channels (if not already exported)
def export_pwm(channel):
    try:
        with open(os.path.join(PWM_BASE, "export"), "w") as f:
            f.write(str(channel))
        time.sleep(0.1)  # Small delay to let it create the directory
    except Exception:
        pass  # Already exported

# Set PWM signal
def set_pwm(path, period, duty, enable):
    with open(os.path.join(path, "period"), "w") as f:
        f.write(str(period))
    with open(os.path.join(path, "duty_cycle"), "w") as f:
        f.write(str(duty))
    with open(os.path.join(path, "enable"), "w") as f:
        f.write(str(enable))

# Initialize PWM
export_pwm(0)
export_pwm(2)

# Functions to control motor
def stop_motor():
    set_pwm(PWM0_PATH, PERIOD_NS, 0, 1)
    set_pwm(PWM2_PATH, PERIOD_NS, 0, 1)
    print("Motor stopped.")

def move_forward():
    set_pwm(PWM0_PATH, PERIOD_NS, 0, 1)
    set_pwm(PWM2_PATH, PERIOD_NS, DUTY_NS, 1)
    print("Motor running forward...")

def move_backward():
    set_pwm(PWM0_PATH, PERIOD_NS, DUTY_NS, 1)
    set_pwm(PWM2_PATH, PERIOD_NS, 0, 1)
    print("Motor running backward...")

# Main key listener loop
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()

    direction = None

    while True:
        key = stdscr.getch()

        if key == curses.KEY_UP:
            if direction != "forward":
                move_forward()
                direction = "forward"

        elif key == curses.KEY_DOWN:
            if direction != "backward":
                move_backward()
                direction = "backward"

        elif key == 27:  # ESC
            stop_motor()
            direction = None
            break

        elif key == -1:  # No key
            if direction is not None:
                stop_motor()
                direction = None

        time.sleep(0.1)

# Run
curses.wrapper(main)

# Stop PWM at the end
stop_motor()
