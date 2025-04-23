import os
import time
import curses

# Paths and Constants
PWM_BASE = "/sys/devices/7000a000.pwm/pwm/pwmchip0"
PWM0_PATH = os.path.join(PWM_BASE, "pwm0")  # Pin 32

PERIOD_NS = 20000000  # 20ms = 50Hz

# Pulse widths in nanoseconds
LEFT_NS = 1000000
CENTER_NS = 1500000
RIGHT_NS = 2000000

# Setup PWM channel
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

# Movement functions
def move_left():
    set_pwm(PWM0_PATH, PERIOD_NS, LEFT_NS, 1)

def move_right():
    set_pwm(PWM0_PATH, PERIOD_NS, RIGHT_NS, 1)

def center_servo():
    set_pwm(PWM0_PATH, PERIOD_NS, CENTER_NS, 1)

def stop_servo():
    set_pwm(PWM0_PATH, PERIOD_NS, CENTER_NS, 0)

# Main control loop
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()
    stdscr.addstr(0, 0, "Use ← → arrows to move. SPACE to center. ESC to exit.")
    stdscr.refresh()

    direction = None
    center_servo()

    while True:
        key = stdscr.getch()

        if key == curses.KEY_LEFT:
            if direction != "left":
                move_left()
                stdscr.addstr(1, 0, "Moving left   ")
                direction = "left"

        elif key == curses.KEY_RIGHT:
            if direction != "right":
                move_right()
                stdscr.addstr(1, 0, "Moving right  ")
                direction = "right"

        elif key == ord(' '):
            center_servo()
            stdscr.addstr(1, 0, "Centering     ")
            direction = "center"

        elif key == 27:  # ESC key
            center_servo()
            time.sleep(1)  # Let it move to center before disabling
            stop_servo()
            stdscr.addstr(1, 0, "Stopped        ")
            stdscr.refresh()
            break

        stdscr.refresh()
        time.sleep(0.1)

# Run
if __name__ == "__main__":
    export_pwm(0)
    center_servo()
    print("Servo initialized. Launching control UI...")
    time.sleep(1)
    curses.wrapper(main)
    stop_servo()
    print("Exiting. Servo disabled.")
