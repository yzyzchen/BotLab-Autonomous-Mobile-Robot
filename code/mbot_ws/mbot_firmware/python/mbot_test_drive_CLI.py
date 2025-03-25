import time
import numpy as np
import lcm
import sys
import select
import tty
import termios
sys.path.append('/usr/lib/python3.8/site-packages/')
from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t
# from mbot_lcm_msgs.mbot_motor_pwm_t import mbot_motor_pwm_t

# normalized between 0. and 1. for motor velocity commands
LIN_VEL_CMD = 15
ANG_VEL_CMD = 7 # rad/sec

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

running = True
fwd_vel = 0.0
turn_vel = 0.0

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)
try:
    tty.setcbreak(sys.stdin.fileno())

    while running:
        if is_data():
            c = sys.stdin.read(1)
            if c == '\x1b':  # x1b is the Escape key
                break
            elif c == 'w': #forward
                fwd_vel = LIN_VEL_CMD
            elif c == 's': #backward
                fwd_vel = -LIN_VEL_CMD
            else:
                fwd_vel = 0.0

            if c == 'a':
                turn_vel = ANG_VEL_CMD
            elif c == 'd':
                turn_vel = -ANG_VEL_CMD
            else:
                turn_vel = 0.0

        # # this gives directly motor velocity commands
        command = mbot_motor_vel_t()
        command.velocity[0] = -fwd_vel + turn_vel
        command.velocity[1] = fwd_vel + turn_vel
        lc.publish("MBOT_MOTOR_VEL_CMD", command.encode())

        # this gives directly motor PWM commands
        # command = mbot_motor_pwm_t()
        # command.pwm[0] = -fwd_vel + turn_vel
        # command.pwm[1] = fwd_vel + turn_vel
        # lc.publish("MBOT_MOTOR_PWM_CMD", command.encode())
        
        time.sleep(0.1)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
