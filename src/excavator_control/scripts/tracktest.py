#!/usr/bin/env python3
import time
import curses
import lss
import lss_const as lssc

# ---------------- CONFIG ----------------
PORT = "/dev/ttyUSB0"
BAUD = lssc.LSS_DefaultBaud
LEFT_ID = 0
RIGHT_ID = 1

BASE_RPM = 0
RPM_STEP = 1
MAX_RPM = 200

LEFT_SIGN = -1
RIGHT_SIGN = +1

POLL_MS = 8
TICK_S  = 0.008
# ----------------------------------------

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(POLL_MS)
    curses.use_default_colors()
    stdscr.keypad(True)

    # Init LSS
    lss.initBus(PORT, BAUD)
    wleft  = lss.LSS(LEFT_ID)
    wright = lss.LSS(RIGHT_ID)

    rpm = BASE_RPM
    mode = "stop"   # latched mode

    last_applied_mode = None
    last_applied_rpm  = None

    def stop_cmd():
        wleft.wheelRPM(0)
        wright.wheelRPM(0)

    def apply_cmd(m, r):
        if m == "forward":
            wleft.wheelRPM(+LEFT_SIGN  * r)
            wright.wheelRPM(+RIGHT_SIGN * r)
        elif m == "backward":
            wleft.wheelRPM(-LEFT_SIGN  * r)
            wright.wheelRPM(-RIGHT_SIGN * r)
        elif m == "left":
            wleft.wheelRPM(-LEFT_SIGN  * r)
            wright.wheelRPM(+RIGHT_SIGN * r)
        elif m == "right":
            wleft.wheelRPM(+LEFT_SIGN  * r)
            wright.wheelRPM(-RIGHT_SIGN * r)
        else:
            stop_cmd()

    stop_cmd()

    help_lines = [
        "LSS Two-Wheel Continuous Keyboard Control",
        "W/S/A/D = set motion (latched)",
        "+ / -   = speed",
        "Space   = stop",
        "Q       = quit",
    ]

    try:
        while True:
            changed = False

            # ---- Read input ----
            while True:
                c = stdscr.getch()
                if c == -1:
                    break
                ch = chr(c).lower() if 0 <= c < 256 else None

                if ch == 'q':
                    raise KeyboardInterrupt

                elif ch == 'w':
                    mode = "forward"
                    changed = True
                elif ch == 's':
                    mode = "backward"
                    changed = True
                elif ch == 'a':
                    mode = "left"
                    changed = True
                elif ch == 'd':
                    mode = "right"
                    changed = True
                elif ch == ' ':
                    mode = "stop"
                    changed = True
                elif ch == '+':
                    rpm = min(MAX_RPM, rpm + RPM_STEP)
                    changed = True
                elif ch == '-':
                    rpm = max(1, rpm - RPM_STEP)
                    changed = True

            # ---- Apply only if needed ----
            if mode != last_applied_mode or rpm != last_applied_rpm:
                apply_cmd(mode, rpm)
                last_applied_mode = mode
                last_applied_rpm  = rpm

            # ---- UI ----
            stdscr.erase()
            for i, line in enumerate(help_lines):
                stdscr.addstr(i, 0, line)
            stdscr.addstr(len(help_lines), 0,
                          f"RPM: {rpm:>3}   Mode: {mode:>8}")
            stdscr.refresh()

            time.sleep(TICK_S)

    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd()

if __name__ == "__main__":
    curses.wrapper(main)
