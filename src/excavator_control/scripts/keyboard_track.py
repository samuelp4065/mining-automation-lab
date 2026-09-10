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

BASE_RPM = 20
RPM_STEP = 5

LEFT_SIGN = -1
RIGHT_SIGN = +1

# Low-latency + anti-flicker tuning
POLL_MS = 8               # getch timeout (ms)
TICK_S  = 0.008           # main loop tick (s)
HOLD_GRACE_S = 0.45       # treat key as held if seen within this window
STOP_DEBOUNCE_S = 0.18    # require this much silence before actually sending stop
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

    # Truth: what we last applied to motors
    last_mode_applied = "stop"  # "stop","forward","backward","left","right"
    last_rpm_applied  = 0.0
    last_apply_time   = 0.0

    # Last time we saw movement keys
    last_seen = {"w": 0.0, "s": 0.0, "a": 0.0, "d": 0.0}
    mode_to_key = {"forward": "w", "backward": "s", "left": "a", "right": "d"}

    def stop_cmd():
        wleft.wheelRPM(0)
        wright.wheelRPM(0)

    def apply_cmd(mode, r):
        nonlocal last_mode_applied, last_rpm_applied, last_apply_time
        if mode == "forward":
            wleft.wheelRPM(+LEFT_SIGN  * r)
            wright.wheelRPM(+RIGHT_SIGN * r)
        elif mode == "backward":
            wleft.wheelRPM(-LEFT_SIGN  * r)
            wright.wheelRPM(-RIGHT_SIGN * r)
        elif mode == "left":
            wleft.wheelRPM(-LEFT_SIGN  * r)
            wright.wheelRPM(+RIGHT_SIGN * r)
        elif mode == "right":
            wleft.wheelRPM(+LEFT_SIGN  * r)
            wright.wheelRPM(-RIGHT_SIGN * r)
        else:
            stop_cmd()
        last_mode_applied, last_rpm_applied = mode, r
        last_apply_time = time.time()

    # start safe
    stop_cmd()

    help_lines = [
        "LSS Two-Wheel Keyboard Control (Hold to move, anti-flicker)",
        "W/S/A/D = drive   +/- = speed   Space = stop   Q = quit",
        f"Poll={POLL_MS}ms  Grace={int(HOLD_GRACE_S*1000)}ms  StopDebounce={int(STOP_DEBOUNCE_S*1000)}ms",
    ]

    try:
        while True:
            now = time.time()

            # ---- 1) Read ALL input first ----
            speed_changed = False
            while True:
                c = stdscr.getch()
                if c == -1:
                    break
                ch = chr(c).lower() if 0 <= c < 256 else None

                if ch == 'q':
                    raise KeyboardInterrupt

                elif ch == '+':
                    rpm = min(200, rpm + RPM_STEP)
                    speed_changed = True
                    # keep movement alive while adjusting speed
                    if last_mode_applied in mode_to_key:
                        last_seen[mode_to_key[last_mode_applied]] = now

                elif ch == '-':
                    rpm = max(1, rpm - RPM_STEP)
                    speed_changed = True
                    if last_mode_applied in mode_to_key:
                        last_seen[mode_to_key[last_mode_applied]] = now

                elif ch == ' ':
                    # explicit stop request
                    for k in last_seen: last_seen[k] = 0.0

                elif ch in last_seen:
                    last_seen[ch] = now

                # ignore others

            # ---- 2) Decide requested mode from recent keys ----
            if   now - last_seen["w"] <= HOLD_GRACE_S: desired_mode = "forward"
            elif now - last_seen["s"] <= HOLD_GRACE_S: desired_mode = "backward"
            elif now - last_seen["a"] <= HOLD_GRACE_S: desired_mode = "left"
            elif now - last_seen["d"] <= HOLD_GRACE_S: desired_mode = "right"
            else:                                      desired_mode = "stop"

            # ---- 3) Decide what to APPLY (with stop debounce) ----
            # If desired is motion: apply immediately (or on speed change).
            # If desired is stop: only send stop if we’ve had enough silence.
            apply_mode = last_mode_applied
            if desired_mode != "stop":
                apply_mode = desired_mode
            else:
                # time since last movement-keystroke
                last_move_time = max(last_seen.values())
                silent_for = now - last_move_time
                if silent_for >= STOP_DEBOUNCE_S:
                    apply_mode = "stop"
                # else keep previous applied motion to avoid micro-stops

            # ---- 4) Apply only when something changes (or we tweaked speed) ----
            if speed_changed or apply_mode != last_mode_applied or rpm != last_rpm_applied:
                apply_cmd(apply_mode, rpm)

            # ---- 5) Render UI from applied state (no flicker) ----
            stdscr.erase()
            for i, line in enumerate(help_lines):
                stdscr.addstr(i, 0, line)
            stdscr.addstr(len(help_lines), 0, f"RPM: {int(last_rpm_applied):>3}   Mode: {last_mode_applied:>8}")
            stdscr.refresh()

            time.sleep(TICK_S)

    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd()

if __name__ == "__main__":
    curses.wrapper(main)
