#!/usr/bin/env python3
"""ROS 2 version of Check_1: read-only status monitor (sends no commands)."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mavros_link import MavrosLink  # noqa: E402

GPS_TEXT = {-1: "NO FIX", 0: "FIX", 1: "SBAS FIX", 2: "RTK/GBAS FIX"}


def main():
    link = MavrosLink("check_drone_status")
    try:
        while True:
            now = time.strftime("%H:%M:%S")
            if link.state is None:
                print(f"{now} waiting for /mavros/state ... (is mavros running?)", flush=True)
            else:
                batt = "?" if link.battery is None else f"{link.battery:.0f}%"
                volt = f" {link.battery_voltage:.1f}V" if link.battery_voltage else ""
                alt = f"{link.rel_alt:5.1f}m" if link.rel_alt is not None else "   ?"
                link_ok = "" if link.connected else "NO LINK "
                print(f"{now} {link_ok}{link.mode:9s} {'ARMED' if link.armed else 'disarmed':8s} "
                      f"alt {alt}  batt {batt}{volt}  GPS {GPS_TEXT.get(link.gps_status, link.gps_status)}",
                      flush=True)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        link.shutdown()


if __name__ == "__main__":
    main()
