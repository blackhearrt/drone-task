import collections
import collections.abc
import math
import time

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

if not hasattr(collections, "Mapping"):
    collections.Mapping = collections.abc.Mapping

if not hasattr(collections, "Sequence"):
    collections.Sequence = collections.abc.Sequence

from dronekit import connect, VehicleMode

CONNECTION_STRING = "tcp:127.0.0.1:5762"

ROLL_CH = "1"
PITCH_CH = "2"
THROTTLE_CH = "3"
YAW_CH = "4"

NEUTRAL = 1500
THROTTLE_MIN = 1000
FIXED_YAW = 1500


def wait_until_armable(vehicle):
    print("Checking if vehicle is armable...")

    while not vehicle.is_armable:
        print("Vehicle is not armable yet...")
        print("  Mode:", vehicle.mode.name)
        print("  GPS fix type:", vehicle.gps_0.fix_type)
        print("  EKF OK:", vehicle.ekf_ok)
        time.sleep(1)

    print("Vehicle is armable")


def switch_to_stabilize(vehicle):
    print("Switching mode to STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")

    while vehicle.mode.name != "STABILIZE":
        print("Waiting for STABILIZE mode...")
        time.sleep(1)

    print("Mode switched to STABILIZE")


def arm_vehicle(vehicle):
    print("Arming vehicle...")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed successfully")


def set_rc(vehicle, roll=1500, pitch=1500, throttle=1000, yaw=1500):
    vehicle.channels.overrides = {
        ROLL_CH: int(roll),
        PITCH_CH: int(pitch),
        THROTTLE_CH: int(throttle),
        YAW_CH: int(yaw),
    }


def clear_rc(vehicle):
    vehicle.channels.overrides = {}
    print("RC overrides cleared")


def get_alt(vehicle):
    return vehicle.location.global_relative_frame.alt


def get_yaw_deg(vehicle):
    yaw_rad = vehicle.attitude.yaw
    yaw_deg = math.degrees(yaw_rad)
    if yaw_deg < 0:
        yaw_deg += 360
    return yaw_deg


def print_status(vehicle, title="Status"):
    print(title)
    print(f"  Mode: {vehicle.mode.name}")
    print(f"  Armed: {vehicle.armed}")
    print(f"  Altitude: {get_alt(vehicle):.2f} m")
    print(f"  Yaw: {get_yaw_deg(vehicle):.2f} deg")


def choose_takeoff_throttle(current_alt, target_alt):
    error = target_alt - current_alt

    if error > 0.80:
        return 1518
    if error > 0.50:
        return 1512
    if error > 0.25:
        return 1506
    if error > 0.10:
        return 1500
    if error > -0.10:
        return 1490
    if error > -0.30:
        return 1480

    return 1470


def choose_descent_throttle(current_alt):
    if current_alt > 1.50:
        return 1450
    if current_alt > 1.00:
        return 1440
    if current_alt > 0.60:
        return 1425
    if current_alt > 0.30:
        return 1400
    return 1000


def controlled_takeoff_and_hold(vehicle, target_alt=1.2, hold_seconds=4):
    print(f"Starting controlled low-altitude takeoff to {target_alt:.2f} m")

    reached_target = False

    for step in range(1, 41):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, target_alt)

        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)

        print(f"Takeoff step {step:02d} | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(0.5)

        alt = get_alt(vehicle)

        if alt >= target_alt - 0.10:
            reached_target = True
            print(f"Reached target zone at altitude {alt:.2f} m")

            set_rc(vehicle, roll=1500, pitch=1500, throttle=1485, yaw=FIXED_YAW)
            time.sleep(1)
            break

    if not reached_target:
        print(
            f"Did not confidently reach target altitude. Current alt: {get_alt(vehicle):.2f} m"
        )

    print(f"Holding near target altitude for {hold_seconds} seconds")

    for second in range(1, hold_seconds + 1):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, target_alt)

        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)

        print(f"Hold {second:02d}s | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(1)


def gentle_pitch_roll_test(vehicle, base_alt=1.2):
    print("Starting gentle pitch/roll test in air")

    print("Phase 1: neutral stabilization")
    for i in range(3):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, base_alt)
        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)
        print(f"  Neutral {i+1}/3 | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(1)

    print("Phase 2: small pitch forward")
    for i in range(2):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, base_alt)
        set_rc(vehicle, roll=1500, pitch=1510, throttle=throttle, yaw=FIXED_YAW)
        print(f"  Pitch forward {i+1}/2 | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(1)

    print("Phase 3: neutral")
    for i in range(2):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, base_alt)
        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)
        print(
            f"  Neutral after pitch {i+1}/2 | altitude={alt:.2f} m | throttle={throttle}"
        )
        time.sleep(1)

    print("Phase 4: small roll right")
    for i in range(2):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, base_alt)
        set_rc(vehicle, roll=1510, pitch=1500, throttle=throttle, yaw=FIXED_YAW)
        print(f"  Roll right {i+1}/2 | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(1)

    print("Phase 5: neutral")
    for i in range(2):
        alt = get_alt(vehicle)
        throttle = choose_takeoff_throttle(alt, base_alt)
        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)
        print(
            f"  Neutral after roll {i+1}/2 | altitude={alt:.2f} m | throttle={throttle}"
        )
        time.sleep(1)


def controlled_descent(vehicle):
    print("Starting controlled descent...")

    for step in range(1, 41):
        alt = get_alt(vehicle)

        if alt <= 0.15:
            print(f"Near ground detected at altitude {alt:.2f} m")
            set_rc(vehicle, roll=1500, pitch=1500, throttle=1000, yaw=FIXED_YAW)
            time.sleep(2)
            return

        throttle = choose_descent_throttle(alt)

        set_rc(vehicle, roll=1500, pitch=1500, throttle=throttle, yaw=FIXED_YAW)

        print(f"Descent step {step:02d} | altitude={alt:.2f} m | throttle={throttle}")
        time.sleep(0.5)

    print(f"Descent loop finished. Final altitude: {get_alt(vehicle):.2f} m")


def main():
    print("Connecting to vehicle...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=30)

    try:
        print("Connected successfully")
        print("Connection string:", CONNECTION_STRING)
        print_status(vehicle, "Initial status")

        wait_until_armable(vehicle)
        switch_to_stabilize(vehicle)
        arm_vehicle(vehicle)

        print_status(vehicle, "Status after arming")

        controlled_takeoff_and_hold(vehicle, target_alt=1.2, hold_seconds=4)
        print_status(vehicle, "Status after takeoff/hold")

        gentle_pitch_roll_test(vehicle, base_alt=1.2)
        print_status(vehicle, "Status after pitch/roll test")

        controlled_descent(vehicle)
        print_status(vehicle, "Final status")

    finally:
        clear_rc(vehicle)
        vehicle.close()
        print("Connection closed")


if __name__ == "__main__":
    main()
