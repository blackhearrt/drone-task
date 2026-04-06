import collections
import collections.abc
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


def wait_until_armable(vehicle):
    print("Checking if vehicle is armable...")

    while not vehicle.is_armable:
        print("Vehicle is not armable yet...")
        print(" Mode:", vehicle.mode.name)
        print(" GPS fix type:", vehicle.gps_0.fix_type)
        print(" EKF OK:", vehicle.ekf_ok)
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


def set_rc(vehicle, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    vehicle.channels.overrides = {
        ROLL_CH: int(roll),
        PITCH_CH: int(pitch),
        THROTTLE_CH: int(throttle),
        YAW_CH: int(yaw),
    }


def clear_rc(vehicle):
    vehicle.channels.overrides = {}
    print("RC overrides cleared")


def test_rc_override(vehicle):
    print("Testing RC override with low throttle...")

    print("Step 1: neutral sticks")
    set_rc(vehicle, roll=1500, pitch=1500, throttle=1000, yaw=1500)
    time.sleep(2)

    print("Current overrides:", vehicle.channels.overrides)

    print("Step 2: small throttle increase")
    set_rc(vehicle, roll=1500, pitch=1500, throttle=1100, yaw=1500)
    time.sleep(3)

    print("Current overrides:", vehicle.channels.overrides)

    print("Step 3: back to minimum throttle")
    set_rc(vehicle, roll=1500, pitch=1500, throttle=1000, yaw=1500)
    time.sleep(2)

    clear_rc(vehicle)


def main():
    print("Connecting to vehicle...")
    vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=30)

    try:
        print("Connected successfully")
        print("Connection string:", CONNECTION_STRING)
        print("Initial mode:", vehicle.mode.name)
        print("Initial armed:", vehicle.armed)
        print("Latitude:", vehicle.location.global_relative_frame.lat)
        print("Longitude:", vehicle.location.global_relative_frame.lon)
        print("Altitude:", vehicle.location.global_relative_frame.alt)

        wait_until_armable(vehicle)
        switch_to_stabilize(vehicle)
        arm_vehicle(vehicle)
        test_rc_override(vehicle)

        print("Final mode:", vehicle.mode.name)
        print("Final armed:", vehicle.armed)

    finally:
        clear_rc(vehicle)
        vehicle.close()
        print("Connection closed")


if __name__ == "__main__":
    main()
