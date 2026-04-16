import collections
import collections.abc
import csv
import logging
import math
import time
from datetime import datetime
from pathlib import Path

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping
if not hasattr(collections, "Mapping"):
    collections.Mapping = collections.abc.Mapping
if not hasattr(collections, "Sequence"):
    collections.Sequence = collections.abc.Sequence

from dronekit import connect, VehicleMode, LocationGlobalRelative


CONNECTION_STRING = "tcp:127.0.0.1:5762"

START_LAT = 50.450739
START_LON = 30.461242

TARGET_LAT = 50.443326
TARGET_LON = 30.448078

TARGET_ALT_M = 300.0

START_TOLERANCE_M = 300.0
TAKEOFF_TIMEOUT_S = 900
CRUISE_TIMEOUT_S = 1800
LANDING_TIMEOUT_S = 600
NO_PROGRESS_TIMEOUT_S = 180
ABORT_DISTANCE_INCREASE_M = 100.0
MAX_MISSION_DISTANCE_M = 2500.0

CRUISE_RADIUS_M = 20.0
LANDING_COMPLETE_ALT_M = 0.20

CONTROL_DT = 1.0
FIXED_YAW_DEG = None

SESSION_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_DIR = Path("logs")
LOG_DIR.mkdir(parents=True, exist_ok=True)
TXT_LOG_PATH = LOG_DIR / f"flight_{SESSION_ID}.log"
CSV_LOG_PATH = LOG_DIR / f"flight_{SESSION_ID}.csv"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(message)s",
    handlers=[
        logging.FileHandler(TXT_LOG_PATH, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)

CSV_FILE = open(CSV_LOG_PATH, "w", newline="", encoding="utf-8")
CSV_WRITER = csv.writer(CSV_FILE)
CSV_WRITER.writerow(
    [
        "timestamp",
        "phase",
        "lat",
        "lon",
        "alt",
        "yaw_deg",
        "distance_m",
        "extra",
    ]
)
CSV_FILE.flush()


def log(msg: str) -> None:
    logging.info(msg)


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def meters_per_deg_lon(lat_deg):
    return 111320.0 * math.cos(math.radians(lat_deg))


def distance_m(lat1, lon1, lat2, lon2):
    d_north = (lat2 - lat1) * 111320.0
    d_east = (lon2 - lon1) * meters_per_deg_lon(lat1)
    return math.hypot(d_north, d_east)


def get_position(vehicle):
    loc = vehicle.location.global_relative_frame
    return loc.lat, loc.lon, loc.alt


def get_alt(vehicle):
    return vehicle.location.global_relative_frame.alt


def get_yaw_deg(vehicle):
    yaw_rad = vehicle.attitude.yaw
    yaw_deg = math.degrees(yaw_rad)
    if yaw_deg < 0:
        yaw_deg += 360
    return yaw_deg


def write_csv(phase, vehicle, distance=None, extra=""):
    lat, lon, alt = get_position(vehicle)
    yaw = get_yaw_deg(vehicle)
    CSV_WRITER.writerow(
        [
            datetime.now().isoformat(timespec="seconds"),
            phase,
            f"{lat:.6f}",
            f"{lon:.6f}",
            f"{alt:.2f}",
            f"{yaw:.2f}",
            "" if distance is None else f"{distance:.2f}",
            extra,
        ]
    )
    CSV_FILE.flush()


def clear_rc(vehicle):
    vehicle.channels.overrides = {}
    log("RC overrides cleared")


def abort_to_rtl(vehicle, reason):
    log(f"ABORT: {reason}")
    try:
        clear_rc(vehicle)
    except Exception:
        pass
    try:
        vehicle.mode = VehicleMode("RTL")
        time.sleep(1)
        log("Mode switched to RTL")
    except Exception as e:
        log(f"Failed to switch to RTL: {e}")


def wait_until_armable(vehicle):
    log("Checking if vehicle is armable...")
    while not vehicle.is_armable:
        log(
            f"Vehicle is not armable yet | mode={vehicle.mode.name} "
            f"gps_fix={vehicle.gps_0.fix_type} ekf_ok={vehicle.ekf_ok}"
        )
        time.sleep(1)
    log("Vehicle is armable")


def ensure_start_position(vehicle):
    lat, lon, _ = get_position(vehicle)
    dist = distance_m(lat, lon, START_LAT, START_LON)
    log(
        f"Start check | expected A={START_LAT:.6f},{START_LON:.6f} "
        f"| current={lat:.6f},{lon:.6f} | dist={dist:.1f} m"
    )
    if dist > START_TOLERANCE_M:
        raise RuntimeError(
            f"Start position is too far from point A. "
            f"Need <= {START_TOLERANCE_M:.1f} m, got {dist:.1f} m."
        )


def switch_to_guided(vehicle):
    log("Switching mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        log("Waiting for GUIDED mode...")
        time.sleep(1)
    log("Mode switched to GUIDED")


def arm_vehicle(vehicle):
    log("Arming vehicle...")
    vehicle.armed = True
    while not vehicle.armed:
        log("Waiting for arming...")
        time.sleep(1)
    log("Vehicle armed successfully")


def disarm_vehicle(vehicle):
    log("Disarming vehicle...")
    vehicle.armed = False
    for _ in range(20):
        if not vehicle.armed:
            log("Vehicle disarmed")
            return
        time.sleep(0.5)
    log("Disarm request sent, but vehicle still reports armed.")


def log_state(phase, vehicle, distance=None, extra=""):
    lat, lon, alt = get_position(vehicle)
    yaw = get_yaw_deg(vehicle)
    msg = f"{phase} | lat={lat:.6f} lon={lon:.6f} alt={alt:.2f}m yaw={yaw:.2f}deg"
    if distance is not None:
        msg += f" | dist={distance:.1f}m"
    if extra:
        msg += f" | {extra}"
    log(msg)
    write_csv(phase, vehicle, distance=distance, extra=extra)


def get_distance_to_target(vehicle, target_lat, target_lon):
    lat, lon, _ = get_position(vehicle)
    return distance_m(lat, lon, target_lat, target_lon)


def takeoff_to_altitude(vehicle, target_alt):
    log(f"Taking off to {target_alt:.1f} m")
    vehicle.groundspeed = 8
    vehicle.simple_takeoff(target_alt)

    start = time.time()
    best_alt = -9999.0
    last_progress = time.time()

    while True:
        alt = get_alt(vehicle)
        log_state("Takeoff", vehicle, extra=f"target_alt={target_alt:.1f}")

        if alt >= target_alt * 0.95:
            log(f"Reached target altitude: {alt:.2f} m")
            return

        if alt > best_alt + 0.5:
            best_alt = alt
            last_progress = time.time()

        if time.time() - start > TAKEOFF_TIMEOUT_S:
            raise RuntimeError(f"Takeoff timed out at {alt:.2f} m")

        if time.time() - last_progress > NO_PROGRESS_TIMEOUT_S:
            raise RuntimeError(f"Takeoff stalled. Best altitude: {best_alt:.2f} m")

        time.sleep(1.0)


def maintain_fixed_yaw(vehicle, fixed_yaw_deg):
    """
    Best-effort yaw lock. If the yaw drifts too much, reissue condition_yaw.
    This is conservative and won't spam commands every loop.
    """
    try:
        current = get_yaw_deg(vehicle)
        delta = abs((current - fixed_yaw_deg + 180) % 360 - 180)
        if delta > 8.0:
            msg = vehicle.message_factory.command_long_encode(
                0, 0,
                115,  # MAV_CMD_CONDITION_YAW
                0,
                float(fixed_yaw_deg),  # heading
                10,                    # speed deg/s
                0,                     # direction (0 shortest)
                0,                     # relative (0 absolute)
                0, 0, 0
            )
            vehicle.send_mavlink(msg)
            vehicle.flush()
            log(f"Reissuing yaw hold: {fixed_yaw_deg:.1f} deg")
    except Exception:
        pass


def goto_target(vehicle, target_lat, target_lon, target_alt, fixed_yaw_deg):
    log("Flying to target...")
    target = LocationGlobalRelative(target_lat, target_lon, target_alt)
    vehicle.groundspeed = 8
    vehicle.simple_goto(target, groundspeed=8)

    start = time.time()
    best_distance = float("inf")
    last_progress = time.time()
    last_log = 0.0

    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_m(current.lat, current.lon, target_lat, target_lon)
        alt = current.alt

        now = time.time()
        if now - last_log >= 1.0:
            log_state("Cruise", vehicle, distance=dist, extra=f"target_alt={target_alt:.1f}")
            last_log = now

        maintain_fixed_yaw(vehicle, fixed_yaw_deg)

        if dist < CRUISE_RADIUS_M:
            log(f"Reached cruise radius: {dist:.2f} m")
            return True

        if dist < best_distance - 1.0:
            best_distance = dist
            last_progress = time.time()

        if dist > best_distance + ABORT_DISTANCE_INCREASE_M:
            raise RuntimeError(
                f"Cruise escaped mission line. "
                f"Best distance to target: {best_distance:.1f} m, current: {dist:.1f} m"
            )

        if dist > MAX_MISSION_DISTANCE_M:
            raise RuntimeError(
                f"Cruise escaped mission area: {dist:.1f} m from target"
            )

        if time.time() - last_progress > NO_PROGRESS_TIMEOUT_S:
            raise RuntimeError(
                f"Cruise stalled. Best distance to target: {best_distance:.1f} m"
            )

        if time.time() - start > CRUISE_TIMEOUT_S:
            raise RuntimeError(
                f"Cruise timed out. Best distance to target: {best_distance:.1f} m"
            )

        time.sleep(CONTROL_DT)


def land_at_current_position(vehicle):
    log("Landing...")
    vehicle.mode = VehicleMode("LAND")

    start = time.time()
    last_log = 0.0

    while True:
        alt = get_alt(vehicle)
        now = time.time()

        if now - last_log >= 1.0:
            log_state("Land", vehicle)
            last_log = now

        if alt <= LANDING_COMPLETE_ALT_M:
            log(f"Landing complete at altitude {alt:.2f} m")
            return

        if time.time() - start > LANDING_TIMEOUT_S:
            raise RuntimeError("Landing timed out")

        time.sleep(1.0)


def main():
    vehicle = None
    try:
        log("Connecting to vehicle...")
        vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=30)

        log("Connected successfully")
        log(f"Connection string: {CONNECTION_STRING}")
        log_state("Connected", vehicle, extra="initial connection ok")

        wait_until_armable(vehicle)
        ensure_start_position(vehicle)

        switch_to_guided(vehicle)
        arm_vehicle(vehicle)

        fixed_yaw = get_yaw_deg(vehicle)
        log(f"Fixed yaw captured for the whole mission: {fixed_yaw:.2f} deg")

        takeoff_to_altitude(vehicle, TARGET_ALT_M)
        log_state("After takeoff", vehicle, extra="at mission altitude")

        success = goto_target(vehicle, TARGET_LAT, TARGET_LON, TARGET_ALT_M, fixed_yaw)

        if success:
            land_at_current_position(vehicle)
            disarm_vehicle(vehicle)
            log("Mission success")

    except KeyboardInterrupt:
        if vehicle is not None:
            abort_to_rtl(vehicle, "Interrupted by user")
        log("Interrupted by user")

    except Exception as e:
        if vehicle is not None:
            abort_to_rtl(vehicle, str(e))
        log(f"Mission aborted safely: {e}")

    finally:
        if vehicle is not None:
            try:
                clear_rc(vehicle)
                vehicle.close()
            except Exception:
                pass
        CSV_FILE.close()
        log(f"Text log: {TXT_LOG_PATH}")
        log(f"CSV log: {CSV_LOG_PATH}")
        log("Connection closed")


if __name__ == "__main__":
    main()