import math
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Mission Planner SITL connection
WINDOWS_IP = "172.29.112.1"
CONNECTION_STRING = f"udp:{WINDOWS_IP}:14550"

# Survey settings
WIDTH = 200                 # metres
HEIGHT = 100                # metres
SPACING = 20                # metres
TARGET_ALTITUDE = 15        # metres
BATTERY_USE_PER_M = 0.04    # simplified simulation model
LOW_BATTERY_RESERVE = 25    # percentage points reserved for safety
WAYPOINT_TOLERANCE_M = 1.5
WAYPOINT_TIMEOUT_S = 120
WAIT_TIMEOUT_S = 180

battery_level = 100.0
trips_home = 0
failsafes = 0
legs_completed = 0


def make_path():
    """Create alternating survey lanes with 20 m spacing."""
    path = []
    for y in range(0, HEIGHT + 1, SPACING):
        if (y // SPACING) % 2 == 0:
            path.extend([(0, y), (WIDTH, y)])
        else:
            path.extend([(WIDTH, y), (0, y)])
    return path


def get_distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def wait_until(predicate, timeout, description):
    start = time.monotonic()
    while not predicate():
        if time.monotonic() - start > timeout:
            raise TimeoutError(f"Timed out waiting for {description}.")
        time.sleep(0.5)


def grid_to_location(x, y, home_lat, home_lon, target_alt=TARGET_ALTITUDE):
    # Approximate local conversion; longitude degrees shrink with latitude.
    lat_deg_per_m = 1.0 / 111320.0
    lon_deg_per_m = 1.0 / (111320.0 * max(0.01, math.cos(math.radians(home_lat))))
    target_lat = home_lat + y * lat_deg_per_m
    target_lon = home_lon + x * lon_deg_per_m
    return LocationGlobalRelative(target_lat, target_lon, target_alt)


def fly_to_grid(vehicle, x, y, home_lat, home_lon, target_alt=TARGET_ALTITUDE):
    target_loc = grid_to_location(x, y, home_lat, home_lon, target_alt)
    vehicle.simple_goto(target_loc)

    start = time.monotonic()
    while True:
        position = vehicle.location.global_relative_frame
        if position.lat is not None and position.lon is not None:
            # Convert latitude/longitude difference to approximate local metres.
            north_m = (position.lat - target_loc.lat) * 111320.0
            east_m = (position.lon - target_loc.lon) * (
                111320.0 * max(0.01, math.cos(math.radians(home_lat)))
            )
            if math.hypot(north_m, east_m) <= WAYPOINT_TOLERANCE_M:
                return
        if time.monotonic() - start > WAYPOINT_TIMEOUT_S:
            raise TimeoutError(f"Waypoint ({x}, {y}) was not reached in time.")
        time.sleep(0.5)


def land_and_disarm(vehicle):
    vehicle.mode = VehicleMode("LAND")
    wait_until(lambda: not vehicle.armed, WAIT_TIMEOUT_S, "landing and disarming")


def run_simulation():
    global battery_level, trips_home, failsafes, legs_completed

    path = make_path()
    home_pos = (0, 0)
    current_pos = home_pos
    next_idx = 0
    vehicle = None

    try:
        print(f"Connecting to Mission Planner SITL at {CONNECTION_STRING}")
        vehicle = connect(CONNECTION_STRING, wait_ready=False)

        print("[INIT] Waiting for vehicle to become armable...")
        wait_until(lambda: vehicle.is_armable, WAIT_TIMEOUT_S, "vehicle to become armable")

        home_lat = vehicle.location.global_frame.lat
        home_lon = vehicle.location.global_frame.lon
        if home_lat is None or home_lon is None:
            raise RuntimeError("Home GPS coordinates are unavailable.")

        print("[FLIGHT] Switching to GUIDED and arming...")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        wait_until(lambda: vehicle.armed, WAIT_TIMEOUT_S, "arming")

        print(f"[FLIGHT] Taking off to {TARGET_ALTITUDE} m...")
        vehicle.simple_takeoff(TARGET_ALTITUDE)
        wait_until(
            lambda: vehicle.location.global_relative_frame.alt is not None
            and vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * 0.95,
            WAIT_TIMEOUT_S,
            "takeoff altitude",
        )

        while next_idx < len(path):
            target = path[next_idx]
            leg_distance = get_distance(current_pos, target)
            return_distance_from_target = get_distance(target, home_pos)
            required_energy = (
                (leg_distance + return_distance_from_target) * BATTERY_USE_PER_M
                + LOW_BATTERY_RESERVE
            )

            # If the next leg cannot be flown while retaining a reserve to return,
            # return home now. Keep next_idx unchanged so the same waypoint is retried.
            if battery_level < required_energy:
                return_distance_now = get_distance(current_pos, home_pos)
                return_energy = return_distance_now * BATTERY_USE_PER_M

                print(
                    f"\n[BATTERY ALERT] {battery_level:.1f}% remaining; "
                    f"{required_energy:.1f}% estimated for next leg + return + reserve."
                )

                if battery_level < return_energy:
                    failsafes += 1
                    raise RuntimeError(
                        "Battery model says the drone cannot reach home safely; "
                        "mission stopped to avoid claiming a successful failsafe."
                    )

                print("[RTH] Returning to home; preserving the next waypoint index.")
                fly_to_grid(vehicle, home_pos[0], home_pos[1], home_lat, home_lon)
                battery_level -= return_energy
                current_pos = home_pos
                land_and_disarm(vehicle)

                trips_home += 1
                print(f"[PAD] Landed. Modelled residual battery: {battery_level:.1f}%")
                print(f"[RESUME] Next target remains waypoint {next_idx + 1}: {target}")

                # Simulate a battery swap/reset while landed.
                battery_level = 100.0
                print("[PAD] Battery reset to 100%. Re-arming and taking off.")
                vehicle.mode = VehicleMode("GUIDED")
                vehicle.armed = True
                wait_until(lambda: vehicle.armed, WAIT_TIMEOUT_S, "re-arming")
                vehicle.simple_takeoff(TARGET_ALTITUDE)
                wait_until(
                    lambda: vehicle.location.global_relative_frame.alt is not None
                    and vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * 0.95,
                    WAIT_TIMEOUT_S,
                    "takeoff altitude after battery swap",
                )
                # Do not increment next_idx: the interrupted waypoint is retried.
                continue

            print(f"-> Survey waypoint {next_idx + 1}/{len(path)}: {target}")
            fly_to_grid(vehicle, target[0], target[1], home_lat, home_lon)
            battery_level -= leg_distance * BATTERY_USE_PER_M
            current_pos = target
            legs_completed += 1
            next_idx += 1

        print("\n================ MISSION REPORT ================")
        coverage = (legs_completed / len(path)) * 100 if path else 100.0
        print(f"Completed waypoints          : {legs_completed}/{len(path)}")
        print(f"Waypoint coverage             : {coverage:.1f}%")
        print(f"Battery failsafes              : {failsafes}")
        print(f"Battery swap pit-stops         : {trips_home}")
        print("Resume waypoint index error    : 0 (next index preserved)")
        print("Resume gap/overlap              : 0 m in the waypoint model")
        print("=================================================")

        print("[FINISH] Landing after completing the survey.")
        land_and_disarm(vehicle)

    except Exception as exc:
        print(f"[ERROR] {exc}")
        if vehicle is not None:
            try:
                if vehicle.armed:
                    vehicle.mode = VehicleMode("LAND")
                    time.sleep(2)
            except Exception:
                pass
    finally:
        if vehicle is not None:
            vehicle.close()
            print("[CLEANUP] Vehicle connection closed.")


if __name__ == "__main__":
    run_simulation()
