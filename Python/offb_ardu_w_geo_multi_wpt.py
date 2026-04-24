#!/usr/bin/env python3
"""
ArduCopter (ArduPilot) Geofence + Mission Breach Test + Kill Envelope
+ Collision Avoidance Hooks + JSONL logging + Replay summary tool.

Test flow:
  - Wait heartbeat & home
  - Configure fence (radius/alt, LAND on breach)
  - Configure GCS failsafe (MAVLink timeout -> LAND)
  - GUIDED takeoff to 5m
  - Upload mission: WP1 inside fence, WP2 outside fence, RTL backup
  - Switch AUTO to run mission
  - Monitor STATUSTEXT + mode + position; log events
"""

import time
import math
import json
import argparse
from pathlib import Path
from pymavlink import mavutil

# --------------------------
# User tunables
# --------------------------
FENCE_RADIUS_M = 18.0
FENCE_ALT_MAX_M = 30.0
TAKEOFF_ALT_M = 5.0

WP1_NORTH_M = 12.0   # inside fence
WP2_NORTH_M = 23.0   # outside fence (should breach if fence=18m)


# Simple avoidance thresholds (hooks; conservative)
AVOID_ENABLE = False
AVOID_STOP_M = 1.8
AVOID_BACKUP_M = 1.0
AVOID_BACKUP_VEL = 0.5     # m/s backward
AVOID_YAW_DEG = 45         # yaw step away (placeholder)
AVOID_HOLD_S = 2.0

# --------------------------
# Helpers
# --------------------------
def now_s() -> float:
    return time.time()

def log_event(fp, kind: str, **fields):
    rec = {"t": now_s(), "kind": kind, **fields}
    fp.write(json.dumps(rec) + "\n")
    fp.flush()

def safe_param_id(pid):
    # pid can be str or bytes depending on pymavlink version
    if isinstance(pid, (bytes, bytearray)):
        return pid.decode("ascii", errors="ignore").strip("\x00")
    return str(pid).strip("\x00")
    
def bearing_to_ne(dist_m: float, bearing_deg: float):
    th = math.radians(bearing_deg)
    north_m = dist_m * math.cos(th)
    east_m  = dist_m * math.sin(th)
    return north_m, east_m

def meters_to_latlon_offsets(lat_deg, north_m, east_m):
    dlat = north_m / 111_111.0
    dlon = east_m / (111_111.0 * math.cos(math.radians(lat_deg)))
    return dlat, dlon

def wait_heartbeat(master, timeout=10):
    master.wait_heartbeat(timeout=timeout)
    return master.target_system, master.target_component

def wait_message(master, types, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type=types, blocking=False)
        if msg:
            return msg
        time.sleep(0.05)
    return None

def request_message(master, msg_id: int):
    # MAV_CMD_REQUEST_MESSAGE
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0, msg_id, 0, 0, 0, 0, 0, 0
    )

def wait_home(master, timeout=10):
    request_message(master, 242)  # HOME_POSITION
    home = wait_message(master, "HOME_POSITION", timeout=timeout)
    if not home:
        raise RuntimeError("No HOME_POSITION received (need GPS + home set).")
    return home.latitude / 1e7, home.longitude / 1e7

def wait_gps_3d_fix(master, timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        gps = master.recv_match(type="GPS_RAW_INT", blocking=False)
        if gps and getattr(gps, "fix_type", 0) >= 3:
            return True
        time.sleep(0.25)
    return False

def set_mode(master, mode_name: str):
    modes = master.mode_mapping()
    if mode_name not in modes:
        raise RuntimeError(f"Mode '{mode_name}' not in mapping: {modes}")
    master.set_mode(modes[mode_name])

def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()

def param_set_and_confirm(master, name: str, value: float, ptype, timeout=2.0):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode("ascii"),
        float(value),
        ptype
    )
    # Wait for echo
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=False)
        if msg:
            pid = safe_param_id(msg.param_id)
            if pid == name:
                return True, msg.param_value
        time.sleep(0.05)
    return False, None

# --------------------------
# ArduPilot fence + kill envelope
# --------------------------
def configure_arducopter_fence(master, radius_m, alt_max_m, log_fp=None):
    INT = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    REAL = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    # FENCE_TYPE bitmap varies by ArduPilot version; "3" is commonly Circle+Alt.
    # Best practice: set it once in QGC Safety then read back, but 3 is a good default for SITL.
    ok1, rb1 = param_set_and_confirm(master, "FENCE_ENABLE", 1, INT)
    ok2, rb2 = param_set_and_confirm(master, "FENCE_RADIUS", radius_m, REAL)
    ok3, rb3 = param_set_and_confirm(master, "FENCE_ALT_MAX", alt_max_m, REAL)
    ok4, rb4 = param_set_and_confirm(master, "FENCE_TYPE", 3, INT)
    ok5, rb5 = param_set_and_confirm(master, "FENCE_ACTION", 1, INT)  # 1 commonly = LAND

    if log_fp:
        log_event(log_fp, "params_fence",
                  FENCE_ENABLE={"ok": ok1, "rb": rb1},
                  FENCE_RADIUS={"ok": ok2, "rb": rb2},
                  FENCE_ALT_MAX={"ok": ok3, "rb": rb3},
                  FENCE_TYPE={"ok": ok4, "rb": rb4},
                  FENCE_ACTION={"ok": ok5, "rb": rb5})

    print(f"FENCE_ENABLE  -> ok={ok1}, rb={rb1}")
    print(f"FENCE_RADIUS  -> set {radius_m} m, ok={ok2}, rb={rb2}")
    print(f"FENCE_ALT_MAX -> set {alt_max_m} m, ok={ok3}, rb={rb3}")
    print(f"FENCE_TYPE    -> set 3 (circle+alt), ok={ok4}, rb={rb4}")
    print(f"FENCE_ACTION  -> set 1 (LAND), ok={ok5}, rb={rb5}")

def configure_gcs_failsafe(master, timeout_s=5.0, action_land=True, log_fp=None):
    """
    Kill envelope piece: if MAVLink/GCS link stops, ArduPilot triggers failsafe.
    Params vary by version, but these are common for Copter:
      FS_GCS_ENABLE, FS_GCS_TIMEOUT, FS_GCS_ACTION
    If your build lacks them, set in QGC Safety tab instead (and keep this as a check).
    """
    INT = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    REAL = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    action = 1 if action_land else 0  # commonly 1=LAND, 0=disabled/hold (varies)

    ok1, rb1 = param_set_and_confirm(master, "FS_GCS_ENABLE", 1, INT)
    ok2, rb2 = param_set_and_confirm(master, "FS_GCS_TIMEOUT", timeout_s, REAL)
    ok3, rb3 = param_set_and_confirm(master, "FS_GCS_ACTION", action, INT)

    if log_fp:
        log_event(log_fp, "params_gcs_failsafe",
                  FS_GCS_ENABLE={"ok": ok1, "rb": rb1},
                  FS_GCS_TIMEOUT={"ok": ok2, "rb": rb2},
                  FS_GCS_ACTION={"ok": ok3, "rb": rb3})

    print(f"FS_GCS_ENABLE  -> ok={ok1}, rb={rb1}")
    print(f"FS_GCS_TIMEOUT -> set {timeout_s}s, ok={ok2}, rb={rb2}")
    print(f"FS_GCS_ACTION  -> set {action} (LAND if supported), ok={ok3}, rb={rb3}")

# --------------------------
# Mission upload (ArduPilot-friendly MISSION_ITEM)
# --------------------------
def upload_mission(master, home_lat, home_lon, bearing_deg, log_fp=None):
    alt = TAKEOFF_ALT_M
    
    # Distances you want along the runway direction
    WP1_DIST_M = 12.0
    WP2_DIST_M = 25.0
    
    # Convert those distances into North/East offsets
    n1, e1 = bearing_to_ne(WP1_DIST_M, bearing_deg)
    n2, e2 = bearing_to_ne(WP2_DIST_M, bearing_deg)

    # Convert North/East meters into lat/lon deltas
    dlat1, dlon1 = meters_to_latlon_offsets(home_lat, n1, e1)
    dlat2, dlon2 = meters_to_latlon_offsets(home_lat, n2, e2)

    # Now the waypoint lat/lon are actually used:
    lat1, lon1 = home_lat + dlat1, home_lon + dlon1
    lat2, lon2 = home_lat + dlat2, home_lon + dlon2
    hold_s = 2.0
    accept_radius_m = 8.0  # <- key: prevent “stuck at WP1”
    
    print(f"bearing={bearing_deg:.1f}°")
    print(f"WP1 NE=({n1:.1f}m, {e1:.1f}m)  lat/lon=({lat1:.7f}, {lon1:.7f})")
    print(f"WP2 NE=({n2:.1f}m, {e2:.1f}m)  lat/lon=({lat2:.7f}, {lon2:.7f})")

    items = [
        dict(cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=lat1, lon=lon1, alt=alt, hold=hold_s),
        # Explicit pause
        dict(cmd=mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, lat=lat1, lon=lon1, alt=alt, hold=5.0),
        dict(cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat=lat2, lon=lon2, alt=alt, hold=hold_s),
        dict(cmd=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, lat=0.0, lon=0.0, alt=0.0, hold=0.0),
    ]

    master.mav.mission_count_send(master.target_system, master.target_component, len(items))
    print("Mission count sent:", len(items))

    sent = set()
    while len(sent) < len(items):
        req = master.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True, timeout=10)
        if not req:
            raise RuntimeError("Timeout waiting for MISSION_REQUEST(_INT) during mission upload")

        seq = int(req.seq)
        if seq < 0 or seq >= len(items):
            raise RuntimeError(f"Vehicle requested seq={seq} outside mission length={len(items)}")

        it = items[seq]
        print("Requested seq:", seq, "type:", req.get_type())

        if req.get_type() == "MISSION_REQUEST_INT":
            master.mav.mission_item_int_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                it["cmd"],
                0,  # current
                1,  # autocontinue
                float(it.get("hold", 0.0)),       # param1 hold
                float(accept_radius_m),           # param2 acceptance radius
                0.0, 0.0,                         # param3/4 (pass/yaw)
                int(float(it["lat"]) * 1e7),
                int(float(it["lon"]) * 1e7),
                float(it["alt"])
            )
        else:
            master.mav.mission_item_send(
                master.target_system,
                master.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                it["cmd"],
                0,  # current
                1,  # autocontinue
                float(it.get("hold", 0.0)),       # param1 hold
                float(accept_radius_m),           # param2 acceptance radius
                0.0, 0.0,                         # param3/4
                float(it["lat"]),
                float(it["lon"]),
                float(it["alt"])
            )

        sent.add(seq)

    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if not ack:
        raise RuntimeError("No MISSION_ACK after mission upload")

    if log_fp:
        log_event(log_fp, "mission_uploaded",
                  wp1={"lat": lat1, "lon": lon1, "alt": alt, "north_m": WP1_NORTH_M},
                  wp2={"lat": lat2, "lon": lon2, "alt": alt, "north_m": WP2_NORTH_M})

    print("Mission uploaded (ACK received).")
    
    
# --------------------------
# Collision avoidance hooks (optional)
# --------------------------
class AvoidanceSM:
    """
    Hook-friendly state machine. You can feed it 'front_distance_m' and it
    will decide desired action. This does NOT implement full control here
    (you might publish velocity/yaw setpoints, or switch modes). It's a hook layer.
    """
    CRUISE = "CRUISE"
    HOLD = "HOLD"
    BACKUP = "BACKUP"
    YAW = "YAW"
    RESUME = "RESUME"

    def __init__(self):
        self.state = self.CRUISE
        self.t_state = now_s()
        self.backed_up = 0.0

    def update(self, front_dist_m: float | None):
        t = now_s()
        # If no sensor data, do nothing (or be conservative: HOLD)
        if front_dist_m is None:
            return self.state, {}

        if self.state == self.CRUISE:
            if front_dist_m < AVOID_STOP_M:
                self.state = self.HOLD
                self.t_state = t
                return self.state, {"cmd": "HOLD"}

        elif self.state == self.HOLD:
            if t - self.t_state >= AVOID_HOLD_S:
                self.state = self.BACKUP
                self.t_state = t
                self.backed_up = 0.0
                return self.state, {"cmd": "BACKUP", "vel": AVOID_BACKUP_VEL}

        elif self.state == self.BACKUP:
            # In a real implementation you'd integrate distance from velocity/time or use local position.
            # Here we time-based approximate: backup for AVOID_BACKUP_M at AVOID_BACKUP_VEL
            dt = t - self.t_state
            need_t = max(0.1, AVOID_BACKUP_M / max(0.05, AVOID_BACKUP_VEL))
            if dt >= need_t:
                self.state = self.YAW
                self.t_state = t
                return self.state, {"cmd": "YAW", "deg": AVOID_YAW_DEG}
            return self.state, {"cmd": "BACKUP", "vel": AVOID_BACKUP_VEL}

        elif self.state == self.YAW:
            # Placeholder: yaw for a short time then resume
            if t - self.t_state >= 1.5:
                self.state = self.RESUME
                self.t_state = t
                return self.state, {"cmd": "RESUME"}

        elif self.state == self.RESUME:
            self.state = self.CRUISE
            self.t_state = t
            return self.state, {"cmd": "CRUISE"}

        return self.state, {}

def get_front_distance_hook(master) -> float | None:
    """
    Hook: Return front obstacle distance in meters if available.
    You can implement this by:
      - reading OBSTACLE_DISTANCE
      - reading DISTANCE_SENSOR
      - reading a custom sensor message you publish to MAVLink
    """
    # Example hook: if DISTANCE_SENSOR messages exist, pick the one facing forward.
    msg = master.recv_match(type=["DISTANCE_SENSOR", "OBSTACLE_DISTANCE"], blocking=False)
    if not msg:
        return None

    if msg.get_type() == "DISTANCE_SENSOR":
        # In MAVLink, distances are often in cm
        # orientation 0 typically means forward, but depends on autopilot/sensor
        try:
            if getattr(msg, "orientation", None) == 0:
                return float(msg.current_distance) / 100.0
        except Exception:
            return None

    # OBSTACLE_DISTANCE is a sector array; you'd pick the forward sector.
    # Left as a hook.
    return None

# --------------------------
# Main run
# --------------------------
def run(conn: str, baud: int, log_path: Path, bearing: float, avoid: bool):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as fp:
        master = mavutil.mavlink_connection(conn, baud=baud, mavlink20=True)
        wait_heartbeat(master)
        print("HB OK, MAVLink2:", master.mavlink20() if callable(master.mavlink20) else master.mavlink20)
        log_event(fp, "heartbeat", conn=conn, baud=baud)

        home_lat, home_lon = wait_home(master)
        print(f"Home: {home_lat:.7f}, {home_lon:.7f}")
        log_event(fp, "home", lat=home_lat, lon=home_lon)

        # Fence + kill envelope
        print("Configuring ArduCopter geofence...")
        configure_arducopter_fence(master, FENCE_RADIUS_M, FENCE_ALT_MAX_M, log_fp=fp)

        print("Configuring GCS (MAVLink) failsafe (kill envelope)...")
        configure_gcs_failsafe(master, timeout_s=5.0, action_land=True, log_fp=fp)

        # GPS gate
        ok_fix = wait_gps_3d_fix(master, timeout=30)
        print("✅ GPS 3D fix confirmed" if ok_fix else "⚠️ GPS 3D fix NOT confirmed (continuing anyway)")
        log_event(fp, "gps_fix", ok=bool(ok_fix))

        # GUIDED takeoff (ArduPilot-friendly)
        set_mode(master, "GUIDED")
        time.sleep(1)
        arm(master)
        log_event(fp, "armed")

        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M
        )
        log_event(fp, "takeoff_cmd", alt=TAKEOFF_ALT_M)
        time.sleep(6)

        # Upload mission and start AUTO
        print("Uploading mission...")
        upload_mission(master, home_lat, home_lon, bearing, log_fp=None)
        
        #This prevents the “AUTO started but current seq not what you think” situation.
        master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
        time.sleep(0.5)

        set_mode(master, "AUTO")
        log_event(fp, "mode_set", mode="AUTO")
        print("Mission started. Watch for fence LAND at WP2.")

        # Monitoring loop
        avoid = AvoidanceSM()
        last_mode = None
        last_pos_print = 0.0

        try:
            while True:
                # STATUSTEXT is the best “human-readable” failsafe indicator
                st = master.recv_match(type="STATUSTEXT", blocking=False)
                if st:
                    txt = st.text if isinstance(st.text, str) else st.text.decode("ascii", errors="ignore")
                    log_event(fp, "statustext", text=txt)
                    low = txt.lower()
                    if "fence" in low or "failsafe" in low or "land" in low:
                        print("⚠️", txt)

                hb = master.recv_match(type="HEARTBEAT", blocking=False)
                if hb:
                    # mode info from heartbeat
                    mode = mavutil.mode_string_v10(hb)
                    if mode != last_mode:
                        last_mode = mode
                        log_event(fp, "mode_change", mode=mode)
                        print("Mode:", mode)

                # Position (optional periodic print/log)
                if time.time() - last_pos_print > 1.0:
                    pos = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
                    if pos:
                        rel_alt = pos.relative_alt / 1000.0
                        lat = pos.lat / 1e7
                        lon = pos.lon / 1e7
                        last_pos_print = time.time()
                        log_event(fp, "pos", lat=lat, lon=lon, rel_alt=rel_alt)
                        print(f"Pos: {lat:.7f}, {lon:.7f}  rel_alt={rel_alt:.1f} m")

                # Collision avoidance hook (optional)
                if AVOID_ENABLE:
                    d = get_front_distance_hook(master)
                    state, action = avoid.update(d)
                    if action:
                        log_event(fp, "avoid", state=state, front_dist_m=d, action=action)
                        # Hook point: implement control decisions here.
                        # Example actions:
                        # - switch to GUIDED and send velocity commands
                        # - loiter/hold
                        # - yaw via condition_yaw
                        # Keeping it as a hook so you can integrate with your own control layer.
                        # print("Avoid:", state, action)

                time.sleep(0.05)

        except KeyboardInterrupt:
            log_event(fp, "user_interrupt")
        finally:
            master.close()
            log_event(fp, "closed")
            print("Connection closed.")

# --------------------------
# Replay / summarize tool
# --------------------------
def replay(logfile: Path):
    if not logfile.exists():
        print("No log file:", logfile)
        return

    events = []
    with logfile.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue

    # Summarize relevant events
    fence_hits = [e for e in events if e.get("kind") == "statustext" and "fence" in e.get("text","").lower()]
    failsafe = [e for e in events if e.get("kind") == "statustext" and "failsafe" in e.get("text","").lower()]
    land_msgs = [e for e in events if e.get("kind") == "statustext" and "land" in e.get("text","").lower()]
    mode_changes = [e for e in events if e.get("kind") == "mode_change"]

    print(f"Log: {logfile}")
    print(f"Total events: {len(events)}")
    print(f"Mode changes: {len(mode_changes)}")
    if mode_changes:
        print("  First/Last mode:", mode_changes[0]["mode"], "->", mode_changes[-1]["mode"])

    print(f"Fence-related STATUSTEXT: {len(fence_hits)}")
    for e in fence_hits[-5:]:
        print(" ", time.strftime("%H:%M:%S", time.localtime(e["t"])), e["text"])

    print(f"Failsafe STATUSTEXT: {len(failsafe)}")
    for e in failsafe[-5:]:
        print(" ", time.strftime("%H:%M:%S", time.localtime(e["t"])), e["text"])

    print(f"Land-related STATUSTEXT: {len(land_msgs)}")
    for e in land_msgs[-5:]:
        print(" ", time.strftime("%H:%M:%S", time.localtime(e["t"])), e["text"])

def main():

    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="/dev/ttyUSB0",
                    help="MAVLink connection string")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", action="store_true",
                    help="Replay/summarize the JSONL log")
    ap.add_argument("--log", default="./flight_events.jsonl",
                    help="Path to JSONL event log")
    ap.add_argument("--avoid", action="store_true",
                    help="Enable collision avoidance hooks"),
    ap.add_argument("--bearing", type=float, default=0.0,
                    help="Initial bearing in degrees (0=N, 90=E)")
    args = ap.parse_args()
    
    bearing_deg = args.bearing
    print(f"Using initial bearing: {bearing_deg:.2f} deg")

    log_path = Path(args.log)
    avoid = bool(args.avoid)

    if args.replay:
        replay(log_path)
    else:
        run(args.conn, args.baud, log_path, bearing_deg, avoid)


if __name__ == "__main__":
    main()

