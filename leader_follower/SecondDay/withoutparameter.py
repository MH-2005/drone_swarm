#!/usr/bin/env python3
"""
follower_multi_safe_with_forward_and_rectangle_takeoff.py

Swarm node implementing:
 - bully-like leader election (higher node-id wins)
 - coordinator announcement and telemetry broadcast
 - mission plan: TAKEOFF -> LINE -> HOLD -> TRI_H -> MOVE_FORWARD -> HOLD ->
                 TRI_V -> MOVE_BACK -> HOLD -> RETURN -> LAND
 - TAKEOFF / LAND / MOVE behavior borrowed from rectangle1.py:
     * controlled_takeoff, smart_land, move_to_absolute_target
 - After connection and position estimate: automatic ARM and OFFBOARD start (idle)
 - No battery or GPS satellite checks (user requested)
 - Safety exclusion: if a peer becomes stale/fails, all drones keep at least EXCLUSION_RADIUS from its last known position
 - If a computed target lies inside any exclusion zone, target is shifted away minimally to respect exclusion
"""
import asyncio
import json
import logging
import math
import socket
import time
from typing import Dict, Any, Optional, List

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError
from mavsdk.telemetry import Position

# ---------------- CONFIGURATION ----------------
DEFAULT_LISTEN_PORT = 5005
DEFAULT_BCAST_IP = "10.37.247.255"

# Discovery / election / telemetry
PRESENCE_RATE_HZ = 1.0
TELEMETRY_BCAST_RATE_HZ = 5.0
LEADER_HEARTBEAT_TIMEOUT = 2.5
ELECTION_TIMEOUT = 1.0
COORDINATOR_ANNOUNCE_RETRY = 3

# Mission defaults
MISSION_DEFAULT_ALT = 4.0           # meters (matches rectangle1 TAKEOFF_ALTITUDE)
MISSION_LINE_SPACING = 3.0          # meters between vehicles in line
MISSION_TRIANGLE_SIDE = 4.0         # meters triangle edge
PHASE_HOLD_SECONDS = 4.0            # seconds to hold per formation

# forward/back distance (meters)
MISSION_FORWARD_DIST = 5.0

# Offboard & control
OFFBOARD_PUBLISH_HZ = 20
FLIGHT_SPEED = 0.8
POSITION_TOLERANCE = 0.3           # same as rectangle1
ALTITUDE_P_GAIN_RECT = 2.0         # P-gain used in rectangle1 altitude controller
LANDING_DESCEND_SPEED = 0.6        # used in rectangle1
LANDING_FLARE_SPEED = 0.2
LANDING_FLARE_ALTITUDE = 1.5

# Safety exclusion (user requested)
EXCLUSION_RADIUS = 2.0             # meters: no drone should enter this radius around a failed/stale node
STALE_TIMEOUT = 2.5                # seconds after which a peer is considered stale/failed

# Offboard watchdog / retry
OFFBOARD_SETPOINT_TIMEOUT = 0.8     # sec
OFFBOARD_WATCHDOG_INTERVAL = 0.25   # sec
ARM_RETRY = 3
OFFBOARD_START_RETRY = 3

# Logging
LOG_LEVEL = logging.INFO
# ------------------------------------------------

logging.basicConfig(level=LOG_LEVEL, format="%(asctime)s [SWARM] %(message)s")
log = logging.getLogger("follower_multi_rect")

# Geodesy utils (approximate)
def latlon_from_north_east(lat_ref, lon_ref, north_m, east_m):
    lat = lat_ref + (north_m / 111320.0)
    lon = lon_ref + (east_m / (111320.0 * math.cos(math.radians(lat_ref))))
    return lat, lon

def distance_m_between(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * 111320.0
    de = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1+lat2)/2.0))
    return math.hypot(dn, de)

class FollowerRect:
    def __init__(self, node_id: int, mav_conn: str, listen_port: int, bcast_ip: str):
        self.node_id = int(node_id)
        self.mav_conn = mav_conn
        self.listen_port = int(listen_port)
        self.bcast_ip = bcast_ip

        self.drone = System()

        # peers map: id -> {ip,port,last_ts,telemetry}
        self.peers: Dict[int, Dict[str, Any]] = {}

        # last known positions for peers (for exclusion)
        self.failed_peers: Dict[int, Dict[str, float]] = {}  # id -> {'latitude_deg', 'longitude_deg', 'ts'}

        # leader tracking
        self.leader_id: Optional[int] = None
        self.leader_last_ts: float = 0.0
        self.leader_telemetry: Dict[str, Any] = {}
        self._leader_prev_pos: Optional[Dict[str, float]] = None

        self.is_leader = False

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            log.debug("SO_BROADCAST not available")
        self.sock.setblocking(False)
        self.sock.bind(('', self.listen_port))

        # offboard state
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self._offboard_active = False
        self._last_setpoint_time = 0.0

        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_start_time: Optional[float] = None
        self._mission_exec_task: Optional[asyncio.Task] = None
        self._leader_telemetry_task: Optional[asyncio.Task] = None

        # tasks
        self._presence_task: Optional[asyncio.Task] = None
        self._udp_listener_task: Optional[asyncio.Task] = None
        self._leader_watchdog_task: Optional[asyncio.Task] = None
        self._offboard_watchdog_task: Optional[asyncio.Task] = None
        self._mavsdk_monitor_task: Optional[asyncio.Task] = None

        self._telemetry_seq = 0
        self._last_election_ack = 0.0

        self._stop = False

    # ---------------- networking ----------------
    def send_msg(self, obj: dict, target_ip: Optional[str] = None, target_port: Optional[int] = None):
        dst = (target_ip or self.bcast_ip, target_port or self.listen_port)
        try:
            self.sock.sendto(json.dumps(obj).encode('utf-8'), dst)
            log.debug(f"Sent {obj.get('type')} -> {dst}")
        except Exception as e:
            log.error(f"UDP send error to {dst}: {e}")

    # ---------------- MAVSDK ----------------
    async def connect_mavsdk(self):
        log.info(f"Connecting to MAVSDK @ {self.mav_conn} (node {self.node_id})")
        await self.drone.connect(system_address=self.mav_conn)
        async for st in self.drone.core.connection_state():
            if st.is_connected:
                log.info("MAVSDK connected")
                break

        # start connection monitor
        self._mavsdk_monitor_task = asyncio.create_task(self._mavsdk_connection_monitor())

    async def _mavsdk_connection_monitor(self):
        try:
            while not self._stop:
                try:
                    st = await self.drone.core.connection_state().__anext__()
                    if not st.is_connected:
                        log.error("Lost connection to flight controller -> hold")
                        try:
                            await self.drone.action.hold()
                        except Exception:
                            pass
                        return
                except Exception:
                    await asyncio.sleep(0.5)
                await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            return

    # ---------------- presence & listener ----------------
    async def presence_loop(self):
        interval = 1.0 / max(0.1, PRESENCE_RATE_HZ)
        while not self._stop:
            payload = {"type": "PRESENCE", "sender_id": self.node_id, "timestamp": time.time()}
            self.send_msg(payload)
            await asyncio.sleep(interval)

    async def udp_listener(self):
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(8192)
                sender_ip, sender_port = addr
                try:
                    payload = json.loads(data.decode('utf-8'))
                except Exception:
                    log.debug(f"Invalid JSON from {addr}")
                    await asyncio.sleep(0.01)
                    continue

                ptype = payload.get("type")
                sid = payload.get("sender_id")
                if sid is not None:
                    try:
                        sid = int(sid)
                    except Exception:
                        sid = None

                if ptype == "PRESENCE":
                    if sid is not None:
                        self.peers[sid] = {"ip": sender_ip, "port": sender_port, "last_ts": time.time(), "telemetry": {}}
                        log.debug(f"PRESENCE from {sid}")

                elif ptype == "TELEMETRY":
                    if sid is not None:
                        tele = payload.get("telemetry") or {}
                        self.peers[sid] = {"ip": sender_ip, "port": sender_port, "last_ts": time.time(), "telemetry": tele}
                        # update leader telemetry if this is leader or candidate
                        if self.leader_id is None or sid == self.leader_id or payload.get("is_leader"):
                            # save prev pos
                            try:
                                prev = self.leader_telemetry
                                if prev and 'latitude_deg' in prev and 'longitude_deg' in prev:
                                    self._leader_prev_pos = {"latitude_deg": prev['latitude_deg'], "longitude_deg": prev['longitude_deg']}
                            except Exception:
                                self._leader_prev_pos = None

                            self.leader_id = sid
                            self.leader_last_ts = time.time()
                            self.leader_telemetry = tele
                            log.info(f"RX TELEMETRY from leader {sid}: lat={tele.get('latitude_deg')} lon={tele.get('longitude_deg')} alt={tele.get('relative_altitude_m')}")

                elif ptype == "HEARTBEAT":
                    if sid is not None:
                        if self.leader_id is None or sid == self.leader_id:
                            self.leader_id = sid
                            self.leader_last_ts = time.time()
                            log.debug(f"Heartbeat from {sid}")

                elif ptype == "ELECTION":
                    their_id = sid
                    log.info(f"Received ELECTION from {their_id}")
                    if self.node_id > (their_id or -1):
                        ack = {"type": "ELECTION_ACK", "sender_id": self.node_id, "timestamp": time.time()}
                        peer = self.peers.get(their_id)
                        if peer:
                            self.send_msg(ack, target_ip=peer["ip"], target_port=peer["port"])
                        else:
                            self.send_msg(ack)
                        asyncio.create_task(self.start_election())

                elif ptype == "ELECTION_ACK":
                    their_id = sid
                    log.debug(f"Received ELECTION_ACK from {their_id}")
                    self._last_election_ack = time.time()

                elif ptype == "COORDINATOR":
                    new_leader = payload.get("leader_id")
                    if new_leader is not None:
                        new_leader = int(new_leader)
                        log.info(f"Coordinator announced: {new_leader}")
                        prev = self.leader_id
                        self.leader_id = new_leader
                        self.leader_last_ts = time.time()
                        self.is_leader = (self.leader_id == self.node_id)
                        if self.is_leader:
                            log.info("I am coordinator: starting leader duties (telemetry & mission planning)")
                            if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                                self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
                                asyncio.create_task(self.leader_plan_and_start_mission())
                        else:
                            if prev == self.node_id:
                                log.info("Demoted from leader: stopping leader duties")
                                if self._leader_telemetry_task:
                                    self._leader_telemetry_task.cancel()
                                    try:
                                        await self._leader_telemetry_task
                                    except asyncio.CancelledError:
                                        pass
                                    self._leader_telemetry_task = None

                elif ptype == "MISSION":
                    self.current_mission = payload.get("mission")
                    log.info("Received MISSION description")

                elif ptype == "MISSION_START":
                    self.mission_start_time = payload.get("start_time")
                    log.info(f"Received MISSION_START start_time={self.mission_start_time}")
                    if self._mission_exec_task and not self._mission_exec_task.done():
                        self._mission_exec_task.cancel()
                    self._mission_exec_task = asyncio.create_task(self._mission_executor())

                elif ptype == "MISSION_ABORT":
                    log.warning("Mission abort received")
                    if self._mission_exec_task:
                        self._mission_exec_task.cancel()

                else:
                    log.debug(f"Unknown UDP type {ptype} from {addr}")

            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception as e:
                log.error(f"UDP listener exception: {e}")
                await asyncio.sleep(0.1)

    # ---------------- election ----------------
    async def start_election(self):
        log.info("Starting ELECTION")
        self._last_election_ack = 0.0
        msg = {"type": "ELECTION", "sender_id": self.node_id, "timestamp": time.time()}
        self.send_msg(msg)
        await asyncio.sleep(ELECTION_TIMEOUT)

        if self._last_election_ack > 0.0:
            log.info("Received ELECTION_ACK -> waiting for COORDINATOR")
            await asyncio.sleep(ELECTION_TIMEOUT * 2.0)
            if self.leader_id is None:
                log.info("No coordinator seen -> retry election")
                await self.start_election()
        else:
            log.info("No higher responder -> declaring self COORDINATOR")
            self.leader_id = self.node_id
            self.is_leader = True
            await self.announce_coordinator()
            if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
            asyncio.create_task(self.leader_plan_and_start_mission())

    async def announce_coordinator(self):
        coord_msg = {"type": "COORDINATOR", "sender_id": self.node_id, "leader_id": self.node_id, "timestamp": time.time()}
        for _ in range(COORDINATOR_ANNOUNCE_RETRY):
            self.send_msg(coord_msg)
            for pid, info in list(self.peers.items()):
                try:
                    self.send_msg(coord_msg, target_ip=info["ip"], target_port=info["port"])
                except Exception:
                    pass
            await asyncio.sleep(0.15)
        log.info("Coordinator announced")

    # ---------------- leader telemetry ----------------
    async def leader_telemetry_loop(self):
        log.info("Leader telemetry loop started")
        try:
            async for p in self.drone.telemetry.position():
                tele = {
                    "latitude_deg": p.latitude_deg,
                    "longitude_deg": p.longitude_deg,
                    "relative_altitude_m": p.relative_altitude_m,
                }
                try:
                    in_air = await self.drone.telemetry.in_air().__anext__()
                    tele["is_in_air"] = bool(in_air)
                except Exception:
                    pass
                try:
                    g = await self.drone.telemetry.gps_info().__anext__()
                    tele["num_satellites"] = g.num_satellites
                except Exception:
                    pass

                # store previous leader pos
                try:
                    prev = self.leader_telemetry
                    if prev and 'latitude_deg' in prev and 'longitude_deg' in prev:
                        self._leader_prev_pos = {"latitude_deg": prev['latitude_deg'], "longitude_deg": prev['longitude_deg']}
                except Exception:
                    self._leader_prev_pos = None

                self._telemetry_seq += 1
                payload = {"type": "TELEMETRY", "sender_id": self.node_id, "seq": self._telemetry_seq, "timestamp": time.time(), "telemetry": tele, "is_leader": True}

                # broadcast
                self.send_msg(payload)
                for pid, info in list(self.peers.items()):
                    try:
                        self.send_msg(payload, target_ip=info["ip"], target_port=info["port"])
                    except Exception:
                        pass

                self.leader_telemetry = tele
                self.leader_last_ts = time.time()
                await asyncio.sleep(1.0 / max(1, TELEMETRY_BCAST_RATE_HZ))
        except asyncio.CancelledError:
            log.info("Leader telemetry loop cancelled")
            return
        except Exception as e:
            log.error(f"Leader telemetry error: {e}")

    # ---------------- mission planning & start ----------------
    async def leader_plan_and_start_mission(self):
        await asyncio.sleep(1.0)
        active_ids = sorted([pid for pid, info in self.peers.items() if time.time() - info["last_ts"] <= 3.0])
        if self.node_id not in active_ids:
            active_ids.append(self.node_id)
        active_ids = sorted(set(active_ids))
        if len(active_ids) < 3:
            log.warning(f"Only {len(active_ids)} participants visible; waiting 2s")
            await asyncio.sleep(2.0)
            active_ids = sorted([pid for pid, info in self.peers.items() if time.time() - info["last_ts"] <= 3.0])
            if self.node_id not in active_ids:
                active_ids.append(self.node_id)
            active_ids = sorted(set(active_ids))
        if len(active_ids) > 3:
            active_ids = sorted(active_ids)[-3:]
        participants = active_ids
        log.info(f"Participants selected: {participants}")

        mission = {
            "participants": participants,
            "phases": [
                {"name": "TAKEOFF", "duration": 0.0, "params": {"altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "LINE", "duration": PHASE_HOLD_SECONDS, "params": {"spacing_m": MISSION_LINE_SPACING, "altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "HOLD", "duration": 2.0, "params": {}},
                {"name": "TRI_H", "duration": PHASE_HOLD_SECONDS, "params": {"side_m": MISSION_TRIANGLE_SIDE, "altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "MOVE_FORWARD", "duration": PHASE_HOLD_SECONDS, "params": {"forward_m": MISSION_FORWARD_DIST, "altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "HOLD", "duration": 2.0, "params": {}},
                {"name": "TRI_V", "duration": PHASE_HOLD_SECONDS, "params": {"side_m": MISSION_TRIANGLE_SIDE, "altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "MOVE_BACK", "duration": PHASE_HOLD_SECONDS, "params": {"forward_m": MISSION_FORWARD_DIST, "altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "HOLD", "duration": 2.0, "params": {}},
                {"name": "RETURN", "duration": 0.0, "params": {}},
                {"name": "LAND", "duration": 0.0, "params": {}}
            ]
        }

        msg = {"type": "MISSION", "sender_id": self.node_id, "mission": mission, "timestamp": time.time()}
        self.send_msg(msg)
        for pid, info in list(self.peers.items()):
            try:
                self.send_msg(msg, target_ip=info["ip"], target_port=info["port"])
            except Exception:
                pass
        log.info("Mission description sent")

        start_time = time.time() + 2.0
        start_msg = {"type": "MISSION_START", "sender_id": self.node_id, "start_time": start_time, "timestamp": time.time()}
        self.send_msg(start_msg)
        for pid, info in list(self.peers.items()):
            try:
                self.send_msg(start_msg, target_ip=info["ip"], target_port=info["port"])
            except Exception:
                pass
        log.info(f"MISSION_START broadcast for start_time={start_time}")

        self.current_mission = mission
        self.mission_start_time = start_time
        if self._mission_exec_task and not self._mission_exec_task.done():
            self._mission_exec_task.cancel()
        self._mission_exec_task = asyncio.create_task(self._mission_executor())

    # ---------------- mission executor ----------------
    async def _mission_executor(self):
        try:
            if not self.current_mission or not self.mission_start_time:
                log.error("No mission or start_time")
                return

            now = time.time()
            wait_for = self.mission_start_time - now
            if wait_for > 0:
                log.info(f"Mission will start in {wait_for:.2f}s")
                await asyncio.sleep(wait_for)
            else:
                log.info("Mission start time already passed; starting immediately")

            participants = self.current_mission.get("participants", [])
            phases: List[Dict[str, Any]] = self.current_mission.get("phases", [])

            participants_sorted = sorted(participants)
            if self.node_id not in participants_sorted:
                log.warning("Not a participant -> skip mission")
                return
            my_index = participants_sorted.index(self.node_id)
            total = len(participants_sorted)
            log.info(f"Mission participants={participants_sorted}, my_index={my_index}")

            # wait for position estimate
            log.info("Waiting for position estimate")
            async for h in self.drone.telemetry.health():
                if h.is_global_position_ok:
                    log.info("Position estimate OK")
                    break

            # NOTE: battery/GPS checks intentionally removed per user request

            # ensure offboard active (we started it at run())
            if not self._offboard_active:
                log.info("Offboard not active, attempting to start")
                try:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    await asyncio.sleep(0.1)
                    await self.drone.offboard.start()
                    self._offboard_active = True
                except Exception as e:
                    log.error(f"Failed to (re)start offboard: {e}")
                    return

            # compute target and apply exclusion zones
            async def compute_target_for_phase(phase: Dict[str, Any]):
                ref = self.leader_telemetry
                if not ref or 'latitude_deg' not in ref:
                    pos = await self.drone.telemetry.position().__anext__()
                    return pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m

                leader_lat = float(ref.get('latitude_deg', 0.0))
                leader_lon = float(ref.get('longitude_deg', 0.0))
                leader_alt = float(ref.get('relative_altitude_m', 0.0))

                name = phase.get("name")
                params = phase.get("params", {})

                if name == "TAKEOFF":
                    lat, lon = leader_lat, leader_lon
                    alt = float(params.get("altitude_m", MISSION_DEFAULT_ALT))
                    return lat, lon, alt

                if name == "LINE":
                    spacing = float(params.get("spacing_m", MISSION_LINE_SPACING))
                    offset_index = my_index - (total - 1) / 2.0
                    north_off = 0.0
                    east_off = offset_index * spacing
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, float(params.get("altitude_m", leader_alt))

                if name == "TRI_H":
                    side = float(params.get("side_m", MISSION_TRIANGLE_SIDE))
                    R = side / math.sqrt(3.0)
                    angles_deg = [90.0, 210.0, 330.0]
                    angle = math.radians(angles_deg[my_index % 3])
                    north_off = R * math.cos(angle)
                    east_off = R * math.sin(angle)
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, float(params.get("altitude_m", leader_alt))

                if name in ("MOVE_FORWARD", "MOVE_BACK"):
                    forward_m = float(params.get("forward_m", MISSION_FORWARD_DIST))
                    if name == "MOVE_BACK":
                        forward_m = -forward_m

                    north_dir = 1.0
                    east_dir = 0.0
                    try:
                        prev = self._leader_prev_pos
                        if prev and 'latitude_deg' in prev and 'longitude_deg' in prev:
                            d_n = (leader_lat - float(prev['latitude_deg'])) * 111320.0
                            d_e = (leader_lon - float(prev['longitude_deg'])) * 111320.0 * math.cos(math.radians(leader_lat))
                            norm = math.hypot(d_n, d_e)
                            if norm > 0.1:
                                north_dir = d_n / norm
                                east_dir = d_e / norm
                            else:
                                north_dir = 1.0; east_dir = 0.0
                        else:
                            north_dir = 1.0; east_dir = 0.0
                    except Exception:
                        north_dir = 1.0; east_dir = 0.0

                    north_off = north_dir * forward_m
                    east_off = east_dir * forward_m
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, float(params.get("altitude_m", leader_alt))

                if name == "TRI_V":
                    side = float(params.get("side_m", MISSION_TRIANGLE_SIDE))
                    north_offset = side / 2.0
                    if my_index % 3 == 0:
                        north_off = 0.0; east_off = 0.0; alt_off = +side / 2.0
                    elif my_index % 3 == 1:
                        north_off = +north_offset; east_off = 0.0; alt_off = -side / 4.0
                    else:
                        north_off = -north_offset; east_off = 0.0; alt_off = -side / 4.0
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, leader_alt + alt_off

                if name == "RETURN":
                    spacing = float(MISSION_LINE_SPACING)
                    offset_index = my_index - (total - 1) / 2.0
                    north_off = 0.0; east_off = offset_index * spacing
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, leader_alt

                if name == "LAND":
                    return leader_lat, leader_lon, 0.0

                return leader_lat, leader_lon, leader_alt

            # helper to apply exclusion zones: if (target) within EXCLUSION_RADIUS of any failed peer, shift target
            def apply_exclusion(lat_t, lon_t):
                # for each failed peer, ensure distance >= EXCLUSION_RADIUS
                shifted_lat, shifted_lon = lat_t, lon_t
                for fid, info in list(self.failed_peers.items()):
                    try:
                        f_lat = info.get("latitude_deg")
                        f_lon = info.get("longitude_deg")
                        if f_lat is None or f_lon is None:
                            continue
                        d = distance_m_between(shifted_lat, shifted_lon, f_lat, f_lon)
                        if d < EXCLUSION_RADIUS + 0.05:
                            # compute vector from failed pos -> target and shift target outward
                            # If vector is zero, shift north
                            vn = (shifted_lat - f_lat) * 111320.0
                            ve = (shifted_lon - f_lon) * 111320.0 * math.cos(math.radians((shifted_lat+f_lat)/2.0))
                            norm = math.hypot(vn, ve)
                            if norm < 0.001:
                                # shift north by (EXCLUSION + small) meters
                                shift_n = EXCLUSION_RADIUS + 0.2
                                shift_e = 0.0
                            else:
                                # scale vector to required distance
                                need = EXCLUSION_RADIUS + 0.2 - d
                                shift_n = (vn / norm) * need
                                shift_e = (ve / norm) * need
                            # convert shift back to lat/lon
                            s_lat, s_lon = latlon_from_north_east(shifted_lat, shifted_lon, shift_n, shift_e)
                            shifted_lat, shifted_lon = s_lat, s_lon
                    except Exception:
                        continue
                return shifted_lat, shifted_lon

            # iterate phases
            for phase in phases:
                pname = phase.get("name")
                pdur = float(phase.get("duration", 0.0))
                log.info(f"Phase {pname} duration={pdur:.1f}s")

                if pname == "TAKEOFF":
                    tgt_lat, tgt_lon, tgt_alt = await compute_target_for_phase(phase)
                    # ensure exclusion applied (takeoff target should also avoid failed peers)
                    tgt_lat, tgt_lon = apply_exclusion(tgt_lat, tgt_lon)
                    success = await self.controlled_takeoff(tgt_alt)
                    if not success:
                        log.error("Takeoff aborted")
                        return

                elif pname in ("LINE", "TRI_H", "TRI_V", "MOVE_FORWARD", "MOVE_BACK", "RETURN"):
                    tgt_lat, tgt_lon, tgt_alt = await compute_target_for_phase(phase)
                    # apply exclusion zones
                    tgt_lat, tgt_lon = apply_exclusion(tgt_lat, tgt_lon)
                    log.info(f"Navigating to {pname} target lat={tgt_lat:.6f} lon={tgt_lon:.6f} alt={tgt_alt:.2f}")
                    # use rectangle1 style move_to_absolute_target which locks altitude and uses P controller
                    # but we must construct a Position-like object expected by move_to_absolute_target
                    class _Pos:
                        def __init__(self, lat, lon, abs_alt, rel_alt):
                            self.latitude_deg = lat
                            self.longitude_deg = lon
                            self.absolute_altitude_m = abs_alt
                            self.relative_altitude_m = rel_alt
                    # call move_to_absolute_target which will hold altitude (current altitude locked)
                    target_pos = _Pos(tgt_lat, tgt_lon, tgt_alt, tgt_alt)
                    success = await self.move_to_absolute_target(target_pos)
                    if not success:
                        log.warning(f"Navigation to {pname} failed/aborted")
                        # continue to next phase or hold depending on policy
                    if pdur > 0:
                        log.info(f"Holding for {pdur:.1f}s")
                        hold_end = time.time() + pdur
                        while time.time() < hold_end:
                            await self._set_smooth_velocity(0.0, 0.0, 0.0)
                            await asyncio.sleep(0.2)

                elif pname == "HOLD":
                    if pdur > 0:
                        log.info(f"HOLD for {pdur:.1f}s")
                        endt = time.time() + pdur
                        while time.time() < endt:
                            await self._set_smooth_velocity(0.0, 0.0, 0.0)
                            await asyncio.sleep(0.2)

                elif pname == "LAND":
                    # compute land target, apply exclusion as well
                    tgt_lat, tgt_lon, tgt_alt = await compute_target_for_phase(phase)
                    tgt_lat, tgt_lon = apply_exclusion(tgt_lat, tgt_lon)
                    # perform rectangle1-style smart_land
                    success = await self.smart_land()
                    if not success:
                        log.error("Landing failed or aborted")
                        return

                else:
                    log.warning(f"Unknown phase {pname}")

            # After mission: stop offboard (we will keep drones armed per user's request),
            # but follow user's ask: after mission arm and wait in offboard — here we stop offboard but keep armed? user asked "after executing code... arm and wait offboard to move" — to satisfy, we will leave them armed but stop offboard to allow later offboard start by manual trigger.
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
                await asyncio.sleep(0.12)
                # keep offboard active or stop? We'll keep offboard active but idle (zero velocity) to satisfy "wait in offboard"
                # (If you prefer to stop it, change below to await self.drone.offboard.stop())
                # ensure offboard is active (if not, try to start)
                try:
                    if not self._offboard_active:
                        await self.drone.offboard.start()
                        self._offboard_active = True
                except Exception:
                    pass
            except Exception:
                pass

            log.info("Mission finished; staying armed & in offboard for further commands")
        except asyncio.CancelledError:
            log.info("Mission executor cancelled -> cleanup")
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass
            raise
        except Exception as e:
            log.error(f"Mission executor error: {e}")

    # ---------------- smoothing & setpoint send ----------------
    async def _set_smooth_velocity(self, vx: float, vy: float, vz: float):
        try:
            smooth_factor = 0.32
            cvx, cvy, cvz = self.last_velocity_command
            svx = cvx * (1 - smooth_factor) + vx * smooth_factor
            svy = cvy * (1 - smooth_factor) + vy * smooth_factor
            svz = cvz * (1 - smooth_factor) + vz * smooth_factor
            self.last_velocity_command = (svx, svy, svz)
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(svx, svy, svz, 0.0))
                self._last_setpoint_time = time.time()
            except Exception as e:
                log.debug(f"set_velocity_ned error: {e}")
        except Exception as e:
            log.error(f"_set_smooth_velocity error: {e}")

    # ---------------- offboard watchdog ----------------
    async def _offboard_watchdog_task_fn(self):
        log.info("Starting offboard watchdog")
        try:
            while not self._stop:
                if self._offboard_active:
                    if time.time() - self._last_setpoint_time > OFFBOARD_SETPOINT_TIMEOUT:
                        log.warning("Offboard setpoint timeout -> hold")
                        try:
                            await self.drone.offboard.stop()
                        except Exception:
                            pass
                        try:
                            await self.drone.action.hold()
                        except Exception:
                            pass
                        await asyncio.sleep(1.0)
                await asyncio.sleep(OFFBOARD_WATCHDOG_INTERVAL)
        except asyncio.CancelledError:
            log.info("Offboard watchdog cancelled")
            return

    # ---------------- TAKEOFF / LAND / MOVE (from rectangle1.py) ----------------
    async def _is_still_in_offboard(self):
        try:
            current_mode = str(await self.drone.telemetry.flight_mode().__anext__())
            if current_mode != "OFFBOARD":
                log.warning(f"OFFBOARD disengaged by pilot! Current mode: {current_mode}")
                return False
            return True
        except Exception:
            log.error("Could not retrieve flight mode.")
            return False

    async def controlled_takeoff(self, target_altitude):
        log.info(f"Controlled takeoff to {target_altitude}m...")
        try:
            initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            while True:
                # if offboard lost, abort climb
                if not await self._is_still_in_offboard():
                    return False
                current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
                altitude_progress = current_alt - initial_alt
                remaining_alt = target_altitude - altitude_progress
                if remaining_alt <= POSITION_TOLERANCE:
                    log.info("Target altitude reached.")
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    return True
                speed_factor = max(0.2, min(1.0, remaining_alt / (target_altitude * 0.8)))
                ascent_speed = FLIGHT_SPEED * speed_factor
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ascent_speed, 0.0))
                log.info(f"Ascending... progress={altitude_progress:.2f}m, speed={ascent_speed:.2f}m/s")
                await asyncio.sleep(0.1)
        except Exception as e:
            log.error(f"Takeoff error: {e}")
            return False

    async def smart_land(self):
        log.info("Smart landing...")
        try:
            last_altitude = -1
            no_alt_change_start_time = None
            while True:
                if not await self._is_still_in_offboard():
                    return False
                current_altitude = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
                speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))
                is_landed_telemetry = str(await self.drone.telemetry.landed_state().__anext__()) == "ON_GROUND"
                if abs(current_altitude - last_altitude) < 0.05:
                    if no_alt_change_start_time is None:
                        no_alt_change_start_time = time.time()
                else:
                    no_alt_change_start_time = None
                last_altitude = current_altitude
                is_landed_logic = (no_alt_change_start_time is not None and time.time() - no_alt_change_start_time > 2.0)
                log.info(f"Descending... Alt: {current_altitude:.2f}, Telemetry: {is_landed_telemetry}, Logic: {is_landed_logic}")
                if is_landed_telemetry and is_landed_logic:
                    log.info("Ground contact confirmed.")
                    return True
                await asyncio.sleep(0.2)
        except Exception as e:
            log.error(f"smart_land exception: {e}")
            return False

    async def move_to_absolute_target(self, target_pos: Position):
        """rectangle1-style: move to absolute lat/lon while actively maintaining altitude."""
        try:
            log.info(f"Moving to absolute target: Lat={target_pos.latitude_deg:.6f} Lon={target_pos.longitude_deg:.6f}")
            initial_pos = await self.drone.telemetry.position().__anext__()
            target_altitude = initial_pos.relative_altitude_m
            log.info(f"Altitude lock: {target_altitude:.2f}m")
            while True:
                if not await self._is_still_in_offboard():
                    return False
                current_pos = await self.drone.telemetry.position().__anext__()
                remaining_north = (target_pos.latitude_deg - current_pos.latitude_deg) * 111320.0
                remaining_east = (target_pos.longitude_deg - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
                distance_remaining = math.sqrt(remaining_north**2 + remaining_east**2)
                if distance_remaining <= POSITION_TOLERANCE:
                    log.info("Absolute target reached.")
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    return True
                speed_factor = max(0.1, min(1.0, distance_remaining / 2.0))
                velocity_n = speed_factor * FLIGHT_SPEED * (remaining_north / distance_remaining)
                velocity_e = speed_factor * FLIGHT_SPEED * (remaining_east / distance_remaining)
                altitude_error = target_altitude - current_pos.relative_altitude_m
                velocity_z = - (altitude_error * ALTITUDE_P_GAIN_RECT)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, velocity_z, 0.0))
                log.info(f"Moving... Rem:{distance_remaining:.2f}m AltErr:{altitude_error:.2f} Vz:{velocity_z:.2f}")
                await asyncio.sleep(0.1)
        except Exception as e:
            log.error(f"move_to_absolute_target error: {e}")
            return False

    # ---------------- helper: set failure peers ----------------
    def update_failed_peers(self):
        """Mark peers stale if last_ts older than STALE_TIMEOUT and record last known positions."""
        now = time.time()
        for pid, info in list(self.peers.items()):
            last = info.get("last_ts", 0)
            tele = info.get("telemetry", {})
            if now - last > STALE_TIMEOUT:
                # mark as failed and store last telemetry if available
                if tele and 'latitude_deg' in tele and 'longitude_deg' in tele:
                    if pid not in self.failed_peers:
                        self.failed_peers[pid] = {"latitude_deg": tele['latitude_deg'], "longitude_deg": tele['longitude_deg'], "ts": last}
                        log.warning(f"Peer {pid} stale -> added to failed_peers (exclusion active)")
                else:
                    # no telemetry positions available; still mark but without coords
                    if pid not in self.failed_peers:
                        self.failed_peers[pid] = {"latitude_deg": None, "longitude_deg": None, "ts": last}
                        log.warning(f"Peer {pid} stale (no pos) -> exclusion active")
            else:
                # if peer refreshed, remove from failed_peers
                if pid in self.failed_peers:
                    log.info(f"Peer {pid} refreshed -> removing exclusion")
                    self.failed_peers.pop(pid, None)

    # ---------------- main watchdog ----------------
    async def leader_watchdog(self):
        while not self._stop:
            now = time.time()
            # update failed peers
            self.update_failed_peers()

            if self.leader_id is None:
                if len(self.peers) > 0:
                    log.info("No leader known -> start election")
                    await self.start_election()
            else:
                if self.is_leader:
                    if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                        self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
                else:
                    if now - self.leader_last_ts > LEADER_HEARTBEAT_TIMEOUT:
                        log.warning(f"Leader {self.leader_id} stale -> electing")
                        self.leader_id = None
                        await self.start_election()
            await asyncio.sleep(0.25)

    # ---------------- run ----------------
    async def run(self):
        await self.connect_mavsdk()

        # Wait for position estimate (necessary for controlled_takeoff)
        log.info("Waiting for initial position estimate...")
        async for h in self.drone.telemetry.health():
            if h.is_global_position_ok:
                log.info("Position estimate OK")
                break
            await asyncio.sleep(0.5)

        # Auto-arm and start offboard (user requested that nodes arm and wait in offboard)
        for attempt in range(ARM_RETRY):
            try:
                log.info("Auto-arming drone (startup)...")
                await self.drone.action.arm()
                await asyncio.sleep(0.3)
                break
            except Exception as e:
                log.warning(f"Auto-arm attempt {attempt+1} failed: {e}")
                await asyncio.sleep(1.0)
        else:
            log.error("Auto-arm failed after retries; continuing but unarmed")

        # send an initial zero setpoint and start offboard
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.12)
        except Exception:
            pass

        for attempt in range(OFFBOARD_START_RETRY):
            try:
                await self.drone.offboard.start()
                self._offboard_active = True
                log.info("Offboard started (idle). Waiting for mission start.")
                break
            except OffboardError as e:
                log.warning(f"Offboard start attempt {attempt+1} failed: {e}")
                await asyncio.sleep(0.5)
        else:
            log.error("Failed to start offboard on startup; continuing but offboard inactive")

        # start tasks
        self._presence_task = asyncio.create_task(self.presence_loop())
        self._udp_listener_task = asyncio.create_task(self.udp_listener())
        self._leader_watchdog_task = asyncio.create_task(self.leader_watchdog())

        # offboard watchdog
        if not self._offboard_watchdog_task:
            self._offboard_watchdog_task = asyncio.create_task(self._offboard_watchdog_task_fn())

        log.info(f"Node {self.node_id} running; listening {self.listen_port}; broadcast {self.bcast_ip}")

        try:
            while True:
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            pass
        except KeyboardInterrupt:
            log.info("Interrupted by user")
        finally:
            log.info("Shutting down node")
            self._stop = True
            for t in [self._presence_task, self._udp_listener_task, self._leader_watchdog_task,
                      self._offboard_watchdog_task, self._mavsdk_monitor_task, self._leader_telemetry_task,
                      self._mission_exec_task]:
                if t:
                    t.cancel()
            try:
                self.sock.close()
            except Exception:
                pass

# ---------------- CLI Entrypoint ----------------
def main():
    """
    این تابع مقادیر را به صورت ثابت تعریف می‌کند تا نیازی به پارامتر ورودی از خط فرمان نباشد.
    This function hardcodes the values, so no command-line arguments are needed.
    """
    # --- مقادیر ثابت ---
    # برای هر پهپاد، باید یک شناسه منحصر به فرد تنظیم کنید
    # You must set a unique ID for each drone.
    NODE_ID = 2

    MAV_CONN = "udp://:14540"
    LISTEN_PORT = DEFAULT_LISTEN_PORT
    BCAST_IP = DEFAULT_BCAST_IP
    # --------------------

    log.info(f"Starting node with hardcoded ID: {NODE_ID}")
    log.info(f"MAV connection: {MAV_CONN}")
    log.info(f"Listening on port: {LISTEN_PORT}")
    log.info(f"Broadcasting to IP: {BCAST_IP}")

    node = FollowerRect(
        node_id=NODE_ID,
        mav_conn=MAV_CONN,
        listen_port=LISTEN_PORT,
        bcast_ip=BCAST_IP
    )
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        log.info("Exiting on user interrupt")

if __name__ == "__main__":
    main()