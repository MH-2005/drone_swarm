#!/usr/bin/env python3
"""
follower_multi_safe_with_forward.py

Multi-node swarm node (single unified binary for leader+followers) with:
 - bully-like leader election (higher node-id wins)
 - coordinator takeover + broadcast/unicast coordinator announcement
 - leader telemetry broadcast (keeps previous position to compute heading)
 - mission planning & synchronized start with phases:
       TAKEOFF -> LINE -> HOLD -> TRI_H -> MOVE_FORWARD -> HOLD ->
       TRI_V -> MOVE_BACK -> HOLD -> RETURN -> LAND
 - robust Offboard safety:
     * initial setpoint before offboard.start()
     * offboard setpoint watchdog
     * MAVSDK connection monitor
     * retries/timeouts for arm and offboard.start()
     * GPS satellites quality check for horizontal moves
     * leader-heartbeat gating for followers (don't arm unless leader fresh)
 - hybrid landing (offboard descent -> action.land)
 - smoothing of velocity commands, timeouts, and defensive handling
 - simple UDP presence/mission distribution (suitable for LAN or secured tunnel)
 
USAGE (example):
  python3 follower_multi_safe_with_forward.py --node-id 100 --mav "udpin://:14541" --listen-port 5005 --bcast-ip 127.0.0.1

Notes:
 - Run one instance per vehicle (each instance must connect to its own PX4/SITL).
 - Node with highest node-id will become coordinator (leader).
 - This file is intended for SITL testing and as a base for real deployments; in
   production you should add message authentication (HMAC/DTLS/VPN) and ensure
   network reliability/security.
"""

import argparse
import asyncio
import json
import logging
import math
import socket
import time
from typing import Dict, Any, Optional, List

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ---------------- CONFIGURATION ----------------
DEFAULT_LISTEN_PORT = 5005
DEFAULT_BCAST_IP = "255.255.255.255"

# Discovery / election / telemetry
PRESENCE_RATE_HZ = 1.0
TELEMETRY_BCAST_RATE_HZ = 5.0
LEADER_HEARTBEAT_TIMEOUT = 2.5
ELECTION_TIMEOUT = 1.0
COORDINATOR_ANNOUNCE_RETRY = 3

# Mission defaults
MISSION_DEFAULT_ALT = 5.0           # meters
MISSION_LINE_SPACING = 3.0          # meters between vehicles in line
MISSION_TRIANGLE_SIDE = 4.0         # meters triangle edge
PHASE_HOLD_SECONDS = 4.0            # seconds to hold per formation

# NEW: forward/back distance (meters)
MISSION_FORWARD_DIST = 5.0

# Offboard & control
OFFBOARD_PUBLISH_HZ = 20
FLIGHT_SPEED = 0.8
POSITION_P_GAIN = 0.22
ALTITUDE_P_GAIN = 0.35
DEADZONE_RADIUS = 0.8               # meters to consider "reached"

# Safety thresholds
BATTERY_LAND_THRESHOLD = 0.18       # fraction
MIN_GPS_SATELLITES = 6              # require at least this many satellites for horizontal ops
OFFBOARD_SETPOINT_TIMEOUT = 0.8     # seconds without setpoint -> watchdog triggers
OFFBOARD_WATCHDOG_INTERVAL = 0.25   # seconds
ARM_RETRY = 3
OFFBOARD_START_RETRY = 3

# Logging
LOG_LEVEL = logging.INFO
# ------------------------------------------------

logging.basicConfig(level=LOG_LEVEL, format="%(asctime)s [SWARM] %(message)s")
log = logging.getLogger("follower_multi_safe_with_forward")

# Geodesy utils (approx)
def latlon_from_north_east(lat_ref, lon_ref, north_m, east_m):
    """Approximate conversion: local north/east (meters) -> lat/lon degrees."""
    lat = lat_ref + (north_m / 111320.0)
    lon = lon_ref + (east_m / (111320.0 * math.cos(math.radians(lat_ref))))
    return lat, lon


class FollowerMultiSafe:
    def __init__(self, node_id: int, mav_conn: str, listen_port: int, bcast_ip: str):
        self.node_id = int(node_id)
        self.mav_conn = mav_conn
        self.listen_port = int(listen_port)
        self.bcast_ip = bcast_ip

        # MAVSDK system (local vehicle)
        self.drone = System()

        # peers map: id -> {ip,port,last_ts,telemetry}
        self.peers: Dict[int, Dict[str, Any]] = {}

        # leader tracking
        self.leader_id: Optional[int] = None
        self.leader_last_ts: float = 0.0
        self.leader_telemetry: Dict[str, Any] = {}
        # store previous leader position to compute heading (north/east vector)
        self._leader_prev_pos: Optional[Dict[str, float]] = None

        # role flags
        self.is_leader = False

        # UDP socket for discovery/mission (non-blocking)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            log.debug("SO_BROADCAST not available on this platform")
        self.sock.setblocking(False)
        self.sock.bind(('', self.listen_port))

        # Offboard/send state
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self._offboard_active = False
        self._last_setpoint_time = 0.0

        # mission
        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_start_time: Optional[float] = None
        self._mission_exec_task: Optional[asyncio.Task] = None
        self._leader_telemetry_task: Optional[asyncio.Task] = None

        # background tasks
        self._presence_task: Optional[asyncio.Task] = None
        self._udp_listener_task: Optional[asyncio.Task] = None
        self._leader_watchdog_task: Optional[asyncio.Task] = None
        self._offboard_watchdog_task: Optional[asyncio.Task] = None
        self._mavsdk_monitor_task: Optional[asyncio.Task] = None

        self._telemetry_seq = 0
        self._last_election_ack = 0.0

        self._stop = False

    # ---------------- networking helper ----------------
    def send_msg(self, obj: dict, target_ip: Optional[str] = None, target_port: Optional[int] = None):
        dst = (target_ip or self.bcast_ip, target_port or self.listen_port)
        try:
            self.sock.sendto(json.dumps(obj).encode('utf-8'), dst)
            log.debug(f"Sent {obj.get('type')} -> {dst}")
        except Exception as e:
            log.error(f"UDP send error to {dst}: {e}")

    # ---------------- MAVSDK connection & monitor ----------------
    async def connect_mavsdk(self):
        log.info(f"Connecting MAVSDK @ {self.mav_conn} (node {self.node_id})")
        await self.drone.connect(system_address=self.mav_conn)
        async for st in self.drone.core.connection_state():
            if st.is_connected:
                log.info("MAVSDK connected to flight controller")
                break

        # start connection monitor
        self._mavsdk_monitor_task = asyncio.create_task(self._mavsdk_connection_monitor())

    async def _mavsdk_connection_monitor(self):
        """Monitor MAVSDK connection; on loss attempt safe fallback."""
        try:
            while not self._stop:
                try:
                    st = await self.drone.core.connection_state().__anext__()
                    if not st.is_connected:
                        log.error("Lost connection to flight controller -> safe fallback")
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

    # ---------------- presence / udp listener ----------------
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
                        # accept as leader telemetry if appropriate
                        if self.leader_id is None or sid == self.leader_id or payload.get("is_leader"):
                            # save previous leader pos before overwrite
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

    # ---------------- election (bully-like) ----------------
    async def start_election(self):
        log.info("Starting ELECTION (broadcast ELECTION)")
        self._last_election_ack = 0.0
        msg = {"type": "ELECTION", "sender_id": self.node_id, "timestamp": time.time()}
        self.send_msg(msg)
        await asyncio.sleep(ELECTION_TIMEOUT)

        if self._last_election_ack > 0.0:
            log.info("Got ELECTION_ACK -> waiting for COORDINATOR announcement")
            await asyncio.sleep(ELECTION_TIMEOUT * 2.0)
            if self.leader_id is None:
                log.info("No COORDINATOR seen -> retrying election")
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

                # save previous leader pos before updating (for heading vector computation)
                try:
                    prev = self.leader_telemetry
                    if prev and 'latitude_deg' in prev and 'longitude_deg' in prev:
                        self._leader_prev_pos = {"latitude_deg": prev['latitude_deg'], "longitude_deg": prev['longitude_deg']}
                except Exception:
                    self._leader_prev_pos = None

                self._telemetry_seq += 1
                payload = {"type": "TELEMETRY", "sender_id": self.node_id, "seq": self._telemetry_seq, "timestamp": time.time(), "telemetry": tele, "is_leader": True}

                # broadcast + unicast
                self.send_msg(payload)
                for pid, info in list(self.peers.items()):
                    try:
                        self.send_msg(payload, target_ip=info["ip"], target_port=info["port"])
                    except Exception:
                        pass

                # update leader telemetry reference
                self.leader_telemetry = tele
                self.leader_last_ts = time.time()

                await asyncio.sleep(1.0 / max(1, TELEMETRY_BCAST_RATE_HZ))
        except asyncio.CancelledError:
            log.info("Leader telemetry loop cancelled")
            return
        except Exception as e:
            log.error(f"Leader telemetry error: {e}")

    # ---------------- leader mission planning & start ----------------
    async def leader_plan_and_start_mission(self):
        await asyncio.sleep(1.0)  # let peers advertise
        # choose participants (prefer highest node-ids if more than 3)
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
            # deterministic: choose top 3 IDs
            active_ids = sorted(active_ids)[-3:]
        participants = active_ids
        log.info(f"Participants selected for mission: {participants}")

        # mission phases including MOVE_FORWARD and MOVE_BACK
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

        # send mission (broadcast + unicast for reliability)
        msg = {"type": "MISSION", "sender_id": self.node_id, "mission": mission, "timestamp": time.time()}
        self.send_msg(msg)
        for pid, info in list(self.peers.items()):
            try:
                self.send_msg(msg, target_ip=info["ip"], target_port=info["port"])
            except Exception:
                pass
        log.info("Mission description sent to peers")

        # schedule start (short sync delay)
        start_time = time.time() + 2.0
        start_msg = {"type": "MISSION_START", "sender_id": self.node_id, "start_time": start_time, "timestamp": time.time()}
        self.send_msg(start_msg)
        for pid, info in list(self.peers.items()):
            try:
                self.send_msg(start_msg, target_ip=info["ip"], target_port=info["port"])
            except Exception:
                pass
        log.info(f"MISSION_START broadcast for start_time={start_time}")

        # leader executes mission locally as well
        self.current_mission = mission
        self.mission_start_time = start_time
        if self._mission_exec_task and not self._mission_exec_task.done():
            self._mission_exec_task.cancel()
        self._mission_exec_task = asyncio.create_task(self._mission_executor())

    # ---------------- mission executor (leader & followers) ----------------
    async def _mission_executor(self):
        try:
            if not self.current_mission or not self.mission_start_time:
                log.error("No mission or start_time -> abort mission executor")
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
                log.warning("This node not in participants -> skipping mission")
                return
            my_index = participants_sorted.index(self.node_id)
            total = len(participants_sorted)
            log.info(f"Executing mission: participants={participants_sorted}, my_index={my_index}")

            # wait for position estimate
            log.info("Waiting for position estimate OK")
            async for h in self.drone.telemetry.health():
                if h.is_global_position_ok:
                    log.info("Position estimate OK")
                    break

            # battery check
            batt = await self._get_battery_fraction()
            if batt < BATTERY_LAND_THRESHOLD:
                log.error(f"Battery too low ({batt:.2f}) -> abort mission")
                return

            # followers must see fresh leader heartbeat before arming
            leader_ok = (self.leader_id is not None and (time.time() - self.leader_last_ts) < LEADER_HEARTBEAT_TIMEOUT)
            if not self.is_leader and not leader_ok:
                log.error("Leader heartbeat not fresh -> abort mission start (safety gate)")
                return

            # arm with retries
            for attempt in range(ARM_RETRY):
                try:
                    log.info(f"Arming attempt {attempt+1}/{ARM_RETRY}")
                    await self.drone.action.arm()
                    await asyncio.sleep(0.3)
                    break
                except Exception as e:
                    log.warning(f"Arm attempt {attempt+1} failed: {e}")
                    await asyncio.sleep(1.0)
            else:
                log.error("Failed to arm after retries -> abort mission")
                return

            # initial zero setpoint then start offboard with retries
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.12)
            except Exception:
                pass

            for attempt in range(OFFBOARD_START_RETRY):
                try:
                    await self.drone.offboard.start()
                    self._offboard_active = True
                    log.info("Offboard started")
                    break
                except OffboardError as e:
                    log.warning(f"Offboard start attempt {attempt+1} failed: {e}")
                    await asyncio.sleep(0.5)
            else:
                log.error("Failed to start offboard -> abort mission and disarm")
                try:
                    await self.drone.action.disarm()
                except Exception:
                    pass
                return

            # start offboard watchdog task if not running
            if not self._offboard_watchdog_task or self._offboard_watchdog_task.done():
                self._offboard_watchdog_task = asyncio.create_task(self._offboard_watchdog_task_fn())

            # helper: compute target lat/lon/alt for a given phase
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
                    return leader_lat, leader_lon, float(params.get("altitude_m", MISSION_DEFAULT_ALT))

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

                if name == "MOVE_FORWARD" or name == "MOVE_BACK":
                    forward_m = float(params.get("forward_m", MISSION_FORWARD_DIST))
                    if name == "MOVE_BACK":
                        forward_m = -forward_m

                    # compute leader heading (north/east vector) using previous leader pos
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

            # iterate phases
            for phase in phases:
                pname = phase.get("name")
                pdur = float(phase.get("duration", 0.0))
                log.info(f"Mission phase: {pname} duration={pdur:.1f}s")

                # GPS check for horizontal phases
                if pname in ("LINE", "TRI_H", "RETURN", "MOVE_FORWARD", "MOVE_BACK"):
                    try:
                        g = await self.drone.telemetry.gps_info().__anext__()
                        sats = getattr(g, "num_satellites", 0)
                        if sats < MIN_GPS_SATELLITES:
                            log.error(f"GPS sats too low ({sats}) for phase {pname} -> hold/skip phase")
                            if pdur > 0:
                                endt = time.time() + pdur
                                while time.time() < endt:
                                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                                    await asyncio.sleep(0.5)
                            continue
                    except Exception:
                        log.warning("GPS info unavailable; proceeding cautiously")

                if pname == "TAKEOFF":
                    _, _, tgt_alt = await compute_target_for_phase(phase)
                    await self.smooth_takeoff(tgt_alt)

                elif pname in ("LINE", "TRI_H", "TRI_V", "MOVE_FORWARD", "MOVE_BACK", "RETURN"):
                    tgt_lat, tgt_lon, tgt_alt = await compute_target_for_phase(phase)
                    log.info(f"Navigating to {pname} target lat={tgt_lat:.6f} lon={tgt_lon:.6f} alt={tgt_alt:.2f}")
                    phase_start = time.time()
                    timeout = max(pdur + 10.0, 30.0)
                    reached = False
                    while time.time() - phase_start < timeout:
                        pos = await self.drone.telemetry.position().__anext__()
                        cur_lat = pos.latitude_deg; cur_lon = pos.longitude_deg; cur_alt = pos.relative_altitude_m
                        err_n = (tgt_lat - cur_lat) * 111320.0
                        err_e = (tgt_lon - cur_lon) * 111320.0 * math.cos(math.radians(cur_lat))
                        err_alt = tgt_alt - cur_alt
                        dist2d = math.hypot(err_n, err_e)
                        if dist2d < DEADZONE_RADIUS and abs(err_alt) < 0.4:
                            await self._set_smooth_velocity(0.0, 0.0, 0.0)
                            reached = True
                            break
                        vx = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_n * POSITION_P_GAIN))
                        vy = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_e * POSITION_P_GAIN))
                        vz = max(-0.8, min(0.8, -err_alt * ALTITUDE_P_GAIN))
                        await self._set_smooth_velocity(vx, vy, vz)
                        await asyncio.sleep(0.12)
                    if not reached:
                        log.warning(f"Phase {pname} navigation timed out after {time.time() - phase_start:.1f}s")
                        await self._set_smooth_velocity(0.0, 0.0, 0.0)
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
                    await self.smooth_land(pre_land_alt=1.2, final_touchdown_timeout=40.0)
                else:
                    log.warning(f"Unknown phase {pname}")

            # finish: stop offboard, try disarm when on ground
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
                await asyncio.sleep(0.12)
                await self.drone.offboard.stop()
                self._offboard_active = False
            except Exception:
                pass

            try:
                in_air = await self.drone.telemetry.in_air().__anext__()
                if not in_air:
                    try:
                        await self.drone.action.disarm()
                        log.info("Disarmed after mission")
                    except Exception:
                        pass
            except Exception:
                pass

            log.info("Mission execution finished")
        except asyncio.CancelledError:
            log.info("Mission executor cancelled -> cleanup offboard")
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
                await asyncio.sleep(0.12)
                await self.drone.offboard.stop()
            except Exception:
                pass
            raise
        except Exception as e:
            log.error(f"Mission executor error: {e}")
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
                await asyncio.sleep(0.12)
            except Exception:
                pass

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
                        log.warning("Offboard setpoint timeout detected -> safe fallback (stop offboard & hold)")
                        try:
                            await self.drone.offboard.stop()
                        except Exception:
                            pass
                        try:
                            await self.drone.action.hold()
                        except Exception:
                            pass
                        # sleep more to avoid rapid flapping
                        await asyncio.sleep(1.0)
                await asyncio.sleep(OFFBOARD_WATCHDOG_INTERVAL)
        except asyncio.CancelledError:
            log.info("Offboard watchdog cancelled")
            return

    # ---------------- takeoff & landing helpers ----------------
    async def smooth_takeoff(self, target_alt):
        log.info(f"Smooth takeoff to {target_alt:.2f} m")
        try:
            start_time = time.time()
            pos = await self.drone.telemetry.position().__anext__()
            cur_alt = pos.relative_altitude_m
            while cur_alt < target_alt - 0.2 and time.time() - start_time < 40:
                pos = await self.drone.telemetry.position().__anext__()
                cur_alt = pos.relative_altitude_m
                remaining = target_alt - cur_alt
                if remaining > 2.0:
                    speed = 0.5
                elif remaining > 0.5:
                    speed = 0.35
                else:
                    speed = 0.15
                await self._set_smooth_velocity(0.0, 0.0, -speed)
                await asyncio.sleep(0.12)
            await self._set_smooth_velocity(0.0, 0.0, 0.0)
            log.info("Takeoff complete")
        except Exception as e:
            log.error(f"Takeoff error: {e}")

    async def smooth_land(self, pre_land_alt: float = 1.5, final_touchdown_timeout: float = 40.0):
        log.info(f"Hybrid smooth land (pre_land_alt={pre_land_alt:.2f})")
        try:
            try:
                pos = await self.drone.telemetry.position().__anext__()
                current_alt = pos.relative_altitude_m
            except Exception:
                current_alt = 5.0
            start_t = time.time()
            while current_alt > pre_land_alt and time.time() - start_t < 60:
                pos = await self.drone.telemetry.position().__anext__()
                current_alt = pos.relative_altitude_m
                remaining = current_alt - pre_land_alt
                if remaining > 3.0:
                    speed = 0.3
                elif remaining > 1.0:
                    speed = 0.18
                else:
                    speed = 0.08
                await self._set_smooth_velocity(0.0, 0.0, speed)
                await asyncio.sleep(0.12)

            log.info("Stopping offboard and calling action.land()")
            try:
                await self._set_smooth_velocity(0.0, 0.0, 0.0)
                await asyncio.sleep(0.12)
                await self.drone.offboard.stop()
                self._offboard_active = False
            except Exception as e:
                log.warning(f"offboard.stop failed: {e}")
            await asyncio.sleep(0.5)
            try:
                await self.drone.action.land()
            except Exception as e:
                log.error(f"action.land failed: {e} -> fallback to slow offboard descent")
                fallback_start = time.time()
                while current_alt > 0.3 and time.time() - fallback_start < 30:
                    pos = await self.drone.telemetry.position().__anext__()
                    current_alt = pos.relative_altitude_m
                    await self._set_smooth_velocity(0.0, 0.0, 0.06)
                    await asyncio.sleep(0.12)

            t0 = time.time()
            while time.time() - t0 < final_touchdown_timeout:
                try:
                    armed = await self.drone.telemetry.armed().__anext__()
                    in_air = await self.drone.telemetry.in_air().__anext__()
                    if not in_air and not armed:
                        log.info("Landed and disarmed")
                        return
                    if not in_air and armed:
                        log.info("Landed but still armed -> attempting disarm")
                        try:
                            await self.drone.action.disarm()
                        except Exception:
                            pass
                    await asyncio.sleep(0.5)
                except Exception:
                    await asyncio.sleep(0.5)
            log.warning("Landing timeout expired")
        except Exception as e:
            log.error(f"smooth_land exception: {e}")
            try:
                await self.drone.offboard.stop()
            except Exception:
                pass

    # ---------------- helper: battery ----------------
    async def _get_battery_fraction(self) -> float:
        try:
            b = await self.drone.telemetry.battery().__anext__()
            frac = getattr(b, "remaining_percent", None)
            if frac is None:
                return 1.0
            return float(frac)
        except Exception:
            return 1.0

    # ---------------- leader watchdog / orchestrator ----------------
    async def leader_watchdog(self):
        while not self._stop:
            now = time.time()
            if self.leader_id is None:
                if len(self.peers) > 0:
                    log.info("No leader known -> initiating election")
                    await self.start_election()
            else:
                if self.is_leader:
                    if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                        self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
                else:
                    if now - self.leader_last_ts > LEADER_HEARTBEAT_TIMEOUT:
                        log.warning(f"Leader {self.leader_id} stale -> electing new leader")
                        self.leader_id = None
                        await self.start_election()
            await asyncio.sleep(0.25)

    # ---------------- run ----------------
    async def run(self):
        await self.connect_mavsdk()

        # start core tasks
        self._presence_task = asyncio.create_task(self.presence_loop())
        self._udp_listener_task = asyncio.create_task(self.udp_listener())
        self._leader_watchdog_task = asyncio.create_task(self.leader_watchdog())

        # start offboard watchdog task (idle until offboard active)
        if not self._offboard_watchdog_task:
            self._offboard_watchdog_task = asyncio.create_task(self._offboard_watchdog_task_fn())

        log.info(f"Node {self.node_id} running; listening on UDP {self.listen_port}; broadcast {self.bcast_ip}")
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
    p = argparse.ArgumentParser()
    p.add_argument("--node-id", type=int, required=True, help="unique numeric node id (higher wins election)")
    p.add_argument("--mav", type=str, default="udpin://:14542", help="MAVSDK connection string (e.g. udpin://:14541 or serial:///dev/ttyAMA0:921600)")
    p.add_argument("--listen-port", type=int, default=DEFAULT_LISTEN_PORT, help="UDP listen/broadcast port")
    p.add_argument("--bcast-ip", type=str, default=DEFAULT_BCAST_IP, help="broadcast ip (e.g. 255.255.255.255 or 192.168.1.255 or 127.0.0.1)")
    args = p.parse_args()

    node = FollowerMultiSafe(node_id=args.node_id, mav_conn=args.mav, listen_port=args.listen_port, bcast_ip=args.bcast_ip)
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        log.info("Exiting on user interrupt")


if __name__ == "__main__":
    main()
