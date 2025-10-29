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

# Import colorlog for colored logging
import colorlog

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
# Set to DEBUG to see high-frequency logs like movement progress
LOG_LEVEL = logging.INFO 
# ------------------------------------------------

# Logger instance will be configured in the main function
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
        self.peers: Dict[int, Dict[str, Any]] = {}
        self.failed_peers: Dict[int, Dict[str, float]] = {}
        self.leader_id: Optional[int] = None
        self.leader_last_ts: float = 0.0
        self.leader_telemetry: Dict[str, Any] = {}
        self._leader_prev_pos: Optional[Dict[str, float]] = None
        self.is_leader = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            log.debug("SO_BROADCAST not available")
        self.sock.setblocking(False)
        self.sock.bind(('', self.listen_port))
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self._offboard_active = False
        self._last_setpoint_time = 0.0
        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_start_time: Optional[float] = None
        self.current_phase_name: str = "Idle"
        self._mission_exec_task: Optional[asyncio.Task] = None
        self._leader_telemetry_task: Optional[asyncio.Task] = None
        self._presence_task: Optional[asyncio.Task] = None
        self._udp_listener_task: Optional[asyncio.Task] = None
        self._leader_watchdog_task: Optional[asyncio.Task] = None
        self._offboard_watchdog_task: Optional[asyncio.Task] = None
        self._mavsdk_monitor_task: Optional[asyncio.Task] = None
        self._status_log_task: Optional[asyncio.Task] = None
        self._telemetry_seq = 0
        self._last_election_ack = 0.0
        self._stop = False

    async def status_logger_task(self):
        """Logs a concise status update every second."""
        while not self._stop:
            try:
                role = "Leader" if self.is_leader else f"Follower (L: {self.leader_id})"
                if self.leader_id is None:
                    role = "Discovering"
                
                phase_status = self.current_phase_name or "N/A"
                
                # This is the periodic status log
                log.info(f"Role: {role} | Phase: {phase_status}")
                await asyncio.sleep(1.0)
            except asyncio.CancelledError:
                break
            except Exception as e:
                log.error(f"Status logger error: {e}")
                await asyncio.sleep(1.0)

    def send_msg(self, obj: dict, target_ip: Optional[str] = None, target_port: Optional[int] = None):
        dst = (target_ip or self.bcast_ip, target_port or self.listen_port)
        try:
            self.sock.sendto(json.dumps(obj).encode('utf-8'), dst)
            log.debug(f"Sent {obj.get('type')} -> {dst}")
        except Exception as e:
            log.error(f"UDP send error to {dst}: {e}")

    async def connect_mavsdk(self):
        log.info(f"Connecting to MAVSDK @ {self.mav_conn}")
        await self.drone.connect(system_address=self.mav_conn)
        async for st in self.drone.core.connection_state():
            if st.is_connected:
                log.info("MAVSDK connected successfully.")
                break
        self._mavsdk_monitor_task = asyncio.create_task(self._mavsdk_connection_monitor())

    async def _mavsdk_connection_monitor(self):
        try:
            async for state in self.drone.core.connection_state():
                if not state.is_connected:
                    log.error("Lost connection to flight controller! Forcing HOLD.")
                    try:
                        await self.drone.action.hold()
                    except Exception:
                        pass
                    # No need to loop further, the connection is lost.
                    return
        except asyncio.CancelledError:
            pass

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
                try: payload = json.loads(data.decode('utf-8'))
                except Exception:
                    log.debug(f"Invalid JSON from {addr}")
                    continue

                ptype = payload.get("type")
                sid = payload.get("sender_id")
                if sid is not None:
                    try: sid = int(sid)
                    except (ValueError, TypeError): sid = None

                if ptype == "PRESENCE":
                    if sid is not None:
                        self.peers[sid] = {"ip": sender_ip, "port": sender_port, "last_ts": time.time(), "telemetry": {}}
                        log.debug(f"PRESENCE from {sid}")

                elif ptype == "TELEMETRY":
                    if sid is not None:
                        tele = payload.get("telemetry") or {}
                        self.peers[sid] = {"ip": sender_ip, "port": sender_port, "last_ts": time.time(), "telemetry": tele}
                        if self.leader_id is None or sid == self.leader_id or payload.get("is_leader"):
                            self.leader_id = sid
                            self.leader_last_ts = time.time()
                            self.leader_telemetry = tele
                            log.debug(f"RX TELEMETRY from leader {sid}")

                elif ptype == "ELECTION":
                    their_id = sid
                    log.info(f"Received ELECTION from node {their_id}")
                    if self.node_id > (their_id or -1):
                        log.info(f"My ID ({self.node_id}) is higher. Replying and starting my own election.")
                        ack = {"type": "ELECTION_ACK", "sender_id": self.node_id, "timestamp": time.time()}
                        peer = self.peers.get(their_id)
                        self.send_msg(ack, target_ip=peer["ip"], target_port=peer["port"]) if peer else self.send_msg(ack)
                        asyncio.create_task(self.start_election())

                elif ptype == "ELECTION_ACK":
                    log.debug(f"Received ELECTION_ACK from {sid}")
                    self._last_election_ack = time.time()

                elif ptype == "COORDINATOR":
                    new_leader = payload.get("leader_id")
                    if new_leader is not None:
                        new_leader = int(new_leader)
                        log.info(f"IMPORTANT: Coordinator announced. New leader is node {new_leader}.")
                        prev_leader = self.leader_id
                        self.leader_id = new_leader
                        self.leader_last_ts = time.time()
                        self.is_leader = (self.leader_id == self.node_id)
                        if self.is_leader:
                            log.info("This node is now the LEADER. Starting leader duties.")
                            if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                                self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
                                asyncio.create_task(self.leader_plan_and_start_mission())
                        elif prev_leader == self.node_id:
                            log.warning("This node was demoted from leader role.")
                            if self._leader_telemetry_task:
                                self._leader_telemetry_task.cancel()
                                self._leader_telemetry_task = None

                elif ptype == "MISSION":
                    self.current_mission = payload.get("mission")
                    log.info("Received new MISSION from leader.")

                elif ptype == "MISSION_START":
                    self.mission_start_time = payload.get("start_time")
                    log.info(f"Received MISSION_START command. Mission will start soon.")
                    if self._mission_exec_task and not self._mission_exec_task.done():
                        self._mission_exec_task.cancel()
                    self._mission_exec_task = asyncio.create_task(self._mission_executor())

                elif ptype == "MISSION_ABORT":
                    log.warning("Received MISSION_ABORT from leader!")
                    if self._mission_exec_task: self._mission_exec_task.cancel()

            except BlockingIOError: await asyncio.sleep(0.01)
            except Exception as e:
                log.error(f"UDP listener exception: {e}")
                await asyncio.sleep(0.1)

    async def start_election(self):
        log.info("Starting ELECTION process...")
        self._last_election_ack = 0.0
        msg = {"type": "ELECTION", "sender_id": self.node_id, "timestamp": time.time()}
        self.send_msg(msg)
        await asyncio.sleep(ELECTION_TIMEOUT)

        if self._last_election_ack > 0.0:
            log.info("Election response received from a higher ID. Waiting for a new leader to be announced.")
        else:
            log.info("No higher ID responded. Declaring self as the new LEADER.")
            self.leader_id = self.node_id
            self.is_leader = True
            await self.announce_coordinator()
            if not self._leader_telemetry_task or self._leader_telemetry_task.done():
                self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())
            asyncio.create_task(self.leader_plan_and_start_mission())

    async def announce_coordinator(self):
        coord_msg = {"type": "COORDINATOR", "sender_id": self.node_id, "leader_id": self.node_id}
        log.info("Broadcasting COORDINATOR announcement...")
        for _ in range(COORDINATOR_ANNOUNCE_RETRY):
            self.send_msg(coord_msg)
            await asyncio.sleep(0.15)
    
    # ... (leader_telemetry_loop and leader_plan_and_start_mission are mostly unchanged)
    async def leader_telemetry_loop(self):
        log.info("Leader telemetry loop started")
        try:
            async for p in self.drone.telemetry.position():
                tele = { "latitude_deg": p.latitude_deg, "longitude_deg": p.longitude_deg, "relative_altitude_m": p.relative_altitude_m }
                try: tele["is_in_air"] = bool(await self.drone.telemetry.in_air().__anext__())
                except: pass
                self._telemetry_seq += 1
                payload = {"type": "TELEMETRY", "sender_id": self.node_id, "seq": self._telemetry_seq, "timestamp": time.time(), "telemetry": tele, "is_leader": True}
                self.send_msg(payload)
                self.leader_telemetry = tele
                self.leader_last_ts = time.time()
                await asyncio.sleep(1.0 / max(1, TELEMETRY_BCAST_RATE_HZ))
        except asyncio.CancelledError: log.info("Leader telemetry loop cancelled")
        except Exception as e: log.error(f"Leader telemetry error: {e}")

    async def leader_plan_and_start_mission(self):
        await asyncio.sleep(2.0) # wait for peers to be discovered
        active_ids = sorted([pid for pid, info in self.peers.items() if time.time() - info["last_ts"] <= 3.0] + [self.node_id])
        participants = sorted(list(set(active_ids)))
        log.info(f"Planning mission for participants: {participants}")
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
        msg = {"type": "MISSION", "sender_id": self.node_id, "mission": mission}
        self.send_msg(msg)
        log.info("Mission description sent.")
        await asyncio.sleep(0.5)
        start_time = time.time() + 2.0
        start_msg = {"type": "MISSION_START", "sender_id": self.node_id, "start_time": start_time}
        self.send_msg(start_msg)
        log.info(f"Sent MISSION_START command for T+{start_time - time.time():.1f}s")
        self.current_mission = mission
        self.mission_start_time = start_time
        if self._mission_exec_task and not self._mission_exec_task.done(): self._mission_exec_task.cancel()
        self._mission_exec_task = asyncio.create_task(self._mission_executor())

    def _apply_exclusion(self, lat_t, lon_t, pname):
        shifted_lat, shifted_lon = lat_t, lon_t
        for fid, info in list(self.failed_peers.items()):
            f_lat = info.get("latitude_deg")
            f_lon = info.get("longitude_deg")
            if f_lat is None or f_lon is None: continue
            
            d = distance_m_between(shifted_lat, shifted_lon, f_lat, f_lon)
            if d < EXCLUSION_RADIUS:
                log.warning(f"SAFETY: Target for phase '{pname}' is in exclusion zone of peer {fid}. Shifting target away.")
                vn = (shifted_lat - f_lat) * 111320.0
                ve = (shifted_lon - f_lon) * 111320.0 * math.cos(math.radians((shifted_lat + f_lat) / 2.0))
                norm = math.hypot(vn, ve)
                if norm < 0.01:
                    shift_n, shift_e = EXCLUSION_RADIUS, 0.0
                else:
                    needed_shift = EXCLUSION_RADIUS - d + 0.2 # Add a small buffer
                    shift_n = (vn / norm) * needed_shift
                    shift_e = (ve / norm) * needed_shift
                
                shifted_lat, shifted_lon = latlon_from_north_east(shifted_lat, shifted_lon, shift_n, shift_e)
        return shifted_lat, shifted_lon

    async def _mission_executor(self):
        try:
            if not self.current_mission or not self.mission_start_time: return
            self.current_phase_name = "Waiting for start time"
            wait_for = self.mission_start_time - time.time()
            if wait_for > 0: await asyncio.sleep(wait_for)
            
            log.info("Mission execution is starting NOW.")
            participants = sorted(self.current_mission.get("participants", []))
            my_index = participants.index(self.node_id)
            total = len(participants)

            # ... (the compute_target_for_phase function is unchanged and can be copied from original)
            async def compute_target_for_phase(phase: Dict[str, Any]):
                ref = self.leader_telemetry
                if not ref or 'latitude_deg' not in ref:
                    pos = await self.drone.telemetry.position().__anext__()
                    return pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
                leader_lat, leader_lon, leader_alt = float(ref.get('latitude_deg')), float(ref.get('longitude_deg')), float(ref.get('relative_altitude_m'))
                name, params = phase.get("name"), phase.get("params", {})
                if name == "TAKEOFF": return leader_lat, leader_lon, float(params.get("altitude_m", MISSION_DEFAULT_ALT))
                if name == "LINE":
                    offset_index = my_index - (total - 1) / 2.0
                    east_off = offset_index * float(params.get("spacing_m", MISSION_LINE_SPACING))
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, 0.0, east_off)
                    return lat, lon, float(params.get("altitude_m", leader_alt))
                if name == "TRI_H":
                    side = float(params.get("side_m", MISSION_TRIANGLE_SIDE))
                    R, angles_deg = side / math.sqrt(3.0), [90.0, 210.0, 330.0]
                    angle = math.radians(angles_deg[my_index % 3])
                    north_off, east_off = R * math.cos(angle), R * math.sin(angle)
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, east_off)
                    return lat, lon, float(params.get("altitude_m", leader_alt))
                if name == "TRI_V":
                    side = float(params.get("side_m", MISSION_TRIANGLE_SIDE))
                    if my_index % 3 == 0: north_off, alt_off = 0.0, side / 2.0
                    elif my_index % 3 == 1: north_off, alt_off = side / 2.0, -side / 4.0
                    else: north_off, alt_off = -side / 2.0, -side / 4.0
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, north_off, 0.0)
                    return lat, lon, leader_alt + alt_off
                if name in ("MOVE_FORWARD", "MOVE_BACK"):
                    forward_m = float(params.get("forward_m", MISSION_FORWARD_DIST)) * (-1 if name == "MOVE_BACK" else 1)
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, forward_m, 0.0) # Simplified to move North/South relative to leader
                    return lat, lon, float(params.get("altitude_m", leader_alt))
                if name == "RETURN":
                    offset_index = my_index - (total - 1) / 2.0
                    east_off = offset_index * MISSION_LINE_SPACING
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, 0.0, east_off)
                    return lat, lon, leader_alt
                if name == "LAND": return leader_lat, leader_lon, 0.0
                return leader_lat, leader_lon, leader_alt

            for phase in self.current_mission.get("phases", []):
                pname = phase.get("name")
                self.current_phase_name = pname
                log.info(f"--- Starting Phase: {pname} ---")
                
                tgt_lat, tgt_lon, tgt_alt = await compute_target_for_phase(phase)
                tgt_lat, tgt_lon = self._apply_exclusion(tgt_lat, tgt_lon, pname)
                
                if pname == "TAKEOFF":
                    if not await self.controlled_takeoff(tgt_alt):
                        log.error("Takeoff failed, aborting mission.")
                        return
                elif pname in ("LINE", "TRI_H", "TRI_V", "MOVE_FORWARD", "MOVE_BACK", "RETURN"):
                    target_pos = Position(tgt_lat, tgt_lon, tgt_alt + 100, tgt_alt) # abs_alt placeholder
                    if not await self.move_to_absolute_target(target_pos):
                        log.warning(f"Navigation to {pname} failed or was interrupted.")
                elif pname == "LAND":
                    if not await self.smart_land():
                        log.error("Landing failed, aborting mission.")
                        return
                
                pdur = float(phase.get("duration", 0.0))
                if pdur > 0:
                    log.info(f"Holding position for {pdur:.1f} seconds.")
                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                    await asyncio.sleep(pdur)

            log.info("MISSION COMPLETE. Holding final position.")
        except asyncio.CancelledError: log.warning("Mission execution was cancelled.")
        except Exception as e: log.error(f"Mission executor unhandled error: {e}")
        finally: self.current_phase_name = "Idle"

    async def _set_smooth_velocity(self, vx: float, vy: float, vz: float):
        # ... (function is unchanged)
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

    async def _offboard_watchdog_task_fn(self):
        # ... (function is unchanged)
        while not self._stop:
            if self._offboard_active and (time.time() - self._last_setpoint_time > OFFBOARD_SETPOINT_TIMEOUT):
                log.warning("Offboard setpoint timeout! Forcing HOLD.")
                try: await self.drone.offboard.stop()
                except: pass
                try: await self.drone.action.hold()
                except: pass
            await asyncio.sleep(OFFBOARD_WATCHDOG_INTERVAL)
            
    async def controlled_takeoff(self, target_altitude):
        log.info(f"Executing controlled takeoff to {target_altitude}m...")
        try:
            initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            while True:
                current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
                if current_alt >= target_altitude - POSITION_TOLERANCE:
                    log.info("Target altitude reached.")
                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                    return True
                
                ascent_speed = FLIGHT_SPEED * max(0.2, min(1.0, (target_altitude - current_alt) / target_altitude))
                await self._set_smooth_velocity(0.0, 0.0, -ascent_speed)
                log.debug(f"Ascending... current alt: {current_alt:.1f}m")
                await asyncio.sleep(0.1)
        except Exception as e:
            log.error(f"Takeoff error: {e}")
            return False

    async def smart_land(self):
        log.info("Executing smart land...")
        try:
            while True:
                current_altitude = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
                speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
                await self._set_smooth_velocity(0.0, 0.0, speed)
                
                is_landed = await self.drone.telemetry.landed_state().__anext__()
                if is_landed == is_landed.ON_GROUND:
                    log.info("Ground contact confirmed. Landing complete.")
                    return True
                log.debug(f"Descending... current alt: {current_altitude:.1f}m")
                await asyncio.sleep(0.2)
        except Exception as e:
            log.error(f"Landing exception: {e}")
            return False

    async def move_to_absolute_target(self, target_pos: Position):
        log.info(f"Moving to target Lat: {target_pos.latitude_deg:.5f}, Lon: {target_pos.longitude_deg:.5f}")
        try:
            # Altitude is locked based on current altitude at the start of the move
            locked_altitude = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            while True:
                current_pos = await self.drone.telemetry.position().__anext__()
                dist = distance_m_between(current_pos.latitude_deg, current_pos.longitude_deg, target_pos.latitude_deg, target_pos.longitude_deg)
                
                if dist <= POSITION_TOLERANCE:
                    log.info("Target position reached.")
                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                    return True

                remaining_north = (target_pos.latitude_deg - current_pos.latitude_deg) * 111320.0
                remaining_east = (target_pos.longitude_deg - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
                
                speed_factor = max(0.1, min(1.0, dist / 2.0))
                velocity_n = speed_factor * FLIGHT_SPEED * (remaining_north / dist)
                velocity_e = speed_factor * FLIGHT_SPEED * (remaining_east / dist)
                
                altitude_error = locked_altitude - current_pos.relative_altitude_m
                velocity_z = altitude_error * ALTITUDE_P_GAIN_RECT # Corrective velocity
                
                await self._set_smooth_velocity(velocity_n, velocity_e, -velocity_z)
                log.debug(f"Moving... distance remaining: {dist:.1f}m")
                await asyncio.sleep(0.1)
        except Exception as e:
            log.error(f"move_to_absolute_target error: {e}")
            return False

    def update_failed_peers(self):
        now = time.time()
        for pid, info in list(self.peers.items()):
            if now - info.get("last_ts", 0) > STALE_TIMEOUT:
                if pid not in self.failed_peers:
                    tele = info.get("telemetry", {})
                    if 'latitude_deg' in tele:
                        self.failed_peers[pid] = {"latitude_deg": tele['latitude_deg'], "longitude_deg": tele['longitude_deg'], "ts": info["last_ts"]}
                        log.warning(f"SAFETY: Peer {pid} has gone stale. Activating exclusion zone.")
                    else:
                         log.warning(f"SAFETY: Peer {pid} has gone stale (no position data).")
            elif pid in self.failed_peers:
                log.info(f"Peer {pid} is back online. Removing exclusion zone.")
                del self.failed_peers[pid]

    async def leader_watchdog(self):
        while not self._stop:
            self.update_failed_peers()
            if self.leader_id is None:
                if len(self.peers) > 0:
                    await self.start_election()
            elif not self.is_leader and (time.time() - self.leader_last_ts > LEADER_HEARTBEAT_TIMEOUT):
                log.warning(f"Leader {self.leader_id} has gone stale. Starting a new election.")
                self.leader_id = None
                await self.start_election()
            await asyncio.sleep(1.0)

    async def run(self):
        await self.connect_mavsdk()
        log.info("Waiting for initial position estimate...")
        async for h in self.drone.telemetry.health():
            if h.is_global_position_ok:
                log.info("Position estimate is OK.")
                break
        
        log.info("Attempting to ARM...")
        for _ in range(ARM_RETRY):
            try:
                await self.drone.action.arm()
                log.info("ARMED successfully.")
                break
            except Exception as e: log.warning(f"Arming failed: {e}. Retrying...")
        else:
            log.error("Could not arm after retries. Exiting.")
            return

        log.info("Attempting to start OFFBOARD mode...")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        for _ in range(OFFBOARD_START_RETRY):
            try:
                await self.drone.offboard.start()
                self._offboard_active = True
                log.info("OFFBOARD mode started successfully. Drone is idle.")
                break
            except OffboardError as e: log.warning(f"Offboard start failed: {e}. Retrying...")
        else:
            log.error("Could not start offboard mode. Exiting.")
            return

        tasks = [
            self.presence_loop(), self.udp_listener(),
            self.leader_watchdog(), self._offboard_watchdog_task_fn(),
            self.status_logger_task()
        ]
        self._presence_task, self._udp_listener_task, self._leader_watchdog_task, \
        self._offboard_watchdog_task, self._status_log_task = tasks

        log.info(f"Node initialization complete. Running...")
        try: await asyncio.gather(*tasks)
        except KeyboardInterrupt: log.info("User interrupted. Shutting down.")
        finally:
            self._stop = True
            all_tasks = [
                self._presence_task, self._udp_listener_task, self._leader_watchdog_task,
                self._offboard_watchdog_task, self._mavsdk_monitor_task, self._leader_telemetry_task,
                self._mission_exec_task, self._status_log_task
            ]
            for t in all_tasks:
                if t: t.cancel()
            self.sock.close()

def setup_logging(node_id: int):
    handler = colorlog.StreamHandler()
    colors = ['cyan', 'magenta', 'yellow', 'green', 'blue', 'white']
    node_color = colors[node_id % len(colors)]
    formatter = colorlog.ColoredFormatter(
        f'%(log_color)s%(asctime)s.%(msecs)03d [%(bold)sNODE-{node_id}%(reset)s] %(message)s',
        datefmt='%H:%M:%S',
        log_colors={
            'DEBUG': 'white', 'INFO': node_color, 'WARNING': 'yellow',
            'ERROR': 'red', 'CRITICAL': 'bold_red',
        }
    )
    handler.setFormatter(formatter)
    logger = logging.getLogger("follower_multi_rect")
    logger.setLevel(LOG_LEVEL)
    if not logger.handlers:
        logger.addHandler(handler)

def main():
    # --- مقادیر ثابت ---
    # برای هر پهپاد، باید یک شناسه منحصر به فرد تنظیم کنید
    NODE_ID = 2
    MAV_CONN = "udp://:14540" # UDP is more common for SITL
    LISTEN_PORT = DEFAULT_LISTEN_PORT
    BCAST_IP = DEFAULT_BCAST_IP
    # --------------------
    setup_logging(node_id=NODE_ID)
    node = FollowerRect(node_id=NODE_ID, mav_conn=MAV_CONN, listen_port=LISTEN_PORT, bcast_ip=BCAST_IP)
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        log.info("Program terminated by user.")

if __name__ == "__main__":
    main()