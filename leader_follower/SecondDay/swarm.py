#!/usr/bin/env python3
"""
sentry_swarm_advanced_mission.py

This definitive script merges a pilot-triggered "Sentry Mode" with the user's
original, complex multi-formation swarm mission.

- Leader Election: The drone with the highest node ID is automatically elected leader.
- Pilot-Triggered Sentry Mode: The leader waits for a pilot to manually switch to
  OFFBOARD mode. This is the sole trigger for the entire swarm operation.
- Original Complex Mission: Once triggered, the swarm executes the full mission sequence:
  TAKEOFF -> LINE -> HOLD -> TRI_H -> MOVE_FORWARD -> HOLD ->
  TRI_V -> MOVE_BACK -> HOLD -> RETURN -> LAND
- Global Emergency Hold: A fault in any drone triggers a broadcasted emergency
  signal, causing the entire swarm to immediately and safely enter a HOLD state.
- Advanced Flight Primitives: Utilizes controlled takeoff and smart, dual-condition
  landing for enhanced safety and precision.
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
LEADER_HEARTBEAT_TIMEOUT = 3.0

# Mission defaults (from original script)
MISSION_DEFAULT_ALT = 5.0
MISSION_LINE_SPACING = 3.0
MISSION_TRIANGLE_SIDE = 4.0
PHASE_HOLD_SECONDS = 4.0
MISSION_FORWARD_DIST = 5.0

# Offboard & Control
FLIGHT_SPEED = 0.8
POSITION_P_GAIN = 0.22
ALTITUDE_P_GAIN = 0.35
POSITION_TOLERANCE = 0.8  # Formerly DEADZONE_RADIUS

# Safety thresholds
BATTERY_LAND_THRESHOLD = 0.18
MIN_GPS_SATELLITES = 6
OFFBOARD_SETPOINT_TIMEOUT = 0.8
OFFBOARD_WATCHDOG_INTERVAL = 0.25
ARM_RETRY = 3
LANDING_DESCEND_SPEED = 0.4
LANDING_FLARE_SPEED = 0.2
LANDING_FLARE_ALTITUDE = 1.5
# ------------------------------------------------

logging.basicConfig(level=logging.INFO, format="%(asctime)s [SWARM] (Node-%(node_id)s) %(message)s")

def latlon_from_north_east(lat_ref, lon_ref, north_m, east_m):
    lat = lat_ref + (north_m / 111320.0)
    lon = lon_ref + (east_m / (111320.0 * math.cos(math.radians(lat_ref))))
    return lat, lon

class SentrySwarmNode:
    def __init__(self, node_id: int, mav_conn: str, listen_port: int, bcast_ip: str):
        self.node_id = node_id
        self.mav_conn = mav_conn
        self.listen_port = listen_port
        self.bcast_ip = bcast_ip

        # Logging Adapter
        self.log = logging.getLogger("sentry_swarm_mission")
        self.log = logging.LoggerAdapter(self.log, {'node_id': self.node_id})

        # MAVSDK and State
        self.drone = System()
        self.mission_aborted = False
        self._offboard_active = False
        self._last_setpoint_time = 0.0

        # Swarm State
        self.peers: Dict[int, Dict[str, Any]] = {}
        self.leader_id: Optional[int] = None
        self.leader_last_ts: float = 0.0
        self.leader_telemetry: Dict[str, Any] = {}
        self._leader_prev_pos: Optional[Dict[str, float]] = None
        self.is_leader = False

        # Networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setblocking(False)
        self.sock.bind(('', self.listen_port))
        
        # Mission State
        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_start_time: Optional[float] = None
        self._mission_exec_task: Optional[asyncio.Task] = None
        self._leader_telemetry_task: Optional[asyncio.Task] = None

        self._stop = False

    # --- Networking and Swarm Management ---
    def send_msg(self, obj: dict):
        dst = (self.bcast_ip, self.listen_port)
        try:
            self.sock.sendto(json.dumps(obj).encode('utf-8'), dst)
        except Exception as e:
            self.log.error(f"UDP send error to {dst}: {e}")

    async def presence_loop(self):
        while not self._stop:
            payload = {"type": "PRESENCE", "sender_id": self.node_id}
            self.send_msg(payload)
            await asyncio.sleep(1.0 / PRESENCE_RATE_HZ)

    async def udp_listener(self):
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(8192)
                msg = json.loads(data.decode('utf-8'))
                ptype, sid = msg.get("type"), msg.get("sender_id")

                if ptype == "PRESENCE" and sid is not None:
                    self.peers[sid] = {"last_ts": time.time()}
                
                elif ptype == "TELEMETRY" and sid is not None and (self.leader_id is None or sid == self.leader_id):
                    self._leader_prev_pos = self.leader_telemetry.copy()
                    self.leader_id, self.leader_last_ts = sid, time.time()
                    self.leader_telemetry = msg.get("telemetry", {})
                
                elif ptype == "COORDINATOR":
                    self.leader_id, self.leader_last_ts = msg.get("leader_id"), time.time()
                    self.is_leader = (self.leader_id == self.node_id)
                    self.log.info(f"Coordinator announced: {self.leader_id}. My role: {'Leader' if self.is_leader else 'Follower'}")
                    if self.is_leader and (not self._leader_telemetry_task or self._leader_telemetry_task.done()):
                        self._leader_telemetry_task = asyncio.create_task(self.leader_telemetry_loop())

                elif ptype == "MISSION": self.current_mission = msg.get("mission")
                elif ptype == "MISSION_START":
                    self.mission_start_time = msg.get("start_time")
                    self.log.info(f"Received MISSION_START, will begin at {self.mission_start_time}")
                    if self._mission_exec_task and not self._mission_exec_task.done(): self._mission_exec_task.cancel()
                    self._mission_exec_task = asyncio.create_task(self._mission_executor())
                
                elif ptype == "EMERGENCY_HOLD":
                    self.log.warning(f"EMERGENCY from Node {sid}! Entering global HOLD.")
                    if self._mission_exec_task and not self._mission_exec_task.done(): self._mission_exec_task.cancel()
                    asyncio.create_task(self.enter_emergency_hold())

            except (BlockingIOError, json.JSONDecodeError, KeyError): await asyncio.sleep(0.01)
            except Exception as e: self.log.error(f"UDP listener error: {e}")

    async def leader_watchdog(self):
        while not self._stop:
            active_nodes = [pid for pid, data in self.peers.items() if time.time() - data["last_ts"] < 5.0]
            all_known_nodes = active_nodes + [self.node_id]
            potential_leader = max(all_known_nodes) if all_known_nodes else self.node_id
            
            if self.leader_id != potential_leader:
                self.leader_id, self.is_leader = potential_leader, (self.node_id == potential_leader)
                if self.is_leader:
                    self.log.info(f"I am the new LEADER. Announcing.")
                    self.send_msg({"type": "COORDINATOR", "sender_id": self.node_id, "leader_id": self.node_id})
            
            await asyncio.sleep(2.0)

    async def leader_telemetry_loop(self):
        while self.is_leader and not self._stop:
            try:
                pos = await self.drone.telemetry.position().__anext__()
                tele = {"latitude_deg": pos.latitude_deg, "longitude_deg": pos.longitude_deg, "relative_altitude_m": pos.relative_altitude_m}
                payload = {"type": "TELEMETRY", "sender_id": self.node_id, "telemetry": tele, "is_leader": True}
                self.send_msg(payload)
                await asyncio.sleep(1.0 / TELEMETRY_BCAST_RATE_HZ)
            except Exception as e: self.log.error(f"Telemetry loop error: {e}")

    # --- Safety and Emergency ---
    async def enter_emergency_hold(self):
        self.log.warning("Entering EMERGENCY HOLD sequence.")
        try:
            if self._offboard_active:
                await self.drone.offboard.stop()
                self._offboard_active = False
            await self.drone.action.hold()
        except Exception as e: self.log.error(f"Failed to enter HOLD during emergency: {e}")

    async def offboard_watchdog(self):
        while not self._stop:
            if self._offboard_active and (time.time() - self._last_setpoint_time > OFFBOARD_SETPOINT_TIMEOUT):
                self.log.error("Offboard watchdog triggered! Broadcasting emergency.")
                self.send_msg({"type": "EMERGENCY_HOLD", "sender_id": self.node_id})
                await self.enter_emergency_hold()
            await asyncio.sleep(OFFBOARD_WATCHDOG_INTERVAL)

    # --- Advanced Flight Primitives ---
    async def _set_velocity(self, vx, vy, vz):
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, 0.0))
            self._last_setpoint_time = time.time()
        except OffboardError as e: self.log.debug(f"Set velocity error: {e}")

    async def _is_still_in_offboard(self):
        try:
            if str(await self.drone.telemetry.flight_mode().__anext__()) != "OFFBOARD":
                self.log.warning("Pilot disengaged OFFBOARD! Aborting mission.")
                self.mission_aborted = True
                return False
            return True
        except Exception:
            self.mission_aborted = True
            return False

    async def controlled_takeoff(self, target_altitude):
        self.log.info(f"Executing controlled takeoff to {target_altitude}m...")
        initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
        while not self.mission_aborted:
            if not await self._is_still_in_offboard(): return False
            current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            remaining_alt = target_altitude - (current_alt - initial_alt)
            if remaining_alt <= POSITION_TOLERANCE:
                await self._set_velocity(0.0, 0.0, 0.0); return True
            speed = FLIGHT_SPEED * max(0.2, min(1.0, remaining_alt / (target_altitude * 0.8)))
            await self._set_velocity(0.0, 0.0, -speed); await asyncio.sleep(0.1)
        return False

    async def smart_land(self):
        self.log.info("Executing smart landing...")
        last_alt, no_alt_change_start = -1, None
        while not self.mission_aborted:
            if not await self._is_still_in_offboard(): return False
            current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            speed = LANDING_DESCEND_SPEED if current_alt > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
            await self._set_velocity(0.0, 0.0, speed)
            landed_telemetry = str(await self.drone.telemetry.landed_state().__anext__()) == "ON_GROUND"
            if abs(current_alt - last_alt) < 0.05:
                if no_alt_change_start is None: no_alt_change_start = time.time()
            else: no_alt_change_start = None
            last_alt = current_alt
            landed_logic = (no_alt_change_start is not None and time.time() - no_alt_change_start > 2.0)
            if landed_telemetry and landed_logic: return True
            await asyncio.sleep(0.2)
        return False

    # --- Mission Planning and Execution (from original script, adapted for new framework) ---
    async def leader_plan_and_start_mission(self):
        self.log.info("Leader is planning and broadcasting the mission.")
        participants = sorted(list(self.peers.keys()) + [self.node_id])
        mission = {
            "participants": participants,
            "phases": [
                {"name": "TAKEOFF", "params": {"altitude_m": MISSION_DEFAULT_ALT}},
                {"name": "LINE", "duration": PHASE_HOLD_SECONDS, "params": {"spacing_m": MISSION_LINE_SPACING}},
                {"name": "HOLD", "duration": 2.0}, {"name": "TRI_H", "duration": PHASE_HOLD_SECONDS, "params": {"side_m": MISSION_TRIANGLE_SIDE}},
                {"name": "MOVE_FORWARD", "duration": PHASE_HOLD_SECONDS, "params": {"forward_m": MISSION_FORWARD_DIST}},
                {"name": "HOLD", "duration": 2.0}, {"name": "TRI_V", "duration": PHASE_HOLD_SECONDS, "params": {"side_m": MISSION_TRIANGLE_SIDE}},
                {"name": "MOVE_BACK", "duration": PHASE_HOLD_SECONDS, "params": {"forward_m": MISSION_FORWARD_DIST}},
                {"name": "HOLD", "duration": 2.0}, {"name": "RETURN"}, {"name": "LAND"}
            ]
        }
        self.send_msg({"type": "MISSION", "sender_id": self.node_id, "mission": mission})
        start_time = time.time() + 4.0
        self.send_msg({"type": "MISSION_START", "sender_id": self.node_id, "start_time": start_time})
        self.current_mission, self.mission_start_time = mission, start_time
        if self._mission_exec_task and not self._mission_exec_task.done(): self._mission_exec_task.cancel()
        self._mission_exec_task = asyncio.create_task(self._mission_executor())

    async def _mission_executor(self):
        try:
            wait_for = self.mission_start_time - time.time()
            if wait_for > 0: await asyncio.sleep(wait_for)
            
            participants = sorted(self.current_mission["participants"])
            my_index, total = participants.index(self.node_id), len(participants)
            
            if not self.is_leader:
                await self.drone.action.arm()
                await self._set_velocity(0.0, 0.0, 0.0); await asyncio.sleep(0.1)
                await self.drone.offboard.start()
                self._offboard_active = True
            
            # --- HELPER: Formation Logic ---
            def compute_target(phase):
                ref = self.leader_telemetry
                leader_lat, leader_lon, leader_alt = ref['latitude_deg'], ref['longitude_deg'], ref['relative_altitude_m']
                name, params = phase["name"], phase.get("params", {})
                
                if name == "TAKEOFF": return leader_lat, leader_lon, params["altitude_m"]
                if name == "LINE":
                    offset = (my_index - (total - 1) / 2.0) * params["spacing_m"]
                    return latlon_from_north_east(leader_lat, leader_lon, 0.0, offset) + (leader_alt,)
                if name == "TRI_H":
                    R, angle_rad = params["side_m"] / math.sqrt(3.0), math.radians([90, 210, 330][my_index % 3])
                    return latlon_from_north_east(leader_lat, leader_lon, R * math.cos(angle_rad), R * math.sin(angle_rad)) + (leader_alt,)
                if name in ("MOVE_FORWARD", "MOVE_BACK"):
                    dist = params["forward_m"] if name == "MOVE_FORWARD" else -params["forward_m"]
                    n, e = 1.0, 0.0
                    if self._leader_prev_pos:
                        dn = (leader_lat - self._leader_prev_pos['latitude_deg']) * 111320.0
                        de = (leader_lon - self._leader_prev_pos['longitude_deg']) * 111320.0 * math.cos(math.radians(leader_lat))
                        norm = math.hypot(dn, de)
                        if norm > 0.1: n, e = dn / norm, de / norm
                    return latlon_from_north_east(leader_lat, leader_lon, n * dist, e * dist) + (leader_alt,)
                if name == "TRI_V":
                    side = params["side_m"]
                    offsets = [(0, 0, side/2.0), (side/2.0, 0, -side/4.0), (-side/2.0, 0, -side/4.0)][my_index % 3]
                    lat, lon = latlon_from_north_east(leader_lat, leader_lon, offsets[0], offsets[1])
                    return lat, lon, leader_alt + offsets[2]
                if name == "RETURN": return compute_target({"name": "LINE", "params": {"spacing_m": MISSION_LINE_SPACING}})
                if name == "LAND": return leader_lat, leader_lon, 0.0
                return leader_lat, leader_lon, leader_alt

            # --- Main Phase Loop ---
            for phase in self.current_mission["phases"]:
                if self.mission_aborted: break
                pname, duration = phase["name"], phase.get("duration", 0.0)
                self.log.info(f"Executing Phase: {pname}")

                if pname == "TAKEOFF":
                    if not await self.controlled_takeoff(phase["params"]["altitude_m"]): raise RuntimeError("Takeoff aborted")
                elif pname == "LAND":
                    if not await self.smart_land(): raise RuntimeError("Landing aborted")
                elif pname == "HOLD":
                    end_time = time.time() + duration
                    while time.time() < end_time and not self.mission_aborted:
                        if not await self._is_still_in_offboard(): break
                        await self._set_velocity(0.0, 0.0, 0.0); await asyncio.sleep(0.2)
                else: # All movement phases
                    tgt_lat, tgt_lon, tgt_alt = compute_target(phase)
                    end_time = time.time() + (duration if duration > 0 else 30.0) # Timeout for non-duration phases
                    while time.time() < end_time and not self.mission_aborted:
                        if not await self._is_still_in_offboard(): break
                        pos = await self.drone.telemetry.position().__anext__()
                        err_n = (tgt_lat - pos.latitude_deg) * 111320.0
                        err_e = (tgt_lon - pos.longitude_deg) * 111320.0 * math.cos(math.radians(pos.latitude_deg))
                        err_alt = tgt_alt - pos.relative_altitude_m
                        if math.hypot(err_n, err_e) < POSITION_TOLERANCE and abs(err_alt) < 0.5: break # Reached target
                        vx, vy = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_n * POSITION_P_GAIN)), max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_e * POSITION_P_GAIN))
                        vz = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, -err_alt * ALTITUDE_P_GAIN))
                        await self._set_velocity(vx, vy, vz); await asyncio.sleep(0.1)

            self.log.info("Mission sequence finished.")
        except (asyncio.CancelledError, RuntimeError) as e:
            if not self.mission_aborted: self.log.error(f"Mission executor stopped: {e}")
        finally:
            if self._offboard_active: await self.drone.offboard.stop(); self._offboard_active = False
            await asyncio.sleep(1)
            if str(await self.drone.telemetry.landed_state().__anext__()) == "ON_GROUND": await self.drone.action.disarm()

    # --- Main Sentry Loop ---
    async def run(self):
        await self.drone.connect(system_address=self.mav_conn); self.log.info("Drone connected.")
        asyncio.gather(self.presence_loop(), self.udp_listener(), self.leader_watchdog(), self.offboard_watchdog())
        while self.leader_id is None: await asyncio.sleep(1)

        while not self._stop:
            if self.is_leader:
                flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
                if flight_mode == "OFFBOARD":
                    self.log.info("LEADER: OFFBOARD triggered by pilot. Starting swarm mission.")
                    try:
                        self.mission_aborted = False
                        await self.drone.action.arm()
                        await self._set_velocity(0.0, 0.0, 0.0); await asyncio.sleep(0.1)
                        await self.drone.offboard.start(); self._offboard_active = True
                        await self.leader_plan_and_start_mission()
                        if self._mission_exec_task: await self._mission_exec_task
                    finally:
                        if self._offboard_active: await self.drone.offboard.stop(); self._offboard_active = False
                        self.log.info(f"Mission concluded {'by pilot abort' if self.mission_aborted else 'normally'}. Returning to Sentry.")
                        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD": await asyncio.sleep(1)
                else: await self._set_velocity(0.0, 0.0, 0.0) # Prime the system while waiting
            await asyncio.sleep(0.5)

def main():
    parser = argparse.ArgumentParser(description="Pilot-triggered advanced swarm mission.")
    parser.add_argument("--node-id", type=int, required=True, help="Unique numeric ID for this drone.")
    parser.add_argument("--mav", type=str, default="udp://:14540", help="MAVSDK connection string.")
    parser.add_argument("--listen-port", type=int, default=DEFAULT_LISTEN_PORT)
    parser.add_argument("--bcast-ip", type=str, default=DEFAULT_BCAST_IP)
    args = parser.parse_args()
    node = SentrySwarmNode(**vars(args))
    asyncio.run(node.run())

if __name__ == "__main__":
    main()