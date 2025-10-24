#!/usr/bin/env python3

#
# follower_final.py
#
# Description:
# The definitive version of the Follower script. It combines the robust Sentry
# logic with intelligent, multi-layered safety protocols. The drone is passive
# until the pilot engages OFFBOARD mode. It then assesses the Leader's state
# and executes its mission, which includes a safe entry maneuver. It constantly
# checks for leader data sanity, a personal safety bubble, and its own battery
# level, ensuring maximum safety and reliability.
#

import asyncio
import math
import logging
import time
import socket
import json
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import Position

# --- Configuration ---
LOG_FILE = "follower.log"
CONNECTION_STRING = "udp://:14542"
LISTENING_PORT = 5005

# Flight Behavior
FLIGHT_SPEED = 1.5
POSITION_TOLERANCE = 0.5
ALTITUDE_P_GAIN = 2.0
CONNECTION_TIMEOUT_S = 2.0

# Formation Offset
OFFSET_NORTH_M = -5.0 # 5 meters BEHIND the leader
OFFSET_EAST_M = 0.0   # 0 meters to the side (in-line)

# State Triggers & Advanced Safety
TAKEOFF_TRIGGER_ALT_M = 1.5
LAND_TRIGGER_ALT_M = 1.0
TRANSITION_SAFETY_ALTITUDE = 3.0 # Meters ABOVE leader during transition
LEADER_MIN_SATELLITES = 10       # Leader data sanity check
MINIMUM_SAFE_DISTANCE = 3.0      # Personal safety bubble radius
CRITICAL_BATTERY_PERCENT = 0.20  # Self-preservation instinct

class Follower:
    """The autonomous wingman with ultimate safety logic."""
    def __init__(self):
        self.state = 'IDLE_ON_GROUND'
        self.drone = System()
        self.log = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s",
                            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()])
        
        self.leader_data = {}
        self.last_leader_message_time = 0
        self.my_battery = None

    async def _connect_and_prepare(self):
        self.log.info("Connecting to system...")
        await self.drone.connect(system_address=CONNECTION_STRING)
        async for state in self.drone.core.connection_state():
            if state.is_connected: self.log.info("System connected."); break

    async def controlled_takeoff(self, target_altitude):
        self.log.info(f"Executing controlled takeoff to {target_altitude}m...")
        initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            altitude_progress = current_alt - initial_alt
            if altitude_progress >= target_altitude - POSITION_TOLERANCE:
                self.log.info("Target altitude reached."); return True
            speed_factor = max(0.2, min(1.0, (target_altitude - altitude_progress) / (target_altitude * 0.8)))
            ascent_speed = FLIGHT_SPEED * speed_factor
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ascent_speed, 0.0))
            await asyncio.sleep(0.1)
        return False

    async def smart_land(self):
        self.log.info("Executing smart landing...")
        last_altitude, no_alt_change_start_time = -1, None
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            current_altitude = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            speed = 0.7 if current_altitude > 1.5 else 0.2
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))
            is_landed_telemetry = str(await self.drone.telemetry.landed_state().__anext__()) == "ON_GROUND"
            if abs(current_altitude - last_altitude) < 0.05:
                if no_alt_change_start_time is None: no_alt_change_start_time = time.time()
            else: no_alt_change_start_time = None
            last_altitude = current_altitude
            is_landed_logic = (no_alt_change_start_time is not None and time.time() - no_alt_change_start_time > 2.0)
            if is_landed_telemetry and is_landed_logic:
                self.log.info("Ground contact confirmed."); return True
            await asyncio.sleep(0.2)
        return False

    async def _background_tasks(self):
        asyncio.create_task(self._udp_listener_task())
        asyncio.create_task(self._battery_monitor_task())

    async def _udp_listener_task(self):
        self.log.info(f"Starting UDP listener on port {LISTENING_PORT}")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            sock.bind(('', LISTENING_PORT))
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    self.leader_data = json.loads(data.decode('utf-8'))
                    self.last_leader_message_time = time.time()
                except BlockingIOError: pass
                except Exception as e: self.log.error(f"UDP Listener error: {e}")
                await asyncio.sleep(0.01)

    async def _battery_monitor_task(self):
        async for battery in self.drone.telemetry.battery():
            self.my_battery = battery

    def calculate_distance_to_leader(self, current_pos):
        if "latitude_deg" not in self.leader_data: return float('inf')
        d_north = (self.leader_data["latitude_deg"] - current_pos.latitude_deg) * 111320.0
        d_east = (self.leader_data["longitude_deg"] - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
        d_alt = self.leader_data["relative_altitude_m"] - current_pos.relative_altitude_m
        return math.sqrt(d_north**2 + d_east**2 + d_alt**2)

    async def run(self):
        """The main Sentry loop that triggers the follower logic."""
        await self._connect_and_prepare()
        await self._background_tasks()

        self.log.info("--- Follower ready. Entering Sentry Mode. ---")
        self.log.info("Fly the Leader. Switch this drone to OFFBOARD to begin following.")
        
        try:
            while True:
                flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
                is_armed = await self.drone.telemetry.armed().__anext__()
                
                if flight_mode == "OFFBOARD":
                    self.log.info("Offboard engaged by pilot. Executing Offboard logic.")
                    await self.drone.offboard.start()
                    await self.execute_offboard_logic()
                    self.log.info("Offboard task complete. Stopping Offboard mode.")
                    await self.drone.offboard.stop()
                else:
                    if is_armed:
                        try:
                            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        except OffboardError: pass
                    await asyncio.sleep(0.2)
        except (asyncio.CancelledError, KeyboardInterrupt):
            self.log.info("Script interrupted by user.")
        finally:
            self.log.info("Entering final safety cleanup...")
            is_in_air = await self.drone.telemetry.in_air().__anext__()
            if is_in_air:
                self.log.warning("Drone is still in the air! Commanding HOLD immediately.")
                await self.drone.action.hold()
            self.log.info("Cleanup complete. Script terminating.")

    async def execute_offboard_logic(self):
        """Contains the decision tree for actions to take while in Offboard mode."""
        leader_alt = self.leader_data.get("relative_altitude_m", 0)
        leader_in_air = self.leader_data.get("is_in_air", False)
        am_i_in_air = await self.drone.telemetry.in_air().__anext__()

        if leader_in_air and leader_alt > TAKEOFF_TRIGGER_ALT_M and not am_i_in_air:
            await self.drone.action.arm()
            if await self.controlled_takeoff(leader_alt):
                await self.transition_to_formation_loop()
                await self.follow_loop()
        elif leader_in_air and am_i_in_air:
            await self.transition_to_formation_loop()
            await self.follow_loop()
        elif am_i_in_air and not leader_in_air:
            if await self.smart_land():
                await asyncio.sleep(1)
                await self.drone.action.disarm()
        else:
            await asyncio.sleep(1)

    async def transition_to_formation_loop(self):
        """Elevated Highway Maneuver for safe entry."""
        self.log.info("Executing safe transition to formation...")
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            leader_pos, current_pos = self.leader_data, await self.drone.telemetry.position().__anext__()
            
            # Phase 1: Climb to safe altitude
            safe_altitude = leader_pos["relative_altitude_m"] + TRANSITION_SAFETY_ALTITUDE
            altitude_error = safe_altitude - current_pos.relative_altitude_m
            if abs(altitude_error) > POSITION_TOLERANCE:
                velocity_z = - (altitude_error * ALTITUDE_P_GAIN)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, velocity_z, 0.0))
                await asyncio.sleep(0.1); continue

            # Phase 2: Reposition horizontally
            lat_offset_m = OFFSET_NORTH_M / 111320.0
            lon_offset_m = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_pos["latitude_deg"])))
            target_lat, target_lon = leader_pos["latitude_deg"] + lat_offset_m, leader_pos["longitude_deg"] + lon_offset_m
            remaining_north = (target_lat - current_pos.latitude_deg) * 111320.0
            remaining_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            distance_remaining = math.sqrt(remaining_north**2 + remaining_east**2)
            if distance_remaining > POSITION_TOLERANCE:
                velocity_n, velocity_e = remaining_north * FLIGHT_SPEED, remaining_east * FLIGHT_SPEED
                velocity_z = - (altitude_error * ALTITUDE_P_GAIN) # Maintain safe altitude
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, velocity_z, 0.0))
                await asyncio.sleep(0.1); continue

            # Phase 3: Descend into formation
            final_altitude_error = leader_pos["relative_altitude_m"] - current_pos.relative_altitude_m
            if abs(final_altitude_error) > POSITION_TOLERANCE:
                velocity_z = - (final_altitude_error * ALTITUDE_P_GAIN)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, velocity_z, 0.0))
                await asyncio.sleep(0.1); continue

            self.log.info("Transition to formation complete."); return

    async def follow_loop(self):
        """The core loop for following the leader."""
        self.log.info("Now in FOLLOW mode.")
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            # --- Highest Priority Checks ---
            if (time.time() - self.last_leader_message_time > CONNECTION_TIMEOUT_S):
                self.log.warning("Connection lost! Holding."); await self.drone.action.hold(); return
            
            if self.my_battery and self.my_battery.remaining_percent < CRITICAL_BATTERY_PERCENT:
                self.log.critical(f"CRITICAL BATTERY! Landing now."); await self.smart_land(); return

            leader_alt = self.leader_data.get("relative_altitude_m", 1.0)
            if leader_alt < LAND_TRIGGER_ALT_M:
                self.log.info("Leader is landing. Exiting follow loop."); return

            # --- Secondary Safety Checks ---
            if self.leader_data.get("num_satellites", 0) < LEADER_MIN_SATELLITES:
                self.log.warning(f"Leader GPS poor! Holding."); await self.drone.action.hold(); return

            current_pos = await self.drone.telemetry.position().__anext__()
            if self.calculate_distance_to_leader(current_pos) < MINIMUM_SAFE_DISTANCE:
                self.log.warning("Safety bubble breached! Evasive maneuver.");
                vec_n = (current_pos.latitude_deg - self.leader_data["latitude_deg"]) * 111320.0
                vec_e = (current_pos.longitude_deg - self.leader_data["longitude_deg"]) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vec_n, vec_e, -0.5)); await asyncio.sleep(0.1); continue

            # --- Core Following Logic ---
            leader_lat, leader_lon = self.leader_data["latitude_deg"], self.leader_data["longitude_deg"]
            lat_offset_m = OFFSET_NORTH_M / 111320.0
            lon_offset_m = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_lat)))
            target_lat, target_lon = leader_lat + lat_offset_m, leader_lon + lon_offset_m
            remaining_north = (target_lat - current_pos.latitude_deg) * 111320.0
            remaining_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            velocity_n, velocity_e = remaining_north * FLIGHT_SPEED, remaining_east * FLIGHT_SPEED
            altitude_error = leader_alt - current_pos.relative_altitude_m
            velocity_z = - (altitude_error * ALTITUDE_P_GAIN)
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, velocity_z, 0.0))
            await asyncio.sleep(0.1)
        
        self.log.warning("Pilot disengaged Offboard. Exiting follow loop.")

if __name__ == "__main__":
    follower = Follower()
    asyncio.run(follower.run())