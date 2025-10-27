#!/usr/bin/env python3
# follower_wifi_simple.py
#
# Description:
# A simplified and very safe version of the Follower script. It removes
# complex maneuvers and smoothing filters in favor of a direct, slow-speed
# approach. The drone simply points towards its target (5m behind the leader)
# and moves at a constant, gentle speed. All core safety layers are retained.
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
CONNECTION_STRING = "serial:///dev/ttyACM0:921600"
LISTENING_PORT = 5005
LISTENING_IP = '0.0.0.0'

# --- SIMPLE Flight Behavior ---
FOLLOW_SPEED_MS = 0.8             # The constant, gentle speed for all horizontal movements (m/s)
ALTITUDE_P_GAIN = 0.5             # Gain for vertical speed control. Lower is gentler.
POSITION_TOLERANCE = 0.7          # How close (in meters) to get to the target point before stopping.

# --- Formation Offset ---
OFFSET_NORTH_M = -5.0 # 5 meters BEHIND the leader
OFFSET_EAST_M = 0.0   # 0 meters to the side (in-line)

# --- Core Safety Configuration ---
CONNECTION_TIMEOUT_S = 2.0
LEADER_MIN_SATELLITES = 10
MINIMUM_SAFE_DISTANCE = 3.0
CRITICAL_BATTERY_PERCENT = 0.20
LAND_TRIGGER_ALT_M = 1.0

class Follower:
    """The simplified autonomous wingman with core safety and predictable movement."""
    def __init__(self):
        self.drone = System()
        self.log = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [FOLLOWER] %(message)s",
                            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()])
        
        self.leader_data = {}
        self.last_leader_message_time = 0
        self.my_battery = None

    # ... (توابع run, _connect_and_prepare, _background_tasks و ... بدون تغییر باقی می‌مانند) ...
    async def run(self):
        self.log.info("--- Follower Drone Initializing ---")
        await self._connect_and_prepare()
        await self._background_tasks()
        self.log.info("--- Follower ready. Entering Sentry Mode. ---")
        self.log.info("System is now passive. Fly the Leader drone.")
        self.log.warning("To begin following, switch this drone to OFFBOARD mode.")
        try:
            while True:
                flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
                if flight_mode == "OFFBOARD":
                    self.log.critical("PILOT ENGAGED OFFBOARD MODE. Preparing to take control...")
                    self.log.info("Step 1: Streaming initial 'hold' command to flight controller...")
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    self.log.info("Step 2: Requesting to start Offboard control...")
                    try:
                        await self.drone.offboard.start()
                        self.log.info("✅ Offboard control started successfully.")
                    except OffboardError as error:
                        self.log.error(f"❌ FATAL: Could not start Offboard mode: {error}")
                        await self.drone.action.disarm()
                        self.log.info("Waiting for pilot to switch out of Offboard mode...")
                        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD": await asyncio.sleep(1)
                        self.log.info("Pilot has switched modes. Returning to Sentry Mode.")
                        continue
                    self.log.info("Step 3: Executing main mission logic...")
                    await self.execute_simple_logic()
                    self.log.info("Offboard task sequence finished. Releasing control.")
                    try: await self.drone.offboard.stop()
                    except OffboardError as e: self.log.error(f"Could not stop Offboard mode cleanly: {e}")
                    self.log.info("Mission complete. Waiting for pilot to switch out of Offboard mode.")
                    while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD": await asyncio.sleep(1)
                    self.log.info("Pilot has switched modes. Returned to Sentry Mode.")
                else:
                    await asyncio.sleep(0.5)
        except (asyncio.CancelledError, KeyboardInterrupt): self.log.info("Script interrupted by user.")
        finally: await self._cleanup()

    async def execute_simple_logic(self):
        """Contains the simplified decision tree."""
        self.log.info("--- Simple Offboard Logic Activated ---")
        leader_in_air = self.leader_data.get("is_in_air", False)
        am_i_in_air = await self.drone.telemetry.in_air().__anext__()

        if leader_in_air and not am_i_in_air:
            self.log.info("Decision: Leader is flying, I am on ground. Arming and taking off.")
            await self.drone.action.arm()
            # Simple takeoff: just go up to leader's altitude
            leader_alt = self.leader_data.get("relative_altitude_m", 3.0)
            await self.drone.action.set_takeoff_altitude(leader_alt)
            await self.drone.action.takeoff()
            await asyncio.sleep(5) # Give it time to stabilize
            await self.follow_loop_simple()
        elif leader_in_air and am_i_in_air:
            self.log.info("Decision: Both drones airborne. Starting to follow.")
            await self.follow_loop_simple()
        elif am_i_in_air and not leader_in_air:
            self.log.info("Decision: I am flying, Leader is on ground. Landing.")
            await self.drone.action.land()
        else:
            self.log.info("Decision: No clear action. Holding.")
            await asyncio.sleep(1)

    async def follow_loop_simple(self):
        """The simplified core loop for following the leader at a constant, slow speed."""
        self.log.info(f"--- Now in SIMPLE FOLLOW Mode (Speed: {FOLLOW_SPEED_MS} m/s) ---")
        
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            current_pos = await self.drone.telemetry.position().__anext__()
            
            # --- Core safety checks remain unchanged ---
            if (time.time() - self.last_leader_message_time > CONNECTION_TIMEOUT_S):
                self.log.critical("CONNECTION LOST! Holding."); await self.drone.action.hold(); return
            if self.my_battery and self.my_battery.remaining_percent < CRITICAL_BATTERY_PERCENT:
                self.log.critical("CRITICAL BATTERY! Landing."); await self.drone.action.land(); return
            if self.leader_data.get("relative_altitude_m", 1.0) < LAND_TRIGGER_ALT_M:
                self.log.info("Leader is landing. Exiting follow loop."); return
            if "latitude_deg" not in self.leader_data:
                self.log.warning("Waiting for leader data..."); await asyncio.sleep(0.5); continue
            
            # --- Simplified "Point and Move" Logic ---
            leader_lat, leader_lon, leader_alt = self.leader_data["latitude_deg"], self.leader_data["longitude_deg"], self.leader_data["relative_altitude_m"]
            
            # 1. Calculate the final target position (5m behind leader)
            lat_offset_m = OFFSET_NORTH_M / 111320.0
            lon_offset_m = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_lat)))
            target_lat, target_lon = leader_lat + lat_offset_m, leader_lon + lon_offset_m
            
            # 2. Calculate horizontal error vector in meters
            rem_north = (target_lat - current_pos.latitude_deg) * 111320.0
            rem_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            horizontal_dist_to_target = math.sqrt(rem_north**2 + rem_east**2)

            vel_n, vel_e = 0.0, 0.0
            # 3. If we are far from the target, move towards it at constant speed
            if horizontal_dist_to_target > POSITION_TOLERANCE:
                # Calculate the direction vector (normalize)
                dir_n = rem_north / horizontal_dist_to_target
                dir_e = rem_east / horizontal_dist_to_target
                
                # Set velocity to the constant follow speed in that direction
                vel_n = dir_n * FOLLOW_SPEED_MS
                vel_e = dir_e * FOLLOW_SPEED_MS
            # If we are close, horizontal speed is zero (to prevent jittering)

            # 4. Calculate vertical velocity separately to match leader's altitude
            alt_error = leader_alt - current_pos.relative_altitude_m
            vel_z = - (alt_error * ALTITUDE_P_GAIN)
            
            # 5. Send the final, simple command
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_z, 0.0))
            await asyncio.sleep(0.1) # Loop runs at 10Hz
        
        self.log.warning("Pilot disengaged Offboard mode. Exiting follow loop.")

    # --- Other functions (unchanged) ---
    async def _connect_and_prepare(self):
        self.log.info(f"Connecting to flight controller at {CONNECTION_STRING}...")
        await self.drone.connect(system_address=CONNECTION_STRING)
        async for state in self.drone.core.connection_state():
            if state.is_connected: self.log.info("✅ System connected to flight controller."); break

    async def _background_tasks(self):
        self.log.info("Starting background tasks (Wi-Fi listener, battery monitor)...")
        asyncio.create_task(self._udp_listener_task())
        asyncio.create_task(self._battery_monitor_task())

    async def _udp_listener_task(self):
        self.log.info(f"👂 Starting Wi-Fi UDP listener on {LISTENING_IP}:{LISTENING_PORT}")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False); sock.bind((LISTENING_IP, LISTENING_PORT))
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    self.leader_data = json.loads(data.decode('utf-8'))
                    self.last_leader_message_time = time.time()
                except BlockingIOError: pass
                except Exception as e: self.log.error(f"UDP Listener Error: {e}")
                await asyncio.sleep(0.01)

    async def _battery_monitor_task(self):
        async for battery in self.drone.telemetry.battery():
            if self.my_battery is None or abs(self.my_battery.remaining_percent - battery.remaining_percent) > 0.05:
                 self.log.info(f"🔋 Battery level is {battery.remaining_percent:.1%}")
            self.my_battery = battery
            
    async def _cleanup(self):
        self.log.info("--- Performing final safety cleanup... ---")
        try:
            is_in_air = await self.drone.telemetry.in_air().__anext__()
            if is_in_air:
                self.log.critical("DRONE IS STILL IN THE AIR ON EXIT! Commanding HOLD immediately.")
                await self.drone.action.hold()
        except Exception as e: self.log.error(f"Error during final cleanup: {e}")
        self.log.info("Cleanup complete. Script terminating.")

if __name__ == "__main__":
    follower = Follower()
    asyncio.run(follower.run())