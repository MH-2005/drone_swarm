#!/usr/bin/env python3
# follower_wifi_smooth_safe.py
#
# Description:
# The definitive SAFE and SMOOTH follower script. This version introduces
# a velocity cap and a smoothing filter to eliminate all jerky movements,
# ensuring predictable and safe flight even with sudden leader movements or
# GPS inaccuracies. It is optimized for maximum safety and gracefulness.
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

# --- SAFE & SMOOTH Flight Behavior ---
MAX_SPEED_MS = 1.0                # ABSOLUTE SPEED LIMIT in m/s
P_GAIN_HORIZONTAL = 1.0           # Proportional gain for horizontal responsiveness. Lower is gentler.
VELOCITY_SMOOTHING_FACTOR = 0.1   # Smoothing factor (0.0 to 1.0). Lower is smoother.
ALTITUDE_P_GAIN = 2.0
CONNECTION_TIMEOUT_S = 2.0
POSITION_TOLERANCE = 0.5

# --- Formation Offset ---
OFFSET_NORTH_M = -5.0 # 5 meters BEHIND the leader
OFFSET_EAST_M = 0.0   # 0 meters to the side (in-line)

# --- State Triggers & Advanced Safety ---
TAKEOFF_TRIGGER_ALT_M = 1.5
LAND_TRIGGER_ALT_M = 1.0
TRANSITION_SAFETY_ALTITUDE = 3.0
LEADER_MIN_SATELLITES = 10
MINIMUM_SAFE_DISTANCE = 3.0
CRITICAL_BATTERY_PERCENT = 0.20

class Follower:
    """The autonomous wingman with ultimate safety logic and smooth flight dynamics."""
    def __init__(self):
        self.drone = System()
        self.log = logging.getLogger(__name__)
        logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [FOLLOWER] %(message)s",
                            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()])
        
        self.leader_data = {}
        self.last_leader_message_time = 0
        self.my_battery = None
        # State variable to store the last commanded velocity for smoothing
        self.commanded_velocity = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)

    # ... (تمام توابع دیگر مانند run, _connect_and_prepare, _background_tasks, و غیره بدون تغییر باقی می‌مانند) ...
    async def run(self):
        """The main Sentry loop that triggers the follower logic."""
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
                        self.log.warning("Disarming for safety. Check GPS lock and telemetry connection.")
                        await self.drone.action.disarm()
                        self.log.info("Waiting for pilot to switch out of Offboard mode...")
                        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
                            await asyncio.sleep(1)
                        self.log.info("Pilot has switched modes. Returning to Sentry Mode.")
                        continue

                    self.log.info("Step 3: Executing main mission logic...")
                    await self.execute_offboard_logic()

                    self.log.info("Offboard task sequence finished. Releasing control.")
                    try:
                        await self.drone.offboard.stop()
                    except OffboardError as e:
                        self.log.error(f"Could not stop Offboard mode cleanly: {e}")

                    self.log.info("Mission complete. Waiting for pilot to switch out of Offboard mode.")
                    while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
                         await asyncio.sleep(1)
                    self.log.info("Pilot has switched modes. Returned to Sentry Mode.")

                else:
                    await asyncio.sleep(0.5)
        except (asyncio.CancelledError, KeyboardInterrupt):
            self.log.info("Script interrupted by user.")
        finally:
            await self._cleanup()

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
        """Listens for leader's vitals on the specified Wi-Fi port."""
        self.log.info(f"👂 Starting Wi-Fi UDP listener on {LISTENING_IP}:{LISTENING_PORT}")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            sock.bind((LISTENING_IP, LISTENING_PORT))
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    self.leader_data = json.loads(data.decode('utf-8'))
                    self.last_leader_message_time = time.time()
                    seq = self.leader_data.get('seq', -1)
                    self.log.debug(f"<- RECV packet #{seq} from {addr[0]}")
                except BlockingIOError: pass
                except Exception as e: self.log.error(f"UDP Listener Error: {e}")
                await asyncio.sleep(0.01)

    async def _battery_monitor_task(self):
        """Monitors the drone's own battery level."""
        async for battery in self.drone.telemetry.battery():
            if self.my_battery is None or abs(self.my_battery.remaining_percent - battery.remaining_percent) > 0.05:
                 self.log.info(f"🔋 Battery level is {battery.remaining_percent:.1%}")
            self.my_battery = battery

    async def controlled_takeoff(self, target_altitude):
        # این تابع بدون تغییر باقی می‌ماند
        self.log.info(f"Executing controlled takeoff to {target_altitude}m...")
        initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            altitude_progress = current_alt - initial_alt
            if altitude_progress >= target_altitude - POSITION_TOLERANCE:
                self.log.info(f"Target altitude of {target_altitude}m reached.")
                return True
            speed_factor = max(0.2, min(1.0, (target_altitude - altitude_progress) / (target_altitude * 0.8)))
            ascent_speed = 1.5 * speed_factor # Using 1.5 m/s as base speed for takeoff
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ascent_speed, 0.0))
            await asyncio.sleep(0.1)
        self.log.warning("Offboard mode disengaged during takeoff. Aborting.")
        return False

    async def smart_land(self):
        # این تابع بدون تغییر باقی می‌ماند
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
                self.log.info("✅ Ground contact confirmed by logic and telemetry.")
                return True
            await asyncio.sleep(0.2)
        self.log.warning("Offboard mode disengaged during landing. Aborting.")
        return False

    def calculate_distance_to_leader(self, current_pos):
        # این تابع بدون تغییر باقی می‌ماند
        if "latitude_deg" not in self.leader_data or not current_pos: return float('inf')
        d_north = (self.leader_data["latitude_deg"] - current_pos.latitude_deg) * 111320.0
        d_east = (self.leader_data["longitude_deg"] - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
        d_alt = self.leader_data["relative_altitude_m"] - current_pos.relative_altitude_m
        return math.sqrt(d_north**2 + d_east**2 + d_alt**2)

    async def execute_offboard_logic(self):
        # این تابع بدون تغییر باقی می‌ماند
        self.log.info("--- Offboard Logic Activated ---")
        leader_alt = self.leader_data.get("relative_altitude_m", 0)
        leader_in_air = self.leader_data.get("is_in_air", False)
        am_i_in_air = await self.drone.telemetry.in_air().__anext__()
        self.log.info(f"State assessment: Leader in air? {leader_in_air} (Alt: {leader_alt:.1f}m). Follower in air? {am_i_in_air}.")
        if leader_in_air and leader_alt > TAKEOFF_TRIGGER_ALT_M and not am_i_in_air:
            self.log.info("Decision: Leader is flying and I am on ground. Initiating takeoff.")
            await self.drone.action.arm()
            if await self.controlled_takeoff(leader_alt):
                await self.transition_to_formation_loop()
                await self.follow_loop()
        elif leader_in_air and am_i_in_air:
            self.log.info("Decision: Both drones are airborne. Transitioning to formation.")
            await self.transition_to_formation_loop()
            await self.follow_loop()
        elif am_i_in_air and not leader_in_air:
            self.log.info("Decision: I am flying but Leader is on ground. Initiating landing.")
            if await self.smart_land():
                self.log.info("Landing complete. Disarming.")
                await self.drone.action.disarm()
        else:
            self.log.info("Decision: No clear action required (both on ground or leader too low). Holding.")
            await asyncio.sleep(1)

    async def transition_to_formation_loop(self):
        # این تابع بدون تغییر باقی می‌ماند
        self.log.info("--- Starting Safe Transition to Formation ---")
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            if "relative_altitude_m" not in self.leader_data:
                self.log.warning("Waiting for leader data to start transition..."); await asyncio.sleep(0.5); continue
            leader_pos = self.leader_data
            current_pos = await self.drone.telemetry.position().__anext__()
            safe_altitude = leader_pos["relative_altitude_m"] + TRANSITION_SAFETY_ALTITUDE
            altitude_error = safe_altitude - current_pos.relative_altitude_m
            if abs(altitude_error) > POSITION_TOLERANCE:
                velocity_z = - (altitude_error * ALTITUDE_P_GAIN)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, velocity_z, 0.0)); await asyncio.sleep(0.1); continue
            lat_offset_m = OFFSET_NORTH_M / 111320.0
            lon_offset_m = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_pos["latitude_deg"])))
            target_lat, target_lon = leader_pos["latitude_deg"] + lat_offset_m, leader_pos["longitude_deg"] + lon_offset_m
            rem_north = (target_lat - current_pos.latitude_deg) * 111320.0
            rem_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            dist_rem = math.sqrt(rem_north**2 + rem_east**2)
            if dist_rem > POSITION_TOLERANCE:
                velocity_n, velocity_e = rem_north * P_GAIN_HORIZONTAL, rem_east * P_GAIN_HORIZONTAL
                velocity_z = - (altitude_error * ALTITUDE_P_GAIN)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, velocity_z, 0.0)); await asyncio.sleep(0.1); continue
            final_altitude_error = leader_pos["relative_altitude_m"] - current_pos.relative_altitude_m
            if abs(final_altitude_error) > POSITION_TOLERANCE:
                velocity_z = - (final_altitude_error * ALTITUDE_P_GAIN)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, velocity_z, 0.0)); await asyncio.sleep(0.1); continue
            self.log.info("✅ Transition to formation complete. Engaging follow mode.")
            return True
        self.log.warning("Offboard disengaged during transition. Aborting.")
        return False
        
    async def follow_loop(self):
        """The core loop for following the leader with the new SMOOTH and CAPPED velocity logic."""
        self.log.info(f"--- Now in FOLLOW Mode (Max Speed: {MAX_SPEED_MS} m/s) ---")
        while str(await self.drone.telemetry.flight_mode().__anext__()) == "OFFBOARD":
            current_pos = await self.drone.telemetry.position().__anext__()
            
            # --- Safety checks remain the same ---
            if (time.time() - self.last_leader_message_time > CONNECTION_TIMEOUT_S):
                self.log.critical(f"CONNECTION LOST! Holding."); await self.drone.action.hold(); return
            if self.my_battery and self.my_battery.remaining_percent < CRITICAL_BATTERY_PERCENT:
                self.log.critical(f"CRITICAL BATTERY! Landing."); await self.smart_land(); return
            leader_alt = self.leader_data.get("relative_altitude_m", 1.0)
            if leader_alt < LAND_TRIGGER_ALT_M:
                self.log.info("Leader is landing. Exiting follow loop."); return
            if self.leader_data.get("num_satellites", 0) < LEADER_MIN_SATELLITES:
                self.log.warning(f"Leader GPS poor! Holding."); await self.drone.action.hold(); return
            if self.calculate_distance_to_leader(current_pos) < MINIMUM_SAFE_DISTANCE:
                self.log.warning(f"SAFETY BUBBLE BREACHED! Evasive maneuver.");
                vec_n = (current_pos.latitude_deg - self.leader_data["latitude_deg"]) * 111320.0
                vec_e = (current_pos.longitude_deg - self.leader_data["longitude_deg"]) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vec_n/2, vec_e/2, -0.5)); await asyncio.sleep(0.1); continue

            # --- NEW: Safe and Smooth Following Logic ---
            leader_lat, leader_lon = self.leader_data["latitude_deg"], self.leader_data["longitude_deg"]
            
            # 1. Calculate the target position with offset
            lat_offset_m = OFFSET_NORTH_M / 111320.0
            lon_offset_m = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_lat)))
            target_lat, target_lon = leader_lat + lat_offset_m, leader_lon + lon_offset_m
            
            # 2. Calculate error vectors in meters
            rem_north = (target_lat - current_pos.latitude_deg) * 111320.0
            rem_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            alt_error = leader_alt - current_pos.relative_altitude_m

            # 3. Calculate the desired TARGET velocity (P-controller)
            target_vel_n = rem_north * P_GAIN_HORIZONTAL
            target_vel_e = rem_east * P_GAIN_HORIZONTAL
            target_vel_z = - (alt_error * ALTITUDE_P_GAIN)

            # 4. Capping the speed: Limit the horizontal target velocity
            horizontal_speed = math.sqrt(target_vel_n**2 + target_vel_e**2)
            if horizontal_speed > MAX_SPEED_MS:
                scale_factor = MAX_SPEED_MS / horizontal_speed
                target_vel_n *= scale_factor
                target_vel_e *= scale_factor
                self.log.debug(f"Speed limited from {horizontal_speed:.1f} to {MAX_SPEED_MS:.1f} m/s")

            # 5. Smoothing the movement: Apply a low-pass filter for jerk-free flight
            smooth = VELOCITY_SMOOTHING_FACTOR
            cmd_vel_n = (1 - smooth) * self.commanded_velocity.north_m_s + smooth * target_vel_n
            cmd_vel_e = (1 - smooth) * self.commanded_velocity.east_m_s + smooth * target_vel_e
            cmd_vel_z = (1 - smooth) * self.commanded_velocity.down_m_s + smooth * target_vel_z
            
            # 6. Update the stored velocity and send the final command
            self.commanded_velocity = VelocityNedYaw(cmd_vel_n, cmd_vel_e, cmd_vel_z, 0.0)
            await self.drone.offboard.set_velocity_ned(self.commanded_velocity)
            
            await asyncio.sleep(0.05) # Loop runs at 20Hz for responsiveness
        
        self.log.warning("Pilot disengaged Offboard mode. Exiting follow loop.")
        self.commanded_velocity = VelocityNedYaw(0.0, 0.0, 0.0, 0.0) # Reset velocity on exit

    async def _cleanup(self):
        """Final safety check upon script termination."""
        self.log.info("--- Performing final safety cleanup... ---")
        try:
            is_in_air = await self.drone.telemetry.in_air().__anext__()
            if is_in_air:
                self.log.critical("DRONE IS STILL IN THE AIR ON EXIT! Commanding HOLD immediately.")
                await self.drone.action.hold()
        except Exception as e:
            self.log.error(f"Error during final cleanup: {e}")
        self.log.info("Cleanup complete. Script terminating.")

if __name__ == "__main__":
    follower = Follower()
    asyncio.run(follower.run())