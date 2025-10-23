#!/usr/bin/env python3

#
# sentry_mission.py
#
# Author: [Your Name/Team]
# Date: 2025-10-23
#
# Description:
# This script operates a drone in a "Sentry Mode," where it passively waits for a pilot
# to manually engage OFFBOARD mode. Once engaged, it executes a fully autonomous
# mission: controlled takeoff, a predefined flight path, and a smart landing with
# dual-condition ground detection. The pilot can reclaim control at any moment by
# switching out of OFFBOARD mode, which immediately aborts the mission.
#

import asyncio
import math
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# --- Configuration ---
LOG_FILE = "sentry_mission.log"
CONNECTION_STRING = "udp://:14540"
TAKEOFF_ALTITUDE = 3.0       # meters
HORIZONTAL_DISTANCE = 5.0    # meters
FLIGHT_SPEED = 0.8           # m/s
POSITION_TOLERANCE = 0.3     # meters for target arrival
LANDING_DESCEND_SPEED = 0.7  # m/s
LANDING_FLARE_SPEED = 0.2    # m/s for the final meters
LANDING_FLARE_ALTITUDE = 1.5 # meters

class SentryMission:
    """Encapsulates the entire sentry and flight mission logic."""

    def __init__(self):
        self.drone = System()
        self.mission_aborted = False

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        self.log = logging.getLogger(__name__)

    async def _connect_and_prepare(self):
        """Connects to the drone and performs pre-flight health checks."""
        self.log.info(f"Connecting to system at {CONNECTION_STRING}...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("System connected.")
                break

        self.log.info("Performing pre-flight health checks...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_gyrometer_calibration_ok and health.is_accelerometer_calibration_ok:
                self.log.info("Pre-flight checks passed.")
                return True
            else:
                self.log.warning("Waiting for system to be ready (GPS, Home, IMU)...")
                await asyncio.sleep(1)

    async def _is_still_in_offboard(self):
        """Safety check: Verifies if the drone is still in OFFBOARD mode."""
        try:
            current_mode = str(await self.drone.telemetry.flight_mode().__anext__())
            if current_mode != "OFFBOARD":
                self.log.warning(f"OFFBOARD mode disengaged by pilot! Current mode: {current_mode}.")
                self.mission_aborted = True
                return False
            return True
        except Exception:
            self.log.error("Could not retrieve flight mode. Aborting for safety.")
            self.mission_aborted = True
            return False

    async def controlled_takeoff(self, target_altitude):
        """Performs a smooth, controlled ascent to the target altitude."""
        self.log.info(f"Executing controlled takeoff to {target_altitude}m...")
        initial_alt = 0
        async for pos in self.drone.telemetry.position():
            initial_alt = pos.relative_altitude_m
            break

        while True:
            if not await self._is_still_in_offboard(): return False

            current_pos = await self.drone.telemetry.position().__anext__()
            altitude = current_pos.relative_altitude_m - initial_alt
            
            remaining_alt = target_altitude - altitude
            if remaining_alt <= POSITION_TOLERANCE:
                self.log.info("Target altitude reached.")
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                return True

            # Dynamic speed adjustment for smooth ascent and landing
            speed_factor = max(0.2, min(1.0, remaining_alt / (target_altitude * 0.8)))
            ascent_speed = FLIGHT_SPEED * speed_factor
            
            # NED coordinate system: Z is down, so negative velocity is up
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ascent_speed, 0.0))
            self.log.info(f"Ascending... Altitude: {altitude:.2f}m, Speed: {ascent_speed:.2f}m/s")
            await asyncio.sleep(0.1)

    async def smart_land(self):
        """Performs a smart landing with dual-condition ground detection."""
        self.log.info("Executing smart landing...")
        
        last_altitude = -1
        no_alt_change_start_time = None
        
        while True:
            if not await self._is_still_in_offboard(): return False
            
            current_pos = await self.drone.telemetry.position().__anext__()
            current_altitude = current_pos.relative_altitude_m
            
            # Set descent speed based on altitude (flare)
            speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))

            # --- Dual-condition Ground Detection Logic ---
            # Condition 1: MAVSDK's landed state telemetry
            landed_state = await self.drone.telemetry.landed_state().__anext__()
            is_landed_telemetry = (str(landed_state) == "ON_GROUND")

            # Condition 2: No altitude change despite downward command
            if abs(current_altitude - last_altitude) < 0.05: # Threshold for no change
                if no_alt_change_start_time is None:
                    no_alt_change_start_time = time.time()
            else:
                no_alt_change_start_time = None
            last_altitude = current_altitude
            
            is_landed_logic = (no_alt_change_start_time is not None and time.time() - no_alt_change_start_time > 2.0)

            self.log.info(f"Descending... Alt: {current_altitude:.2f}m, Speed: {speed:.2f}m/s, Telemetry: {is_landed_telemetry}, Logic: {is_landed_logic}")

            if is_landed_telemetry and is_landed_logic:
                self.log.info("Ground contact confirmed by dual conditions.")
                return True
                
            await asyncio.sleep(0.2)
            
    async def move_to_relative_target(self, north_m, east_m):
        """Moves the drone to a target relative to its starting position."""
        self.log.info(f"Moving to relative target: N={north_m}m, E={east_m}m")
        
        start_pos = await self.drone.telemetry.position().__anext__()
        
        while True:
            if not await self._is_still_in_offboard(): return False

            current_pos = await self.drone.telemetry.position().__anext__()
            traveled_north = (current_pos.latitude_deg - start_pos.latitude_deg) * 111320.0
            traveled_east = (current_pos.longitude_deg - start_pos.longitude_deg) * 111320.0 * math.cos(math.radians(start_pos.latitude_deg))
            
            remaining_north = north_m - traveled_north
            remaining_east = east_m - traveled_east
            distance_remaining = math.sqrt(remaining_north**2 + remaining_east**2)

            if distance_remaining <= POSITION_TOLERANCE:
                self.log.info("Target position reached.")
                return True
                
            # Proportional velocity control
            speed_factor = max(0.1, min(1.0, distance_remaining / 2.0))
            velocity_n = speed_factor * FLIGHT_SPEED * (remaining_north / distance_remaining)
            velocity_e = speed_factor * FLIGHT_SPEED * (remaining_east / distance_remaining)

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, 0.0, 0.0))
            self.log.info(f"Moving... Remaining: {distance_remaining:.2f}m, Vel_N: {velocity_n:.2f}m/s, Vel_E: {velocity_e:.2f}m/s")
            await asyncio.sleep(0.1)

    async def execute_full_mission(self):
        """The complete autonomous mission sequence, executed only in Offboard mode."""
        self.log.info("--- OFFBOARD TRIGGERED: EXECUTING FULL MISSION ---")
        
        # --- Arming ---
        self.log.info("Arming system...")
        await self.drone.action.arm()
        await asyncio.sleep(1)

        # --- Takeoff ---
        if not await self.controlled_takeoff(TAKEOFF_ALTITUDE):
            self.log.error("Takeoff failed or was aborted.")
            return False

        await asyncio.sleep(2) # Stabilize after takeoff

        # --- Main Flight Path ---
        self.log.info(f"Phase 1: Moving forward {HORIZONTAL_DISTANCE}m.")
        if not await self.move_to_relative_target(HORIZONTAL_DISTANCE, 0.0):
            self.log.error("Forward flight failed or was aborted.")
            return False

        await asyncio.sleep(2) # Hover at destination
        
        self.log.info("Phase 2: Returning to home position.")
        if not await self.move_to_relative_target(0.0, 0.0):
            self.log.error("Return flight failed or was aborted.")
            return False
        
        await asyncio.sleep(2) # Stabilize at home

        # --- Landing ---
        if not await self.smart_land():
            self.log.error("Landing failed or was aborted.")
            return False
        
        self.log.info("--- MISSION COMPLETE ---")
        return True

    async def run(self):
        """The main entry point and Sentry Mode loop."""
        if not await self._connect_and_prepare():
            return

        self.log.info("--- System ready. Entering Sentry Mode. ---")
        self.log.info("Pilot can arm and fly manually. Switch to OFFBOARD to trigger the mission.")
        
        # Sentry loop
        while True:
            try:
                flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
                is_armed = await self.drone.telemetry.armed().__anext__()

                if flight_mode == "OFFBOARD":
                    # --- Trigger condition met ---
                    await self.drone.offboard.start()
                    mission_success = await self.execute_full_mission()
                    
                    if self.mission_aborted:
                        self.log.warning("Mission was aborted by pilot. Returning to Sentry Mode.")
                        self.mission_aborted = False # Reset for next trigger
                        await self.drone.offboard.stop()
                        # The loop will continue, re-entering Sentry Mode
                    elif mission_success:
                        self.log.info("Mission successful. System will now disarm and script will terminate.")
                        await self.drone.offboard.stop()
                        await self.drone.action.disarm()
                        break # Exit the sentry loop and terminate script
                    else:
                        self.log.error("Mission failed. System will hold and await pilot intervention.")
                        await self.drone.offboard.stop()
                        await self.drone.action.hold()
                        break

                else:
                    # --- Announce presence for Offboard readiness ---
                    if is_armed:
                        try:
                            # This command prepares the drone to accept an OFFBOARD switch.
                            # It will raise an error if not in Offboard, which is expected.
                            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        except OffboardError:
                            # This is normal; we ignore it.
                            pass
                
                await asyncio.sleep(0.5)

            except (asyncio.CancelledError, KeyboardInterrupt):
                self.log.info("Sentry mode interrupted by user. Shutting down.")
                break
            except Exception as e:
                self.log.error(f"An error occurred in the Sentry loop: {e}")
                break

if __name__ == "__main__":
    mission = SentryMission()
    asyncio.run(mission.run())