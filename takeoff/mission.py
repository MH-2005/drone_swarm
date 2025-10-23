#!/usr/bin/env python3

#
# sentry_mission_absolute.py
#
# Description:
# This corrected script uses absolute GPS coordinates for navigation, ensuring a
# precise return to the initial takeoff point. It captures the home position
# after takeoff and uses it as a fixed reference for all movements.
#

import asyncio
import math
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import Position # <--- NEW: Import Position type

# --- Configuration (No changes) ---
LOG_FILE = "sentry_mission.log"
# ... (all other configuration variables are the same)
CONNECTION_STRING = "udp://:14540"
TAKEOFF_ALTITUDE = 3.0
HORIZONTAL_DISTANCE = 5.0
FLIGHT_SPEED = 0.8
POSITION_TOLERANCE = 0.3
LANDING_DESCEND_SPEED = 0.7
LANDING_FLARE_SPEED = 0.2
LANDING_FLARE_ALTITUDE = 1.5

class SentryMission:
    def __init__(self):
        self.drone = System()
        self.mission_aborted = False
        self.home_position = None  # <--- NEW: Variable to store home position

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        self.log = logging.getLogger(__name__)

    # ... (_connect_and_prepare, _is_still_in_offboard, controlled_takeoff, smart_land are unchanged) ...
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

            speed_factor = max(0.2, min(1.0, remaining_alt / (target_altitude * 0.8)))
            ascent_speed = FLIGHT_SPEED * speed_factor
            
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
            
            speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))

            landed_state = await self.drone.telemetry.landed_state().__anext__()
            is_landed_telemetry = (str(landed_state) == "ON_GROUND")

            if abs(current_altitude - last_altitude) < 0.05:
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
    # --- RENAMED AND REWRITTEN FUNCTION ---
    async def move_to_absolute_target(self, target_pos):
        """
        Moves the drone to an absolute GPS target position.
        The target_pos is a mavsdk.telemetry.Position object.
        """
        self.log.info(f"Moving to absolute target: Lat={target_pos.latitude_deg:.5f}, Lon={target_pos.longitude_deg:.5f}")

        while True:
            if not await self._is_still_in_offboard(): return False

            current_pos = await self.drone.telemetry.position().__anext__()

            # Calculate vector to target in meters (approximate but effective)
            remaining_north = (target_pos.latitude_deg - current_pos.latitude_deg) * 111320.0
            remaining_east = (target_pos.longitude_deg - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            distance_remaining = math.sqrt(remaining_north**2 + remaining_east**2)

            if distance_remaining <= POSITION_TOLERANCE:
                self.log.info("Absolute target position reached.")
                return True

            # Proportional velocity control to ensure smooth arrival
            speed_factor = max(0.1, min(1.0, distance_remaining / 2.0))
            velocity_n = speed_factor * FLIGHT_SPEED * (remaining_north / distance_remaining)
            velocity_e = speed_factor * FLIGHT_SPEED * (remaining_east / distance_remaining)

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(velocity_n, velocity_e, 0.0, 0.0))
            self.log.info(f"Moving... Remaining: {distance_remaining:.2f}m, Vel_N: {velocity_n:.2f}m/s, Vel_E: {velocity_e:.2f}m/s")
            await asyncio.sleep(0.1)


    # --- HEAVILY MODIFIED FUNCTION ---
    async def execute_full_mission(self):
        """The complete autonomous mission sequence, using absolute coordinates."""
        self.log.info("--- OFFBOARD TRIGGERED: EXECUTING FULL MISSION ---")

        self.log.info("Arming system...")
        await self.drone.action.arm()
        await asyncio.sleep(1)

        if not await self.controlled_takeoff(TAKEOFF_ALTITUDE):
            self.log.error("Takeoff failed or was aborted.")
            return False

        # --- NEW: Capture home position right after takeoff ---
        self.log.info("Capturing home position for precise return...")
        self.home_position = await self.drone.telemetry.position().__anext__()
        if not self.home_position:
            self.log.error("Critical: Failed to capture home position. Aborting.")
            return False
        self.log.info(f"Home position captured: Lat={self.home_position.latitude_deg:.5f}, Lon={self.home_position.longitude_deg:.5f}")

        await asyncio.sleep(2)

        # --- NEW: Calculate forward destination as an absolute coordinate ---
        self.log.info(f"Phase 1: Moving forward {HORIZONTAL_DISTANCE}m.")
        lat_offset_deg = HORIZONTAL_DISTANCE / 111320.0
        forward_target_pos = Position(
            self.home_position.latitude_deg + lat_offset_deg,
            self.home_position.longitude_deg,
            self.home_position.absolute_altitude_m,
            self.home_position.relative_altitude_m
        )
        if not await self.move_to_absolute_target(forward_target_pos):
            self.log.error("Forward flight failed or was aborted.")
            return False

        await asyncio.sleep(2)

        # --- NEW: Return to the saved home position ---
        self.log.info("Phase 2: Returning to the captured home position.")
        if not await self.move_to_absolute_target(self.home_position):
            self.log.error("Return flight failed or was aborted.")
            return False

        await asyncio.sleep(2)

        if not await self.smart_land():
            self.log.error("Landing failed or was aborted.")
            return False

        self.log.info("--- MISSION COMPLETE ---")
        return True

    # ... (run function is unchanged) ...
    async def run(self):
        """The main entry point and Sentry Mode loop."""
        if not await self._connect_and_prepare():
            return

        self.log.info("--- System ready. Entering Sentry Mode. ---")
        self.log.info("Pilot can arm and fly manually. Switch to OFFBOARD to trigger the mission.")
        
        while True:
            try:
                flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
                is_armed = await self.drone.telemetry.armed().__anext__()

                if flight_mode == "OFFBOARD":
                    await self.drone.offboard.start()
                    mission_success = await self.execute_full_mission()
                    
                    if self.mission_aborted:
                        self.log.warning("Mission was aborted by pilot. Returning to Sentry Mode.")
                        self.mission_aborted = False
                        await self.drone.offboard.stop()
                    elif mission_success:
                        self.log.info("Mission successful. System will now disarm and script will terminate.")
                        await self.drone.offboard.stop()
                        await self.drone.action.disarm()
                        break
                    else:
                        self.log.error("Mission failed. System will hold and await pilot intervention.")
                        await self.drone.offboard.stop()
                        await self.drone.action.hold()
                        break

                else:
                    if is_armed:
                        try:
                            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        except OffboardError:
                            pass
                
                await asyncio.sleep(0.15)

            except (asyncio.CancelledError, KeyboardInterrupt):
                self.log.info("Sentry mode interrupted by user. Shutting down.")
                break
            except Exception as e:
                self.log.error(f"An error occurred in the Sentry loop: {e}")
                break

if __name__ == "__main__":
    mission = SentryMission()
    asyncio.run(mission.run())