#!/usr/bin/env python3

#
# sentry_mission_stable.py
#
# Description:
# This script operates a drone in a "Sentry Mode," awaiting manual engagement of
# OFFBOARD mode. Once triggered, it executes a precise square flight pattern.
# A key feature is the active P-controller for altitude, which dynamically
# corrects for any height loss during horizontal movements, ensuring a stable
# flight path. The pilot retains ultimate control and can abort at any time.
#

import asyncio
import math
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import Position

# --- Configuration ---
LOG_FILE = "sentry_mission.log"
CONNECTION_STRING = "udp://:14540"
TAKEOFF_ALTITUDE = 3.0
FLIGHT_SPEED = 1.0
POSITION_TOLERANCE = 0.3
LANDING_DESCEND_SPEED = 0.7
LANDING_FLARE_SPEED = 0.2
LANDING_FLARE_ALTITUDE = 1.5
SQUARE_SIDE_LENGTH = 5.0
ALTITUDE_P_GAIN = 2.0  # Proportional gain for the altitude controller

class SentryMission:
    """Encapsulates the entire sentry and flight mission logic."""

    def __init__(self):
        self.drone = System()
        self.mission_aborted = False
        self.home_position = None

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
            if all([health.is_global_position_ok, health.is_home_position_ok,
                    health.is_gyrometer_calibration_ok, health.is_accelerometer_calibration_ok]):
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
        initial_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m

        while True:
            if not await self._is_still_in_offboard(): return False
            
            current_alt = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            altitude_progress = current_alt - initial_alt
            
            remaining_alt = target_altitude - altitude_progress
            if remaining_alt <= POSITION_TOLERANCE:
                self.log.info("Target altitude reached.")
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                return True

            speed_factor = max(0.2, min(1.0, remaining_alt / (target_altitude * 0.8)))
            ascent_speed = FLIGHT_SPEED * speed_factor
            
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ascent_speed, 0.0))
            self.log.info(f"Ascending... Altitude: {altitude_progress:.2f}m, Speed: {ascent_speed:.2f}m/s")
            await asyncio.sleep(0.1)

    async def smart_land(self):
        """Performs a smart landing with dual-condition ground detection."""
        self.log.info("Executing smart landing...")
        last_altitude = -1
        no_alt_change_start_time = None

        while True:
            if not await self._is_still_in_offboard(): return False
            
            current_altitude = (await self.drone.telemetry.position().__anext__()).relative_altitude_m
            speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))

            is_landed_telemetry = str(await self.drone.telemetry.landed_state().__anext__()) == "ON_GROUND"

            if abs(current_altitude - last_altitude) < 0.05:
                if no_alt_change_start_time is None: no_alt_change_start_time = time.time()
            else:
                no_alt_change_start_time = None
            last_altitude = current_altitude
            
            is_landed_logic = (no_alt_change_start_time is not None and time.time() - no_alt_change_start_time > 2.0)

            self.log.info(f"Descending... Alt: {current_altitude:.2f}m, Telemetry: {is_landed_telemetry}, Logic: {is_landed_logic}")

            if is_landed_telemetry and is_landed_logic:
                self.log.info("Ground contact confirmed by dual conditions.")
                return True
                
            await asyncio.sleep(0.2)
            
    async def move_to_absolute_target(self, target_pos):
        """Moves the drone to an absolute GPS target while actively maintaining altitude."""
        self.log.info(f"Moving to absolute target: Lat={target_pos.latitude_deg:.5f}, Lon={target_pos.longitude_deg:.5f}")

        initial_pos = await self.drone.telemetry.position().__anext__()
        target_altitude = initial_pos.relative_altitude_m
        self.log.info(f"Altitude lock engaged. Target altitude: {target_altitude:.2f}m")

        while True:
            if not await self._is_still_in_offboard(): return False

            current_pos = await self.drone.telemetry.position().__anext__()
            
            # --- Horizontal velocity calculation (Proportional control) ---
            remaining_north = (target_pos.latitude_deg - current_pos.latitude_deg) * 111320.0
            remaining_east = (target_pos.longitude_deg - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            distance_remaining = math.sqrt(remaining_north**2 + remaining_east**2)

            if distance_remaining <= POSITION_TOLERANCE:
                self.log.info("Absolute target position reached.")
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                return True

            speed_factor = max(0.1, min(1.0, distance_remaining / 2.0))
            velocity_n = speed_factor * FLIGHT_SPEED * (remaining_north / distance_remaining)
            velocity_e = speed_factor * FLIGHT_SPEED * (remaining_east / distance_remaining)

            # --- Vertical velocity calculation (Altitude P-Controller) ---
            altitude_error = target_altitude - current_pos.relative_altitude_m
            # NED Z-axis is inverted: positive error (too low) needs negative velocity (ascend)
            velocity_z = - (altitude_error * ALTITUDE_P_GAIN)
            
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(velocity_n, velocity_e, velocity_z, 0.0)
            )
            
            self.log.info(f"Moving... Rem: {distance_remaining:.2f}m, AltError: {altitude_error:.2f}m, Vz_correct: {velocity_z:.2f}m/s")
            await asyncio.sleep(0.1)

    async def execute_full_mission(self):
        """Executes the complete square flight mission."""
        self.log.info("--- OFFBOARD TRIGGERED: EXECUTING SQUARE MISSION ---")

        self.log.info("Arming system...")
        await self.drone.action.arm()
        await asyncio.sleep(1)

        if not await self.controlled_takeoff(TAKEOFF_ALTITUDE):
            self.log.error("Takeoff failed or was aborted.")
            return False

        self.log.info("Capturing home position and calculating square corners...")
        self.home_position = await self.drone.telemetry.position().__anext__()
        p0 = self.home_position

        lat_offset_m = SQUARE_SIDE_LENGTH / 111320.0
        lon_offset_m = SQUARE_SIDE_LENGTH / (111320.0 * math.cos(math.radians(p0.latitude_deg)))

        p1 = Position(p0.latitude_deg + lat_offset_m, p0.longitude_deg, p0.absolute_altitude_m, p0.relative_altitude_m)
        p2 = Position(p1.latitude_deg, p1.longitude_deg + lon_offset_m, p0.absolute_altitude_m, p0.relative_altitude_m)
        p3 = Position(p0.latitude_deg, p2.longitude_deg, p0.absolute_altitude_m, p0.relative_altitude_m)

        self.log.info(f"Square corners calculated relative to Home (P0): Lat={p0.latitude_deg:.5f}, Lon={p0.longitude_deg:.5f}")

        corners = {"Corner 1 (Forward)": p1, "Corner 2 (Right)": p2, "Corner 3 (Back)": p3, "Home (P0)": p0}
        for name, target_pos in corners.items():
            self.log.info(f"--- Flying to {name} ---")
            if not await self.move_to_absolute_target(target_pos):
                self.log.error(f"Flight to {name} failed or was aborted.")
                return False
            self.log.info(f"Arrived at {name}. Hovering for 2 seconds.")
            await asyncio.sleep(2)

        if not await self.smart_land():
            self.log.error("Landing failed or was aborted.")
            return False

        self.log.info("--- SQUARE MISSION COMPLETE ---")
        return True

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
                        self.log.info("Pausing for 1 second to ensure system is stable before disarming...")
                        await asyncio.sleep(1)
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
                
                await asyncio.sleep(0.1)

            except (asyncio.CancelledError, KeyboardInterrupt):
                self.log.info("Sentry mode interrupted by user. Shutting down.")
                break
            except Exception as e:
                self.log.error(f"An error occurred in the Sentry loop: {e}")
                break

if __name__ == "__main__":
    mission = SentryMission()
    asyncio.run(mission.run())