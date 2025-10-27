#!/usr/bin/env python3
"""
Follower (Simulator-optimized)
- شنود UDP برای دریافت JSON از لیدر (asyncio datagram endpoint)
- راه‌اندازی Offboard امن: ارسال initial setpoint، ایجاد publisher با فرکانس ثابت
- smooth takeoff, follow و smooth land با درنظرگیری چارچوب NED (z = down)
- watchdog برای از دست رفتن لیدر و cleanup ایمن
"""
import asyncio
import math
import logging
import time
import json
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.telemetry import FlightMode

# ==== پیکربندی ====
CONNECTION_STRING = "udp://:14542"
LISTENING_PORT = 5005

FLIGHT_SPEED = 0.8
TAKEOFF_SPEED = 0.5
LANDING_SPEED = 0.3
OFFSET_NORTH_M = -5.0
OFFSET_EAST_M = 0.0
SAFETY_ALTITUDE_BUFFER = 2.0

POSITION_P_GAIN = 0.2
ALTITUDE_P_GAIN = 0.3
DEADZONE_RADIUS = 0.8

OFFBOARD_PUBLISH_HZ = 20  # publisher frequency (setpoint send)
LEADER_TIMEOUT_S = 2.0    # اگر بیشتر از این بدون پیام بود، lost

# ==== لاگینگ (تنها یک‌بار) ====
logging.basicConfig(level=logging.INFO, format="%(asctime)s [FOLLOWER] %(message)s")
log = logging.getLogger("follower")


class LeaderProtocol(asyncio.DatagramProtocol):
    """Datagram protocol to receive leader telemetry (non-blocking, efficient)"""
    def __init__(self, follower):
        self.follower = follower

    def datagram_received(self, data: bytes, addr):
        try:
            payload = json.loads(data.decode("utf-8"))
            # payload expected: {"seq":..., "timestamp":..., "telemetry": {...}}
            telemetry = payload.get("telemetry", {})
            # assign atomically-ish
            self.follower.leader_data = telemetry
            self.follower.last_leader_message_time = time.time()
            self.follower.last_leader_seq = int(payload.get("seq", 0))
            self.follower.last_leader_timestamp = float(payload.get("timestamp", 0.0))
            log.debug(f"Received leader seq={self.follower.last_leader_seq} alt={telemetry.get('relative_altitude_m')}")
        except Exception as e:
            log.error(f"❌ UDP parse error: {e}")


class Follower:
    def __init__(self):
        self.drone = System()
        self.leader_data = {}
        self.last_leader_message_time = 0.0
        self.last_leader_seq = 0
        self.last_leader_timestamp = 0.0

        self.is_following = False
        self.last_velocity_command = (0.0, 0.0, 0.0)

        self._offboard_task = None
        self._udp_transport = None
        self._tasks = []

        self._stop = False

    async def run(self):
        log.info("🚀 Follower starting (SIMULATOR VERSION)...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log.info("✅ System connected to simulator")
                break

        # Start asyncio UDP listener (datagram endpoint)
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: LeaderProtocol(self),
            local_addr=("0.0.0.0", LISTENING_PORT)
        )
        self._udp_transport = transport
        log.info(f"📡 Listening for leader on UDP port {LISTENING_PORT}")

        # Wait for position estimate to be ready
        await self.wait_for_position_estimate()


        try:
            # Create background tasks list (none blocking main loop)
            self._tasks.append(asyncio.create_task(self._state_machine()))
            # Wait until stop
            await asyncio.gather(*self._tasks)
        except asyncio.CancelledError:
            pass
        except KeyboardInterrupt:
            log.info("🛑 Interrupted by user")
        except Exception as e:
            log.error(f"❌ Unexpected error: {e}")
        finally:
            await self._safety_cleanup()
            if self._udp_transport:
                self._udp_transport.close()
            for t in self._tasks:
                t.cancel()
            log.info("🛑 Follower stopped")

    async def wait_for_position_estimate(self):
        """Async wait until position estimate OK (avoid run_until_complete)."""
        try:
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok:
                    log.info("✅ Position estimate OK")
                    return
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"Error while waiting for position estimate: {e}")

       

    async def _state_machine(self):
        """
        Main run loop:
        - monitor flight mode
        - when OFFBOARD requested and we have leader data, start offboard safely and run main_sequence
        """
        try:
            async for mode in self.drone.telemetry.flight_mode():
                # mode is a FlightMode enum
                if mode == FlightMode.OFFBOARD and not self.is_following:
                    if self._has_leader_data():
                        log.info("🎛️ OFFBOARD engaged - starting sequence")
                        try:
                            # start sequence that includes arming, offboard start, takeoff, follow
                            await self.start_offboard_and_follow_sequence()
                        except Exception as e:
                            log.error(f"❌ Error during follow sequence: {e}")
                    else:
                        log.warning("📡 OFFBOARD but no leader data yet - waiting")
                await asyncio.sleep(0.05)
                if self._stop:
                    break
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"❌ State machine error: {e}")

    async def start_offboard_and_follow_sequence(self):
        """
        Sequence:
        1) arm
        2) send initial setpoint
        3) start offboard
        4) start offboard_publisher task
        5) smooth takeoff to safe altitude
        6) smooth follow
        """
        # Ensure we have fresh leader data
        if not self._has_leader_data():
            raise RuntimeError("No leader data available at start")

        # Arm first
        log.info("🔋 Arming...")
        await self.drone.action.arm()
        await asyncio.sleep(0.3)

        # initial setpoint (zero velocities) before starting offboard
        log.info("📨 Sending initial offboard setpoint")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.15)

        # start offboard
        try:
            await self.drone.offboard.start()
            log.info("✅ Offboard started")
        except OffboardError as e:
            log.error(f"❌ Offboard start failed: {e}")
            raise

        # start publisher task to continuously send setpoints at OFFBOARD_PUBLISH_HZ
        self._offboard_task = asyncio.create_task(self._offboard_publisher())
        self._tasks.append(self._offboard_task)

        # run main sequence (takeoff + follow)
        try:
            await self.main_sequence()
        finally:
            # ensure offboard publisher cancelled & offboard stopped
            if self._offboard_task:
                self._offboard_task.cancel()
                try:
                    await self._offboard_task
                except asyncio.CancelledError:
                    pass
            try:
                await self.drone.offboard.stop()
            except Exception as e:
                log.debug(f"offboard.stop() error (ignored): {e}")

    async def _offboard_publisher(self):
        """پابلیشر که آخرین velocity را با فرکانس ثابت ارسال می‌کند (تا offboard قطع نشود)"""
        interval = 1.0 / OFFBOARD_PUBLISH_HZ
        try:
            while True:
                vx, vy, vz = self.last_velocity_command
                try:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, 0.0))
                except Exception as e:
                    log.debug(f"❌ Offboard publisher set error: {e}")
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            return

    def _has_leader_data(self):
        return (time.time() - self.last_leader_message_time) < LEADER_TIMEOUT_S and 'latitude_deg' in self.leader_data

    async def smooth_takeoff(self, target_altitude: float):
        """برخاست بسیار نرم برای شبیه‌ساز. توجه: vz برای VelocityNedYaw، مقدار 'down' است."""
        log.info(f"🛫 Smooth takeoff to {target_altitude:.1f} m")
        try:
            start_time = time.time()
            # Read current altitude
            pos = await self.drone.telemetry.position().__anext__()
            current_alt = pos.relative_altitude_m
            while current_alt < target_altitude - 0.2 and time.time() - start_time < 30:
                pos = await self.drone.telemetry.position().__anext__()
                current_alt = pos.relative_altitude_m
                remaining = target_altitude - current_alt
                if remaining > 2.0:
                    speed = TAKEOFF_SPEED
                elif remaining > 0.5:
                    speed = TAKEOFF_SPEED * 0.7
                else:
                    speed = TAKEOFF_SPEED * 0.3
                # for ascend, we must send negative vz (down = negative)
                await self._set_smooth_velocity(0.0, 0.0, -speed)
                await asyncio.sleep(0.1)
            # stop vertical motion softly
            await self._set_smooth_velocity(0.0, 0.0, 0.0)
            log.info("✅ Takeoff complete")
        except Exception as e:
            log.error(f"❌ Takeoff error: {e}")
            raise

    async def smooth_follow(self):
        """دنبال کردن نرم لیدر"""
        log.info("🎯 Starting smooth follow")
        follow_start = time.time()
        lost_counter = 0
        max_lost = int(LEADER_TIMEOUT_S / 0.1) + 1

        while self.is_following and time.time() - follow_start < 300:
            try:
                if not self._has_leader_data():
                    lost_counter += 1
                    log.warning("📡 No leader data - holding position")
                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                    if lost_counter > max_lost:
                        log.error("❌ Leader lost for too long - executing safe stop")
                        # stop offboard and hold
                        await self._execute_safe_stop()
                        return
                    await asyncio.sleep(0.1)
                    continue
                else:
                    lost_counter = 0

                # if leader landed -> land too
                if not self.leader_data.get('is_in_air', True):
                    log.info("🛬 Leader landed - following to land")
                    await self.smooth_land()
                    return

                # read our current position
                current_pos = await self.drone.telemetry.position().__anext__()
                leader_lat = float(self.leader_data["latitude_deg"])
                leader_lon = float(self.leader_data["longitude_deg"])
                leader_alt = float(self.leader_data["relative_altitude_m"])

                # convert meter offsets to degrees
                lat_offset = OFFSET_NORTH_M / 111320.0
                # use current latitude for lon conversion for slightly better accuracy
                lon_offset = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(current_pos.latitude_deg)))

                target_lat = leader_lat + lat_offset
                target_lon = leader_lon + lon_offset
                target_alt = leader_alt

                # compute errors in meters
                error_north = (target_lat - current_pos.latitude_deg) * 111320.0
                # use cos of current latitude to convert lon error
                error_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
                error_alt = target_alt - current_pos.relative_altitude_m  # positive = target higher

                vel_north, vel_east, vel_alt = self._calculate_smooth_velocity(error_north, error_east, error_alt)

                # apply the velocities (they will be published by publisher task as well)
                await self._set_smooth_velocity(vel_north, vel_east, vel_alt)
                await asyncio.sleep(0.1)
            except Exception as e:
                log.error(f"❌ Follow loop error: {e}")
                await asyncio.sleep(0.1)

    async def _set_smooth_velocity(self, vx, vy, vz):
        """Smoothing between last command and requested to avoid jerks.
        Note: vz is 'down' in VelocityNedYaw (positive = down), so caller must pass appropriate sign.
        """
        try:
            smooth_factor = 0.3
            cvx, cvy, cvz = self.last_velocity_command
            smooth_vx = cvx * (1 - smooth_factor) + vx * smooth_factor
            smooth_vy = cvy * (1 - smooth_factor) + vy * smooth_factor
            smooth_vz = cvz * (1 - smooth_factor) + vz * smooth_factor
            # Update memory command (the publisher task will actually send this periodically)
            self.last_velocity_command = (smooth_vx, smooth_vy, smooth_vz)
            # Also send immediately once to reduce latency
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(smooth_vx, smooth_vy, smooth_vz, 0.0))
            except Exception as e:
                log.debug(f"Immediate set_velocity_ned failed (will rely on publisher): {e}")
        except Exception as e:
            log.error(f"❌ Velocity set error: {e}")

    def _calculate_smooth_velocity(self, error_north, error_east, error_alt):
        """
        Compute velocity commands from errors.
        IMPORTANT: returns (vx, vy, vz) where vz is 'down' (positive down).
        So to ascend (error_alt>0) we must return negative vz.
        """
        try:
            distance_2d = math.sqrt(error_north ** 2 + error_east ** 2)
            if distance_2d < DEADZONE_RADIUS and abs(error_alt) < 0.5:
                return 0.0, 0.0, 0.0

            vel_north = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, error_north * POSITION_P_GAIN))
            vel_east = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, error_east * POSITION_P_GAIN))

            # error_alt = target - current (positive if we need to go up)
            vel_alt_cmd = error_alt * ALTITUDE_P_GAIN
            # convert up-command to 'down' (down = -up)
            vel_alt = max(-0.5, min(0.5, -vel_alt_cmd))

            return vel_north, vel_east, vel_alt
        except Exception as e:
            log.error(f"❌ Velocity calculation error: {e}")
            return 0.0, 0.0, 0.0

    async def smooth_land(self):
        """فرود نرم"""
        log.info("🛬 Starting smooth landing...")
        try:
            start_time = time.time()
            pos = await self.drone.telemetry.position().__anext__()
            current_alt = pos.relative_altitude_m
            while current_alt > 0.3 and time.time() - start_time < 60:
                pos = await self.drone.telemetry.position().__anext__()
                current_alt = pos.relative_altitude_m
                if current_alt > 3.0:
                    speed = LANDING_SPEED
                elif current_alt > 1.0:
                    speed = LANDING_SPEED * 0.6
                else:
                    speed = LANDING_SPEED * 0.2
                # landing => positive 'down' speed
                await self._set_smooth_velocity(0.0, 0.0, speed)
                await asyncio.sleep(0.1)
            await self._set_smooth_velocity(0.0, 0.0, 0.0)
            await asyncio.sleep(1.0)
            try:
                await self.drone.action.disarm()
                log.info("✅ Landing and disarm complete")
            except Exception as e:
                log.debug(f"Disarm error: {e}")
        except Exception as e:
            log.error(f"❌ Landing error: {e}")
            raise

    async def main_sequence(self):
        """سکانس اصلی: takeoff سپس follow"""
        self.is_following = True
        try:
            leader_alt = float(self.leader_data.get("relative_altitude_m", 0.0))
            takeoff_alt = max(3.0, leader_alt + SAFETY_ALTITUDE_BUFFER)
            await self.smooth_takeoff(takeoff_alt)
            await self.smooth_follow()
        except Exception as e:
            log.error(f"❌ Error in main sequence: {e}")
        finally:
            self.is_following = False

    async def _execute_safe_stop(self):
        """Stop offboard (send zero setpoint), hold and disarm if necessary"""
        try:
            # send zero velocity immediately
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        except Exception:
            pass
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        try:
            await self.drone.action.hold()
        except Exception:
            pass
        # optionally disarm if on ground - do not force disarm in air
        try:
            in_air = await self.drone.telemetry.in_air().__anext__()
            if not in_air:
                await self.drone.action.disarm()
        except Exception:
            pass

    async def _safety_cleanup(self):
        """clean up resources and put drone to safe state"""
        log.info("🛑 Performing safety cleanup...")
        try:
            in_air = await self.drone.telemetry.in_air().__anext__()
            if in_air:
                log.warning("⚠️ Drone is in air - attempting to hold and stop offboard")
                try:
                    await self._execute_safe_stop()
                except Exception as e:
                    log.error(f"❌ Error trying to safely stop: {e}")
        except Exception as e:
            log.debug(f"Error checking in_air during cleanup: {e}")

        # cancel offboard publisher if running
        if self._offboard_task:
            self._offboard_task.cancel()
            try:
                await self._offboard_task
            except asyncio.CancelledError:
                pass

        # try to stop offboard gracefully
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass

        # close UDP transport
        if self._udp_transport:
            try:
                self._udp_transport.close()
            except Exception:
                pass

        # disarm if we are on ground
        try:
            in_air = await self.drone.telemetry.in_air().__anext__()
            if not in_air:
                try:
                    await self.drone.action.disarm()
                except Exception:
                    pass
        except Exception:
            pass

if __name__ == "__main__":
    try:
        follower = Follower()
        asyncio.run(follower.run())
    except KeyboardInterrupt:
        log.info("🛑 Follower interrupted by user")
