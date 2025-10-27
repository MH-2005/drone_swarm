#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follower01.py
Follower که:
- به MAVSDK وصل می‌شود
- به پورت UDP مشخص گوش می‌دهد (پیش‌فرض 5005) تا تله‌متری لیـدر را دریافت کند
- پس از آماده‌شدن موقعیت و ورود به OFFBOARD، smooth takeoff و formation follow را انجام می‌دهد
- لاگ مفصل TX/RX/velocity/target/error می‌دهد تا بتوانی بررسی کنی follower دقیقاً چه کاری می‌کند
"""
import asyncio
import math
import logging
import socket
import json
import time
import uuid
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# -------- CONFIG ----------
CONNECTION_STRING = "udp://:14542"  # connection to the local PX4 instance for this follower
LISTENING_PORT = 5005                 # port to listen for leader telemetry
FOLLOWER_ID = f"follower-{uuid.uuid4().hex[:6]}"
OFFBOARD_PUBLISH_HZ = 20
SAFETY_ALT_BUFFER = 2.0

# formation control params
FLIGHT_SPEED = 0.8
POSITION_P_GAIN = 0.2
ALTITUDE_P_GAIN = 0.3
DEADZONE_RADIUS = 0.8
OFFSET_NORTH_M = -5.0
OFFSET_EAST_M = 0.0  # default lateral offset if needed

# logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [FOLLOWER] %(message)s")
log = logging.getLogger("follower01")
# --------------------------

class Follower:
    def __init__(self):
        self.drone = System()
        self.leader_data = {}
        self.last_leader_message_time = 0.0
        self.peers = {}  # id -> {ip, port, last_ts, telemetry}
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self.is_following = False
        self._stop = False

        # send socket (used for unicast if needed)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass
        self.send_target = ("255.255.255.255", LISTENING_PORT)

    async def run(self):
        log.info("🚀 Follower starting (SIMULATOR VERSION)...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log.info("✅ System connected to simulator")
                break

        # start UDP listener task
        self._udp_task = asyncio.create_task(self._udp_listener_task())

        # wait for position estimate asynchronously
        await self.wait_for_position_estimate()

        log.info("⏳ Waiting for OFFBOARD mode and leader data...")

        # start watchdog that monitors leader liveness and triggers actions
        self._watchdog_task = asyncio.create_task(self._leader_watchdog())

        # main loop: watch flight mode and start follow when OFFBOARD engaged
        try:
            async for mode in self.drone.telemetry.flight_mode():
                # mode is a FlightMode enum string (e.g., "OFFBOARD", "MANUAL", ...)
                try:
                    mode_str = str(mode)
                except Exception:
                    mode_str = repr(mode)
                if mode_str == "OFFBOARD" and not self.is_following:
                    if self._has_leader_data():
                        log.info("🎛️ OFFBOARD engaged - starting follow sequence")
                        # start follow
                        asyncio.create_task(self._start_follow_sequence())
                    else:
                        log.warning("📡 OFFBOARD but no leader data yet")
                await asyncio.sleep(0.05)
                if self._stop:
                    break
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.error(f"Main loop error: {e}")
        finally:
            await self._cleanup()

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

    async def _udp_listener_task(self):
        """Simple non-blocking recvfrom loop that updates leader/presence data."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            sock.bind(('', LISTENING_PORT))
            log.info(f"📡 Listening for leader on UDP port {LISTENING_PORT}")
            while not self._stop:
                try:
                    data, addr = sock.recvfrom(4096)
                    sender_ip, sender_port = addr
                    try:
                        payload = json.loads(data.decode('utf-8'))
                    except Exception as e:
                        log.debug(f"Malformed UDP from {sender_ip}:{sender_port} -> {e}")
                        await asyncio.sleep(0.01)
                        continue

                    sid = payload.get("id")
                    tele = payload.get("telemetry", {}) or {}
                    seq = payload.get("seq")
                    ts = payload.get("timestamp")
                    # record peer network info
                    if sid:
                        self.peers[sid] = {"ip": sender_ip, "port": sender_port, "last_ts": time.time(), "telemetry": tele}
                    # if leader payload, update leader_data
                    if payload.get("sender_type") == "leader" or tele:
                        # If leader message structure differs, adapt here
                        self.leader_data = tele
                        self.last_leader_message_time = time.time()

                    # LOG reception
                    lat = tele.get("latitude_deg")
                    lon = tele.get("longitude_deg")
                    alt = tele.get("relative_altitude_m")
                    in_air = tele.get("is_in_air")
                    log.info(f"RX from {sender_ip}:{sender_port} id={sid} seq={seq} ts={ts} lat={lat} lon={lon} alt={alt} in_air={in_air}")

                except BlockingIOError:
                    # no data currently
                    pass
                except Exception as e:
                    log.error(f"❌ UDP error: {e}")
                await asyncio.sleep(0.01)

    def _has_leader_data(self):
        return (time.time() - self.last_leader_message_time) < 2.0 and 'latitude_deg' in self.leader_data

    async def _leader_watchdog(self):
        """Monitors leader presence and logs state; if leader lost long enough, executes safe fallback."""
        lost_count = 0
        while not self._stop:
            if not self._has_leader_data():
                lost_count += 1
                if lost_count == 1:
                    log.warning("📡 No fresh leader data")
                # if leader missing for prolonged time -> safe fallback
                if lost_count > 50:  # ~0.5s with 0.01 sleep? tune as needed
                    log.error("Leader missing for extended period -> executing safe stop")
                    await self._execute_safe_stop()
                    # optionally break or keep trying election logic
                    # break
            else:
                lost_count = 0
            await asyncio.sleep(0.01)

    async def _start_follow_sequence(self):
        """Sequence: arm, initial setpoint, offboard start, takeoff, follow."""
        if self.is_following:
            return
        self.is_following = True
        try:
            log.info(">>> Starting follow sequence")
            # ensure global position ok (should already be true)
            async for h in self.drone.telemetry.health():
                if h.is_global_position_ok:
                    break

            # Arm
            log.info("🔋 Arming...")
            await self.drone.action.arm()
            await asyncio.sleep(0.3)

            # initial setpoint then offboard.start()
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.12)
            try:
                await self.drone.offboard.start()
                log.info("✅ Offboard started")
            except OffboardError as e:
                log.error(f"Offboard start failed: {e}")
                raise

            # start offboard publisher
            self._offboard_task = asyncio.create_task(self._offboard_publisher())

            # compute takeoff altitude relative to leader telemetry (if available)
            leader_alt = float(self.leader_data.get("relative_altitude_m", 0.0) or 0.0)
            takeoff_alt = max(3.0, leader_alt + SAFETY_ALT_BUFFER)
            await self.smooth_takeoff(takeoff_alt)

            # formation follow loop
            await self._formation_follow_loop()

        except Exception as e:
            log.error(f"Error in follow sequence: {e}")
        finally:
            self.is_following = False
            # stop publisher and offboard
            if hasattr(self, "_offboard_task") and self._offboard_task:
                self._offboard_task.cancel()
                try:
                    await self._offboard_task
                except asyncio.CancelledError:
                    pass
            try:
                await self.drone.offboard.stop()
            except Exception:
                pass
            log.info("<<< Follow sequence ended")

    async def _offboard_publisher(self):
        """Sends last_velocity_command at a steady rate to ensure PX4 receives it."""
        interval = 1.0 / OFFBOARD_PUBLISH_HZ
        try:
            while True:
                vx, vy, vz = self.last_velocity_command
                try:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, 0.0))
                except Exception as e:
                    log.debug(f"Offboard set_velocity_ned failed (publisher): {e}")
                await asyncio.sleep(interval)
        except asyncio.CancelledError:
            return

    async def smooth_takeoff(self, target_altitude):
        log.info(f"🛫 Smooth takeoff to {target_altitude:.1f} m")
        try:
            start = time.time()
            pos = await self.drone.telemetry.position().__anext__()
            cur_alt = pos.relative_altitude_m
            while cur_alt < target_altitude - 0.2 and time.time() - start < 30:
                pos = await self.drone.telemetry.position().__anext__()
                cur_alt = pos.relative_altitude_m
                remaining = target_altitude - cur_alt
                if remaining > 2.0:
                    speed = 0.5
                elif remaining > 0.5:
                    speed = 0.35
                else:
                    speed = 0.15
                log.info(f"Takeoff: cur_alt={cur_alt:.2f} target={target_altitude:.2f} speed={speed:.2f}")
                await self._set_smooth_velocity(0.0, 0.0, -speed)
                await asyncio.sleep(0.1)
            await self._set_smooth_velocity(0.0, 0.0, 0.0)
            log.info("✅ Takeoff complete")
        except Exception as e:
            log.error(f"Takeoff error: {e}")
            raise

    async def _formation_follow_loop(self):
        log.info("🎯 Entering formation follow loop")
        start_time = time.time()
        while True:
            try:
                if not self._has_leader_data():
                    log.warning("No leader data - holding")
                    await self._set_smooth_velocity(0.0, 0.0, 0.0)
                    await asyncio.sleep(0.1)
                    continue

                # if leader landed -> land
                if not self.leader_data.get("is_in_air", True):
                    log.info("Leader landed -> landing too")
                    await self.smooth_land()
                    return

                # build deterministic slots from known peers + self
                active = [nid for nid, info in self.peers.items() if time.time() - info['last_ts'] <= 3.0]
                if FOLLOWER_ID not in active:
                    active.append(FOLLOWER_ID)
                active = sorted(set(active))
                total = len(active)
                active.sort()
                my_index = active.index(FOLLOWER_ID)
                offset_index = my_index - (total - 1) / 2.0
                offset_east = offset_index * 3.0
                offset_north = OFFSET_NORTH_M

                # compute target pos from leader telemetry
                leader_lat = float(self.leader_data.get("latitude_deg", 0.0))
                leader_lon = float(self.leader_data.get("longitude_deg", 0.0))
                leader_alt = float(self.leader_data.get("relative_altitude_m", 0.0))

                lat_offset_deg = offset_north / 111320.0
                lon_offset_deg = offset_east / (111320.0 * math.cos(math.radians(leader_lat)))
                target_lat = leader_lat + lat_offset_deg
                target_lon = leader_lon + lon_offset_deg
                target_alt = leader_alt

                # current position
                cur = await self.drone.telemetry.position().__anext__()
                err_n = (target_lat - cur.latitude_deg) * 111320.0
                err_e = (target_lon - cur.longitude_deg) * 111320.0 * math.cos(math.radians(cur.latitude_deg))
                err_alt = target_alt - cur.relative_altitude_m

                # compute velocities
                vx, vy, vz = self._calculate_smooth_velocity(err_n, err_e, err_alt)

                # LOG target & errors
                log.info(f"Target lat={target_lat:.6f} lon={target_lon:.6f} alt={target_alt:.2f}")
                log.info(f"ERR n={err_n:.2f} m e={err_e:.2f} m alt={err_alt:.2f} m -> cmd vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")

                await self._set_smooth_velocity(vx, vy, vz)
                await asyncio.sleep(0.1)

            except Exception as e:
                log.error(f"Formation loop error: {e}")
                await asyncio.sleep(0.1)

    def _calculate_smooth_velocity(self, error_north, error_east, error_alt):
        distance_2d = math.sqrt(error_north ** 2 + error_east ** 2)
        if distance_2d < DEADZONE_RADIUS and abs(error_alt) < 0.5:
            return 0.0, 0.0, 0.0
        vel_north = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, error_north * POSITION_P_GAIN))
        vel_east = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, error_east * POSITION_P_GAIN))
        vel_alt = error_alt * ALTITUDE_P_GAIN
        # convert up to down sign: positive down for VelocityNedYaw
        vz = max(-0.5, min(0.5, -vel_alt))
        return vel_north, vel_east, vz

    async def _set_smooth_velocity(self, vx, vy, vz):
        try:
            smooth_factor = 0.3
            cvx, cvy, cvz = self.last_velocity_command
            svx = cvx * (1 - smooth_factor) + vx * smooth_factor
            svy = cvy * (1 - smooth_factor) + vy * smooth_factor
            svz = cvz * (1 - smooth_factor) + vz * smooth_factor

            # log commanded velocities (debug)
            log.debug(f"CMD-vel request vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} -> smoothed vx={svx:.2f}, vy={svy:.2f}, vz={svz:.2f}")

            self.last_velocity_command = (svx, svy, svz)
            # immediate send (publisher also running)
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(svx, svy, svz, 0.0))
            except Exception as e:
                log.debug(f"Immediate set_velocity_ned failed (will rely on publisher): {e}")

        except Exception as e:
            log.error(f"❌ Velocity set error: {e}")

    async def smooth_land(self):
        log.info("🛬 Starting smooth landing...")
        try:
            start = time.time()
            pos = await self.drone.telemetry.position().__anext__()
            current_alt = pos.relative_altitude_m
            while current_alt > 0.3 and time.time() - start < 60:
                pos = await self.drone.telemetry.position().__anext__()
                current_alt = pos.relative_altitude_m
                if current_alt > 3.0:
                    speed = 0.3
                elif current_alt > 1.0:
                    speed = 0.18
                else:
                    speed = 0.08
                log.info(f"Landing: cur_alt={current_alt:.2f} speed={speed:.2f}")
                await self._set_smooth_velocity(0.0, 0.0, speed)
                await asyncio.sleep(0.1)
            await self._set_smooth_velocity(0.0, 0.0, 0.0)
            await asyncio.sleep(1.0)
            try:
                await self.drone.action.disarm()
            except Exception:
                pass
            log.info("✅ Landing complete")
        except Exception as e:
            log.error(f"Landing error: {e}")
            raise

    async def _execute_safe_stop(self):
        log.info("Executing safe stop: zero setpoint & hold")
        try:
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

    async def _cleanup(self):
        log.info("Cleaning up follower resources...")
        self._stop = True
        try:
            await self._execute_safe_stop()
        except Exception:
            pass
        try:
            self.send_sock.close()
        except Exception:
            pass
        log.info("Cleanup finished")


if __name__ == "__main__":
    try:
        follower = Follower()
        asyncio.run(follower.run())
    except KeyboardInterrupt:
        log.info("Interrupted by user")
