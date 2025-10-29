#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follower01.py (Enhanced Version)
Follower که:
- به MAVSDK وصل می‌شود.
- به پورت UDP مشخص گوش می‌دهد تا تله‌متری رهبر را دریافت کند.
- پس از ورود به OFFBOARD، تیکاف نرم و دنبال کردن فرمیشن را انجام می‌دهد.
- شامل منطق فرود هوشمند و چک‌های ایمنی مداوم برای کنترل خلبان است.
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
CONNECTION_STRING = "udp://:14542"  # آدرس اتصال به PX4 این پهپاد
LISTENING_PORT = 5005               # پورتی که به داده‌های رهبر گوش می‌دهد
FOLLOWER_ID = f"follower-{uuid.uuid4().hex[:6]}"
OFFBOARD_PUBLISH_HZ = 20
SAFETY_ALT_BUFFER = 2.0

# -- پارامترهای فرود هوشمند --
LANDING_DESCEND_SPEED = 0.5
LANDING_FLARE_SPEED = 0.2
LANDING_FLARE_ALTITUDE = 1.5

# -- پارامترهای کنترل فرمیشن --
FLIGHT_SPEED = 0.8
POSITION_P_GAIN = 0.2
ALTITUDE_P_GAIN = 0.3
DEADZONE_RADIUS = 0.8
OFFSET_NORTH_M = -5.0
OFFSET_EAST_M = 0.0

# -- تنظیمات لاگ --
logging.basicConfig(level=logging.INFO, format="%(asctime)s [FOLLOWER] %(message)s")
log = logging.getLogger("follower01")
# --------------------------

class Follower:
    def __init__(self):
        self.drone = System()
        self.leader_data = {}
        self.last_leader_message_time = 0.0
        self.peers = {}
        self.last_velocity_command = (0.0, 0.0, 0.0)
        self.is_following = False
        self._stop = False

    async def _is_still_in_offboard(self):
        """چک ایمنی: بررسی می‌کند که آیا پهپاد هنوز در حالت OFFBOARD است یا نه."""
        try:
            current_mode = str(await self.drone.telemetry.flight_mode().__anext__())
            if current_mode != "OFFBOARD":
                log.warning(f"خلبان کنترل را پس گرفت! حالت فعلی: {current_mode}")
                return False
            return True
        except Exception:
            log.error("خطا در دریافت حالت پرواز. برای ایمنی عملیات متوقف می‌شود.")
            return False

    async def run(self):
        log.info("🚀 Follower starting...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log.info("✅ به کنترلر پرواز متصل شد")
                break

        # اجرای وظایف پس‌زمینه
        asyncio.create_task(self._udp_listener_task())
        await self.wait_for_position_estimate()
        asyncio.create_task(self._leader_watchdog())

        log.info("⏳ منتظر حالت OFFBOARD و دریافت داده از رهبر...")

        # حلقه اصلی: نظارت بر حالت پرواز
        try:
            async for mode in self.drone.telemetry.flight_mode():
                mode_str = str(mode)
                if mode_str == "OFFBOARD" and not self.is_following:
                    if self._has_leader_data():
                        log.info("🎛️ حالت OFFBOARD فعال شد - شروع توالی دنبال کردن")
                        asyncio.create_task(self._start_follow_sequence())
                    else:
                        log.warning("📡 در حالت OFFBOARD اما هنوز داده‌ای از رهبر دریافت نشده است")
                else:
                    # تلاش برای ارسال دستور صفر برای آماده‌سازی انتقال روان به Offboard
                    try:
                        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                    except OffboardError:
                        pass  # این خطا طبیعی است چون هنوز در حالت Offboard نیستیم
                
                await asyncio.sleep(0.1)
                if self._stop:
                    break
        finally:
            await self._cleanup()

    async def wait_for_position_estimate(self):
        """منتظر می‌ماند تا موقعیت GPS معتبر شود."""
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                log.info("✅ موقعیت GPS معتبر است")
                return

    async def _udp_listener_task(self):
        """به طور مداوم به پیام‌های UDP از رهبر گوش می‌دهد."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            sock.bind(('', LISTENING_PORT))
            log.info(f"📡 در حال گوش دادن به رهبر روی پورت UDP {LISTENING_PORT}")
            while not self._stop:
                try:
                    data, addr = sock.recvfrom(4096)
                    payload = json.loads(data.decode('utf-8'))
                    
                    if payload.get("sender_type") == "leader":
                        self.leader_data = payload.get("telemetry", {})
                        self.last_leader_message_time = time.time()
                        # لاگ کردن پیام دریافتی برای دیباگ
                        # log.info(f"RX from {addr[0]}: seq={payload.get('seq')}")
                except BlockingIOError:
                    pass
                except Exception as e:
                    log.error(f"❌ خطای UDP: {e}")
                await asyncio.sleep(0.01)

    def _has_leader_data(self):
        """چک می‌کند آیا داده تازه از رهبر موجود است یا نه."""
        return (time.time() - self.last_leader_message_time) < 2.0 and 'latitude_deg' in self.leader_data

    async def _leader_watchdog(self):
        """نظارت می‌کند که ارتباط با رهبر قطع نشده باشد."""
        while not self._stop:
            if self.is_following and not self._has_leader_data():
                log.error("ارتباط با رهبر برای مدت طولانی قطع شد! اجرای توقف ایمن.")
                await self._execute_safe_stop()
            await asyncio.sleep(0.5)

    async def _start_follow_sequence(self):
        """توالی کامل عملیات: Arm، شروع Offboard، تیکاف و دنبال کردن."""
        if self.is_following: return
        self.is_following = True
        try:
            log.info(">>> شروع توالی دنبال کردن")
            await self.drone.action.arm()
            
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.2)
            await self.drone.offboard.start()
            log.info("✅ حالت Offboard شروع شد")

            self._offboard_task = asyncio.create_task(self._offboard_publisher())

            leader_alt = float(self.leader_data.get("relative_altitude_m", 0.0) or 0.0)
            takeoff_alt = max(3.0, leader_alt + SAFETY_ALT_BUFFER)
            await self.smooth_takeoff(takeoff_alt)

            await self._formation_follow_loop()
        except Exception as e:
            log.error(f"خطا در توالی دنبال کردن: {e}")
        finally:
            self.is_following = False
            if hasattr(self, "_offboard_task"):
                self._offboard_task.cancel()
            try:
                await self.drone.offboard.stop()
            except OffboardError as e:
                log.warning(f"خطا در توقف Offboard: {e}")
            log.info("<<< توالی دنبال کردن پایان یافت")

    async def _offboard_publisher(self):
        """آخرین دستور سرعت را به طور مداوم برای پهپاد ارسال می‌کند."""
        interval = 1.0 / OFFBOARD_PUBLISH_HZ
        while True:
            vx, vy, vz = self.last_velocity_command
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vx, vy, vz, 0.0))
            except OffboardError:
                pass # اگر حالت Offboard متوقف شده باشد، این خطا طبیعی است
            await asyncio.sleep(interval)

    async def smooth_takeoff(self, target_altitude):
        log.info(f"🛫 تیکاف نرم تا ارتفاع {target_altitude:.1f} متر")
        pos = await self.drone.telemetry.position().__anext__()
        cur_alt = pos.relative_altitude_m
        while cur_alt < target_altitude - 0.2:
            if not await self._is_still_in_offboard(): return

            pos = await self.drone.telemetry.position().__anext__()
            cur_alt = pos.relative_altitude_m
            remaining = target_altitude - cur_alt
            speed = 0.5 if remaining > 2.0 else 0.3
            self.last_velocity_command = (0.0, 0.0, -speed)
            await asyncio.sleep(0.1)
        
        self.last_velocity_command = (0.0, 0.0, 0.0)
        log.info("✅ تیکاف کامل شد")

    async def _formation_follow_loop(self):
        log.info("🎯 ورود به حلقه دنبال کردن فرمیشن")
        while True:
            if not await self._is_still_in_offboard():
                await self._execute_safe_stop()
                return

            if not self._has_leader_data():
                log.warning("داده‌ای از رهبر نیست - در حال شناوری")
                self.last_velocity_command = (0.0, 0.0, 0.0)
                await asyncio.sleep(0.1)
                continue

            if not self.leader_data.get("is_in_air", True):
                log.info("رهبر فرود آمد -> در حال فرود...")
                await self.smart_land()
                return

            # محاسبه موقعیت هدف بر اساس آفست‌ها
            leader_lat = self.leader_data["latitude_deg"]
            leader_lon = self.leader_data["longitude_deg"]
            leader_alt = self.leader_data["relative_altitude_m"]

            lat_offset_deg = OFFSET_NORTH_M / 111320.0
            lon_offset_deg = OFFSET_EAST_M / (111320.0 * math.cos(math.radians(leader_lat)))
            
            target_lat = leader_lat + lat_offset_deg
            target_lon = leader_lon + lon_offset_deg
            target_alt = leader_alt

            # محاسبه خطا و دستور سرعت
            cur = await self.drone.telemetry.position().__anext__()
            err_n = (target_lat - cur.latitude_deg) * 111320.0
            err_e = (target_lon - cur.longitude_deg) * 111320.0 * math.cos(math.radians(cur.latitude_deg))
            err_alt = target_alt - cur.relative_altitude_m

            vx, vy, vz = self._calculate_velocity(err_n, err_e, err_alt)
            self.last_velocity_command = (vx, vy, vz)
            
            await asyncio.sleep(1.0 / OFFBOARD_PUBLISH_HZ)

    def _calculate_velocity(self, err_n, err_e, err_alt):
        dist_2d = math.sqrt(err_n**2 + err_e**2)
        if dist_2d < DEADZONE_RADIUS:
            vx, vy = 0.0, 0.0
        else:
            vx = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_n * POSITION_P_GAIN))
            vy = max(-FLIGHT_SPEED, min(FLIGHT_SPEED, err_e * POSITION_P_GAIN))

        vz = max(-0.5, min(0.5, -err_alt * ALTITUDE_P_GAIN))
        return vx, vy, vz

    async def smart_land(self):
        log.info("🛬 اجرای فرود هوشمند...")
        last_altitude = -1
        no_alt_change_start_time = None

        while True:
            if not await self._is_still_in_offboard():
                await self._execute_safe_stop()
                return

            current_pos = await self.drone.telemetry.position().__anext__()
            current_altitude = current_pos.relative_altitude_m
            speed = LANDING_DESCEND_SPEED if current_altitude > LANDING_FLARE_ALTITUDE else LANDING_FLARE_SPEED
            self.last_velocity_command = (0.0, 0.0, speed)

            is_landed_telemetry = await self.drone.telemetry.landed_state().__anext__() == "ON_GROUND"

            if abs(current_altitude - last_altitude) < 0.05:
                if no_alt_change_start_time is None: no_alt_change_start_time = time.time()
            else:
                no_alt_change_start_time = None
            last_altitude = current_altitude
            
            is_landed_logic = (no_alt_change_start_time is not None and time.time() - no_alt_change_start_time > 2.0)

            if is_landed_telemetry and is_landed_logic:
                log.info("✅ تماس با زمین تایید شد.")
                self.last_velocity_command = (0.0, 0.0, 0.0)
                await asyncio.sleep(1.0)
                await self.drone.action.disarm()
                return
            
            await asyncio.sleep(0.2)

    async def _execute_safe_stop(self):
        log.info("اجرای توقف ایمن: شناوری در محل")
        self.last_velocity_command = (0.0, 0.0, 0.0)
        try:
            await self.drone.action.hold()
        except Exception as e:
            log.warning(f"اجرای Hold ناموفق بود: {e}")

    async def _cleanup(self):
        log.info("پاک‌سازی منابع...")
        self._stop = True
        await self._execute_safe_stop()
        log.info("پایان کار.")

if __name__ == "__main__":
    try:
        follower = Follower()
        asyncio.run(follower.run())
    except KeyboardInterrupt:
        log.info("عملیات توسط کاربر متوقف شد")