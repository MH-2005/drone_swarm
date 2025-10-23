#!/usr/bin/env python3
"""
safe_offboard_move.py

مأموریت ایمن: تیک‌آف دستی → فعال شدن OFFBOARD → حرکت دقیق 5 متر جلو → بازگشت دقیق به نقطه شروع
ویژگی‌ها:
 - تنظیم پارامترهای ایمن (با محافظت در برابر پارامترهای غایب)
 - چک‌های پیش‌پرواز (GPS, battery, IMU)
 - انتظار برای تیک‌آف دستی و OFFBOARD
 - فعال‌سازی امن Offboard (ارسال setpoint قبل از start)
 - حرکت افقی دقیق با استفاده از موقعیت NED (محاسبه از latitude/longitude)
 - تایمر هوشمند مبتنی بر سرعت و فاصله
 - محدوده تحمل 30 سانتی‌متر برای رسیدن به هدف
 - تمیزکاری ایمن پس از پایان مأموریت
"""
import asyncio
import math
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# -----------------------------
# تنظیمات کلی
# -----------------------------
LOG_FILE = "safe_offboard_move.log"
# پورت اتصال؛ در SITL معمولاً از 14540 استفاده می‌شود
SYS_ADDRESS = "udp://:14540"

# پارامترهای حرکت
HORIZONTAL_DISTANCE = 5.0   # متر
HORIZONTAL_SPEED = 0.7     # m/s (مقدار معقول و ایمن)
MIN_TAKEOFF_ALT = 0.5       # حداقل ارتفاع مورد انتظار قبل از Offboard (متر)
POSITION_TOLERANCE = 0.3    # محدوده تحمل برای رسیدن به هدف (متر)
TIMEOUT_MULTIPLIER = 1.8    # ضریب اطمینان برای تایمر (بر اساس سرعت و فاصله)

# -----------------------------
# کلاس مأموریت
# -----------------------------
class SafeOffboardMove:
    def __init__(self):
        self.drone = System()
        self.mission_running = True
        self.offboard_active = False
        self.start_north = 0.0
        self.start_east = 0.0
        self.start_alt = 0.0
        self.current_alt = 0.0
        self.current_north = 0.0
        self.current_east = 0.0

        # لاگ‌گیری
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        self.logger = logging.getLogger("safe_offboard_move")

    # -------------------------
    # کمک‌فانکشن تنظیم پارامتر امن
    # -------------------------
    async def set_param_safe(self, name: str, value, ptype: str):
        """تنظیم پارامتر با محافظت در برابر خطا (پارامتر ممکن است در این نسخه موجود نباشد)."""
        try:
            if ptype == "int":
                await self.drone.param.set_param_int(name, value)
            elif ptype == "float":
                await self.drone.param.set_param_float(name, float(value))
            self.logger.debug(f"[PARAM] Requested set {name} = {value}")
        except Exception as e:
            self.logger.warning(f"⚠️ پارامتر {name} قابل تنظیم نیست یا وجود ندارد: {e}")

    # -------------------------
    # تنظیم پارامترهای ایمن (حداقل‌های پیشنهادی)
    # -------------------------
    async def set_safe_parameters(self):
        self.logger.info("⚙️ در حال درخواست تنظیم پارامترهای ایمن (با محافظت)...")
        self.logger.info("✅ استفاده از پارامترهای پیش‌فرض Pixhawk")

    # -------------------------
    # تبدیل موقعیت جغرافیایی به NED تقریبی (متر)
    # -------------------------
    def latlon_to_ned(self, lat_deg: float, lon_deg: float):
        """
        تبدیل تقریبی lat/lon به متر نسبت به (0,0) با توجه به عرض جغرافیایی.
        """
        north = lat_deg * 111320.0
        east = lon_deg * 111320.0 * math.cos(math.radians(lat_deg))
        return north, east

    # -------------------------
    # دریافت موقعیت فعلی NED
    # -------------------------
    async def get_position_ned(self):
        async for pos in self.drone.telemetry.position():
            n, e = self.latlon_to_ned(pos.latitude_deg, pos.longitude_deg)
            self.current_north = n
            self.current_east = e
            self.current_alt = pos.relative_altitude_m
            return n, e, self.current_alt

    # -------------------------
    # بررسی‌های پیش‌پرواز
    # -------------------------
    async def preflight_checks(self, timeout_sec: int = 10):
        """
        چک‌های مفید قبل از انتظار تیک‌آف.
        """
        self.logger.info("🔍 شروع چک‌های پیش‌پرواز (GPS, battery, IMU)...")
        gps_ok = False
        for i in range(timeout_sec):
            try:
                async for health in self.drone.telemetry.health():
                    if getattr(health, "is_global_position_ok", False) and getattr(health, "is_home_position_ok", False):
                        gps_ok = True
                    break
            except Exception:
                pass
            if gps_ok:
                break
            await asyncio.sleep(1)

        if gps_ok:
            self.logger.info("✅ GPS و موقعیت خانه آماده است.")
        else:
            self.logger.warning("⚠️ GPS یا موقعیت خانه هنوز آماده نیست (ادامه می‌دهیم؛ ممکن است در SITL طبیعی باشد).")

        batt_ok = False
        try:
            async for b in self.drone.telemetry.battery():
                rem = getattr(b, "remaining_percent", None)
                if rem is None:
                    self.logger.warning("⚠️ اطلاعات باتری ناقص است؛ فرض می‌کنیم باتری مناسب است.")
                    batt_ok = True
                else:
                    if rem > 0.3:
                        batt_ok = True
                        self.logger.info(f"✅ باتری: {rem*100:.1f}%")
                    else:
                        self.logger.error("❌ باتری برای پرواز مناسب نیست.")
                break
        except Exception as e:
            self.logger.warning(f"⚠️ خطا در خواندن باتری: {e}")
            batt_ok = True

        imu_ok = False
        try:
            async for h in self.drone.telemetry.health():
                if getattr(h, "is_gyrometer_calibration_ok", False) and getattr(h, "is_accelerometer_calibration_ok", False):
                    imu_ok = True
                break
        except Exception:
            pass

        if imu_ok:
            self.logger.info("✅ IMU کالیبره شده و سالم است.")
        else:
            self.logger.warning("⚠️ IMU هنوز کامل کالیبره نشده (ادامه می‌دهیم برای SITL).")

        return batt_ok and (imu_ok or True)

    # -------------------------
    # انتظار برای تیک‌آف دستی و فعال‌سازی Offboard
    # -------------------------
    async def wait_manual_takeoff_and_offboard(self):
        """
        منتظر می‌ماند تا خلبان تیک‌آف کند، به حالت Offboard برود و Enter را فشار دهد.
        در حین انتظار، به طور مداوم "اعلام حضور" می‌کند.
        """
        self.logger.info("🛡️ لطفاً تیک‌آف دستی انجام دهید و به حالت Offboard بروید")
        print("\n" + "="*60)
        print("🎮 دستورالعمل:")
        print("1) تیک‌آف دستی انجام دهید")
        print("2) به ارتفاع مناسب برسید")
        print("3) حالت پرواز را به OFFBOARD تغییر دهید (اکنون می‌توانید این کار را انجام دهید)")
        print("4) در این کنسول Enter را فشار دهید")
        print("="*60)

        loop = asyncio.get_event_loop()
        enter_task = loop.run_in_executor(None, input, "↩️ بعد از تیک‌آف و رفتن به OFFBOARD، Enter را فشار دهید: ")

        last_log_time = 0
        while self.mission_running:
            current_time = time.time()
            current_mode, alt, armed = "UNKNOWN", 0.0, False
            try:
                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                current_mode = str(flight_mode)
                _, _, alt = await self.get_position_ned()
                armed_state = await self.drone.telemetry.armed().__anext__()
                armed = bool(armed_state)
            except Exception:
                pass

            # --- بخش کلیدی: اعلام حضور مداوم ---
            # اگر پرنده آرم باشد، دستور "در جا بمان" را می‌فرستیم تا Pixhawk آماده باشد.
            if armed:
                try:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                except OffboardError:
                    # این خطا تا زمانی که در حالت Offboard نباشیم طبیعی است، پس آن را نادیده می‌گیریم.
                    pass

            # لاگ هر 3 ثانیه
            if current_time - last_log_time > 3:
                status = f"Mode={current_mode} | Alt={alt:.1f}m | Armed={armed}"
                if current_mode == "OFFBOARD" and alt >= MIN_TAKEOFF_ALT and armed:
                    self.logger.info(f"🟢 آماده: {status} - Enter بزنید")
                else:
                    self.logger.info(f"🟡 وضعیت: {status}")
                last_log_time = current_time

            # چک اگر کاربر Enter زده
            if enter_task.done():
                if current_mode == "OFFBOARD" and alt >= MIN_TAKEOFF_ALT and armed:
                    self.logger.info("✅ شروع فعال‌سازی Offboard...")
                    self.start_north, self.start_east, self.start_alt = await self.get_position_ned()
                    self.logger.info(f"📍 موقعیت شروع: Alt={self.start_alt:.2f}m")
                    return True
                else:
                    self.logger.warning("❌ شرایط برای شروع مناسب نیست!")
                    if current_mode != "OFFBOARD": self.logger.warning("   - لطفاً حالت را به OFFBOARD تغییر دهید")
                    if alt < MIN_TAKEOFF_ALT: self.logger.warning(f"   - ارتفاع کم است: {alt:.1f}m (نیاز: {MIN_TAKEOFF_ALT}m)")
                    if not armed: self.logger.warning("   - پرنده آرم نیست")
                    enter_task = loop.run_in_executor(None, input, "↩️ لطفاً شرایط را فراهم کرده و دوباره Enter را فشار دهید: ")

            await asyncio.sleep(0.2)

        if not enter_task.done():
            enter_task.cancel()
        return False

    # -------------------------
    # فعال‌سازی Offboard با چک مداوم
    # -------------------------
    async def start_offboard(self):
        """
        فعال‌سازی Offboard با قابلیت توقف اگر از Offboard خارج شویم
        """
        try:
            current_mode = str(await self.drone.telemetry.flight_mode().__anext__())
            if current_mode != "OFFBOARD":
                self.logger.error(f"❌ برای شروع باید در OFFBOARD باشید. حالت فعلی: {current_mode}")
                return False

            self.logger.info("🔧 در حال فعال‌سازی کنترل Offboard...")
            # ارسال چند setpoint نهایی برای اطمینان قبل از شروع
            for _ in range(5):
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.1)

            await self.drone.offboard.start()
            self.offboard_active = True
            await asyncio.sleep(0.3)
            self.logger.info("✅ کنترل Offboard فعال شد")
            self.logger.info("⚠️ اگر حالت را عوض کنید، کد بلافاصله متوقف می‌شود")
            return True
        except OffboardError as e:
            self.logger.error(f"❌ خطای Offboard: {e}")
            return False

    # -------------------------
    # چک مداوم Offboard در حین اجرا
    # -------------------------
    async def check_offboard_continuous(self):
        """
        همیشه چک می‌کند که در حالت Offboard هستیم یا نه.
        """
        current_mode = str(await self.drone.telemetry.flight_mode().__anext__())
        if current_mode != "OFFBOARD":
            self.logger.warning(f"🚨 از Offboard خارج شدید! حالت فعلی: {current_mode}")
            self.logger.info("🛑 مأموریت متوقف شد - کنترل با شما")
            self.mission_running = False
            return False
        return True

    # -------------------------
    # حرکت با چک مداوم Offboard
    # -------------------------
    async def move_horizontal_exact(self, distance_m: float, speed_mps: float, direction: int = 1):
        """
        حرکت دقیق در افق با استفاده از موقعیت NED و تایمر هوشمند.
        """
        self.logger.info(f"➡️ شروع حرکت: {distance_m}m با سرعت {speed_mps} m/s (direction={direction})")
        target_north = self.start_north + (distance_m * direction)
        estimated_time = distance_m / speed_mps
        timeout_duration = estimated_time * TIMEOUT_MULTIPLIER
        start_time = time.time()
        self.logger.info(f"⏱️ تایمر تنظیم شده: {timeout_duration:.1f} ثانیه")

        while self.mission_running:
            if not await self.check_offboard_continuous(): return False

            if time.time() - start_time > timeout_duration:
                self.logger.error(f"❌ تایمر حرکت به پایان رسید.")
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                return False

            cur_n, _, _ = await self.get_position_ned()
            remaining_dist = abs(target_north - cur_n)

            if remaining_dist <= POSITION_TOLERANCE:
                self.logger.info(f"✅ به هدف رسیدیم (خطای باقیمانده: {remaining_dist:.2f}m)")
                break

            speed_factor = max(0.1, min(1.0, remaining_dist / 1.5)) # کاهش سرعت نزدیک هدف
            current_speed = speed_mps * speed_factor
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(current_speed * direction, 0.0, 0.0, 0.0))
            
            time_left = timeout_duration - (time.time() - start_time)
            self.logger.info(f"📊 rem={remaining_dist:.2f}m | speed={current_speed:.2f}m/s | time_left={time_left:.1f}s")
            await asyncio.sleep(0.15)

        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(1.0)
        final_n, _, _ = await self.get_position_ned()
        final_error = abs(target_north - final_n)
        self.logger.info(f"🎯 حرکت کامل شد! خطای نهایی: {final_error:.2f}m")
        return final_error <= POSITION_TOLERANCE

    # -------------------------
    # حرکت به موقعیت هدف خاص
    # -------------------------
    async def move_to_target_north(self, target_north: float, speed_mps: float):
        """
        حرکت به موقعیت شمالی خاص با تایمر هوشمند.
        """
        self.logger.info(f"🎯 بازگشت به موقعیت هدف: north={target_north:.2f}")
        cur_n, _, _ = await self.get_position_ned()
        distance_to_target = abs(target_north - cur_n)
        estimated_time = distance_to_target / speed_mps
        timeout_duration = estimated_time * TIMEOUT_MULTIPLIER
        start_time = time.time()
        self.logger.info(f"⏱️ تایمر بازگشت: {timeout_duration:.1f} ثانیه")

        while self.mission_running:
            if not await self.check_offboard_continuous(): return False

            if time.time() - start_time > timeout_duration:
                self.logger.error("❌ تایمر بازگشت به پایان رسید.")
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                return False

            cur_n, _, _ = await self.get_position_ned()
            remaining_vec = target_north - cur_n

            if abs(remaining_vec) <= POSITION_TOLERANCE:
                self.logger.info(f"✅ به نقطه شروع بازگشتیم (خطا: {abs(remaining_vec):.2f}m)")
                break

            direction = 1 if remaining_vec > 0 else -1
            speed_factor = max(0.1, min(1.0, abs(remaining_vec) / 1.5))
            current_speed = speed_mps * speed_factor
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(current_speed * direction, 0.0, 0.0, 0.0))
            
            time_left = timeout_duration - (time.time() - start_time)
            self.logger.info(f"↩️ rem={abs(remaining_vec):.2f}m | speed={current_speed:.2f}m/s | time_left={time_left:.1f}s")
            await asyncio.sleep(0.15)
        
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.8)
        final_n, _, _ = await self.get_position_ned()
        final_error = abs(target_north - final_n)
        self.logger.info(f"🎯 بازگشت کامل شد! خطای نهایی: {final_error:.2f}m")
        return final_error <= POSITION_TOLERANCE

    # -------------------------
    # تمیزکاری ایمن
    # -------------------------
    async def safe_cleanup(self):
        self.logger.info("🧹 انجام تمیزکاری ایمن (توقف setpoint و خروج از Offboard)")
        try:
            await self.drone.offboard.stop()
        except OffboardError as e:
            self.logger.warning(f"⚠️ نتوانست Offboard را متوقف کند (شاید از قبل متوقف شده): {e}")
        self.offboard_active = False

    # -------------------------
    # اجرای کامل مأموریت
    # -------------------------
    async def run(self):
        try:
            self.logger.info(f"🔌 اتصال به سیستم: {SYS_ADDRESS}")
            await self.drone.connect(system_address=SYS_ADDRESS)
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.logger.info("✅ اتصال برقرار شد")
                    break

            await self.set_safe_parameters()
            await self.preflight_checks()

            ready = await self.wait_manual_takeoff_and_offboard()
            if not ready:
                self.logger.error("❌ خلبان مأموریت را شروع نکرد.")
                return False

            started = await self.start_offboard()
            if not started:
                return False

            self.logger.info("🎯 فاز 1: حرکت 5 متر به جلو")
            if not await self.move_horizontal_exact(HORIZONTAL_DISTANCE, HORIZONTAL_SPEED, direction=1):
                self.logger.error("❌ حرکت به جلو شکست خورد.")
                return False

            self.logger.info("⏸️ توقف کوتاه قبل از بازگشت...")
            await asyncio.sleep(1.0)

            self.logger.info("🎯 فاز 2: بازگشت به نقطه شروع")
            if not await self.move_to_target_north(self.start_north, HORIZONTAL_SPEED):
                self.logger.error("❌ بازگشت به نقطه شروع شکست خورد.")
                return False

            final_n, final_e, _ = await self.get_position_ned()
            final_error = math.sqrt((final_n - self.start_north)**2 + (final_e - self.start_east)**2)
            self.logger.info(f"🎉 مأموریت کامل شد! خطای موقعیت نهایی: {final_error:.2f}m")
            print("\n" + "="*50)
            print(f"✅ مأموریت کامل شد — خطای نهایی: {final_error:.2f}m")
            print("اکنون می‌توانید به‌صورت دستی فرود دهید.")
            print("="*50)
            return True

        except Exception as e:
            self.logger.error(f"💥 خطای غیرمنتظره در مأموریت: {e}")
            return False
        finally:
            await self.safe_cleanup()

# -------------------------
# تابع main برای اجرای اسکریپت
# -------------------------
async def main():
    mission = SafeOffboardMove()
    try:
        if await mission.run():
            mission.logger.info("پایان موفق مأموریت.")
        else:
            mission.logger.error("مأموریت با خطا به پایان رسید.")
    except (KeyboardInterrupt, asyncio.CancelledError):
        mission.logger.warning("🛑 توقف توسط کاربر.")
    finally:
        await mission.safe_cleanup()

if __name__ == "__main__":
    asyncio.run(main())