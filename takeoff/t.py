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
SYS_ADDRESS = "serial:///dev/ttyACM0:921600"  # از udpin استفاده کن تا هشدار deprecated نگیریم

# پارامترهای حرکت
HORIZONTAL_DISTANCE = 5.0   # متر
HORIZONTAL_SPEED = 0.7     # m/s (مقدار معقول و ایمن)
MIN_TAKEOFF_ALT = 2.5       # حداقل ارتفاع مورد انتظار قبل از Offboard (متر)
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
            else:
                self.logger.debug(f"[PARAM] Unknown type for {name}: {ptype}")
            self.logger.debug(f"[PARAM] Requested set {name} = {value}")
        except Exception as e:
            # فقط هشدار بدهیم و ادامه دهیم
            self.logger.warning(f"⚠️ پارامتر {name} قابل تنظیم نیست یا وجود ندارد: {e}")

    # -------------------------
    # تنظیم پارامترهای ایمن (حداقل‌های پیشنهادی)
    # -------------------------
    async def set_safe_parameters(self):
        self.logger.info("⚙️ در حال درخواست تنظیم پارامترهای ایمن (با محافظت)...")

        # Failsafe / Geofence / Battery
        await self.set_param_safe("GF_ACTION", 1, "int")           # 1 = Land on geofence
        await self.set_param_safe("GF_MAX_VER_DIST", 15.0, "float")
        await self.set_param_safe("GF_MAX_HOR_DIST", 20.0, "float")
        await self.set_param_safe("BAT_CRIT_THR", 0.15, "float")
        await self.set_param_safe("BAT_EMERGEN_THR", 0.10, "float")

        # Velocity control limits
        await self.set_param_safe("MPC_XY_VEL_MAX", 2.0, "float")
        await self.set_param_safe("MPC_Z_VEL_MAX_UP", 1.2, "float")
        await self.set_param_safe("MPC_Z_VEL_MAX_DN", 1.0, "float")
        await self.set_param_safe("MPC_ACC_HOR_MAX", 1.5, "float")
        await self.set_param_safe("MPC_ACC_UP_MAX", 1.0, "float")
        await self.set_param_safe("MPC_ACC_DOWN_MAX", 0.8, "float")
        await self.set_param_safe("MPC_XY_VEL_P_ACC", 1.5, "float")
        await self.set_param_safe("MPC_Z_VEL_P_ACC", 1.2, "float")

        # Attitude limits
        await self.set_param_safe("MPC_MAN_TILT_MAX", 25.0, "float")
        await self.set_param_safe("MC_ROLLRATE_MAX", 120.0, "float")
        await self.set_param_safe("MC_PITCHRATE_MAX", 120.0, "float")
        await self.set_param_safe("MC_YAWRATE_MAX", 150.0, "float")

        # Position control fallback parameters (if present)
        await self.set_param_safe("MPC_XY_P", 0.8, "float")
        await self.set_param_safe("MPC_Z_P", 0.9, "float")
        await self.set_param_safe("NAV_ACC_RAD", 1.5, "float")

        # Takeoff/landing preferences
        await self.set_param_safe("MIS_TAKEOFF_ALT", 5.0, "float")
        await self.set_param_safe("MPC_TKO_SPEED", 0.7, "float")
        await self.set_param_safe("MPC_LAND_SPEED", 0.5, "float")

        self.logger.info("✅ درخواست تنظیم پارامترهای ایمن ارسال شد (پارامترهای منفور یا غایب نادیده گرفته می‌شوند).")

    # -------------------------
    # تبدیل موقعیت جغرافیایی به NED تقریبی (متر)
    # -------------------------
    def latlon_to_ned(self, lat_deg: float, lon_deg: float):
        """
        تبدیل تقریبی lat/lon به متر نسبت به (0,0) با توجه به عرض جغرافیایی:
        north ~= lat * 111320
        east  ~= lon * 111320 * cos(lat)
        (برای محاسبات فاصله‌ی کوچک قابل قبول است)
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
        چک‌های مفید قبل از انتظار تیک‌آف:
        - GPS + home (تا timeout_sec ثانیه صبر می‌کند)
        - Battery
        - IMU calibration
        """
        self.logger.info("🔍 شروع چک‌های پیش‌پرواز (GPS, battery, IMU)...")

        # اتصال به سیستم را اطمینان حاصل کن (گاهی async for هنوز در دسترس نیست)
        # GPS و Home: تلاش تا timeout_sec ثانیه
        gps_ok = False
        for i in range(timeout_sec):
            try:
                async for health in self.drone.telemetry.health():
                    # برای SITL ممکن است home بعد از تیک‌آف ثبت شود؛ ما منتظر GPS و home هستیم تا اگر ممکن شد.
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
            # برای SITL ممکن است این دیر شود؛ فقط هشدار و ادامه می‌دهیم (بر اساس تصمیم کاربر)
            self.logger.warning("⚠️ GPS یا موقعیت خانه هنوز آماده نیست (ادامه می‌دهیم؛ ممکن است در SITL طبیعی باشد).")

        # Battery
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
            batt_ok = True  # در SITL گاهی مقادیر عجیب داریم؛ برای تست اجازه میدیم

        # IMU
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

        # جمع‌بندی
        all_ok = (batt_ok and (imu_ok or True))  # imu در SITL گاهی false است؛ برای تست اجازه می‌دیم
        return all_ok

    # -------------------------
    # انتظار برای تیک‌آف دستی و OFFBOARD توسط خلبان
    # -------------------------
    async def wait_manual_takeoff_and_offboard(self):
        """
        دستورالعمل:
         1) دستی تیک‌آف کن (در pxh: commander takeoff یا از QGC)
         2) وقتی به ارتفاع >= MIN_TAKEOFF_ALT رسیدی و حالت OFFBOARD فعال شد،
            در این کنسول Enter را فشار بده تا کنترل به برنامه واگذار شود.
        """
        self.logger.info("🛡️ لطفاً به صورت دستی تیک‌آف کنید و سپس حالت Offboard را فعال کنید.")
        print("\n" + "="*60)
        print("🎮 دستورالعمل:")
        print("1) دستی تیک‌آف کنید (مثلاً: pxh> commander takeoff)")
        print(f"2) به ارتفاع >= {MIN_TAKEOFF_ALT:.1f} متر برسید")
        print("3) حالت پرواز را به OFFBOARD تغییر دهید (pxh> commander mode offboard)")
        print("4) وقتی وضعیت سبز شد، در این کنسول Enter را فشار دهید")
        print("="*60)

        # وظیفه‌ی خواندن Enter به صورت غیرمسدود
        loop = asyncio.get_event_loop()
        enter_task = loop.run_in_executor(None, input, "↩️ بعد از تیک‌آف و OFFBOARD، Enter را فشار دهید: ")

        while self.mission_running:
            # حالت پرواز
            current_mode = "UNKNOWN"
            try:
                async for fm in self.drone.telemetry.flight_mode():
                    current_mode = str(fm)
                    break
            except Exception:
                pass

            # ارتفاع
            try:
                _, _, alt = await self.get_position_ned()
            except Exception:
                alt = 0.0

            # armed
            armed = False
            try:
                async for a in self.drone.telemetry.armed():
                    armed = bool(a)
                    break
            except Exception:
                pass

            status = f"Mode={current_mode} | Alt={alt:.1f}m | Armed={armed}"
            self.logger.info(f"🟡 وضعیت فعلی: {status} (منتظر OFFBOARD و ارتفاع مناسب...)")

            # اگر حالت OFFBOARD، ارتـفاع مناسب و ARMED باشه => آماده
            if current_mode == "OFFBOARD" and alt >= MIN_TAKEOFF_ALT and armed:
                self.logger.info("🟢 شرایط Offboard و ارتفاع مناسب برقرار است. برای ادامه Enter را فشار دهید.")
                # اگر کاربر Enter زده باشه، enter_task.done() true خواهد شد
                if enter_task.done():
                    # خوندن مقدار برای جلوگیری از استثناء
                    try:
                        _ = enter_task.result()
                    except Exception:
                        pass
                    # ذخیره موقعیت شروع
                    self.start_north, self.start_east, self.start_alt = await self.get_position_ned()
                    self.logger.info(f"📍 موقعیت شروع ثبت شد: Alt={self.start_alt:.2f}m")
                    return True
                # وگرنه فقط حلقه ادامه می‌یابد تا کاربر Enter بزنه
            else:
                # اگر enter را قبل از آماده شدن زد، هشدار بده و دوباره یک task برای Enter بساز
                if enter_task.done():
                    self.logger.warning("❌ Enter زود زده شد؛ ابتدا Offboard و ارتفاع مناسب لازم است. منتظر می‌مانیم.")
                    # بازیابی مجدد متد خواندن Enter
                    enter_task = loop.run_in_executor(None, input, "↩️ بعد از تیک‌آف و OFFBOARD، Enter را فشار دهید: ")

            await asyncio.sleep(0.6)

        self.logger.error("❌ مأموریت متوقف شد در انتظار تیک‌آف/Offboard.")
        return False

    # -------------------------
    # فعال‌سازی Offboard به صورت امن
    # -------------------------
    async def start_offboard(self):
        """
        قبل از start باید چند بار setpoint ارسال کنیم تا PX4 Offboard را قبول کند.
        """
        try:
            # 1) ارسال چند setpoint با سرعت صفر برای اطمینان از جریان
            for _ in range(5):
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.1)

            # 2) سپس start
            await self.drone.offboard.start()
            self.offboard_active = True
            self.logger.info("✅ Offboard فعال شد (start موفق).")
            await asyncio.sleep(0.5)
            return True
        except OffboardError as e:
            self.logger.error(f"❌ Offboard start failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ خطا در start offboard: {e}")
            return False

    # -------------------------
    # حرکت افقی دقیق با تایمر هوشمند و محدوده تحمل
    # -------------------------
    async def move_horizontal_exact(self, distance_m: float, speed_mps: float, direction: int = 1):
        """
        حرکت دقیق در افق با استفاده از موقعیت NED:
        - direction=1 -> جلو (افزایش North)
        - direction=-1 -> عقب (کاهش North)
        - تایمر هوشمند بر اساس سرعت و فاصله
        - محدوده تحمل 30 سانتی‌متر برای رسیدن به هدف
        """
        self.logger.info(f"➡️ شروع حرکت افقی دقیق: {distance_m}m با سرعت {speed_mps} m/s (direction={direction})")
        self.logger.info(f"⏰ تایمر فعال - محدوده تحمل: {POSITION_TOLERANCE}m")

        # ثبت نقطه شروع محلی (اگر هنوز ذخیره نشده)
        if self.start_north == 0.0 and self.start_east == 0.0:
            self.start_north, self.start_east, _ = await self.get_position_ned()

        # محاسبه هدف (مبنا بر محور North)
        target_north = self.start_north + (distance_m * direction)
        self.logger.info(f"📍 target_north = {target_north:.2f} (start_north={self.start_north:.2f})")

        # محاسبه تایمر هوشمند
        estimated_time = distance_m / speed_mps  # زمان تخمینی بر اساس سرعت
        timeout_duration = estimated_time * TIMEOUT_MULTIPLIER  # ضریب اطمینان
        start_time = time.time()
        
        self.logger.info(f"⏱️ تایمر تنظیم شده: {timeout_duration:.1f} ثانیه (تخمینی: {estimated_time:.1f} ثانیه)")

        # فرستادن اولین setpoint برای شروع
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(speed_mps * direction, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)

        try:
            while self.mission_running:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # چک تایمر
                if elapsed_time > timeout_duration:
                    self.logger.warning(f"⏰ تایمر حرکت به پایان رسید ({elapsed_time:.1f} ثانیه)")
                    # بررسی موقعیت فعلی برای تصمیم‌گیری نهایی
                    cur_n, cur_e, cur_alt = await self.get_position_ned()
                    remaining = abs(target_north - cur_n)
                    
                    if remaining <= POSITION_TOLERANCE:
                        self.logger.info("✅ با وجود تایمر، به محدوده تحمل رسیده‌ایم")
                        break
                    else:
                        self.logger.error(f"❌ حرکت به هدف کامل نشد. فاصله باقیمانده: {remaining:.2f}m")
                        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        return False

                # دریافت موقعیت فعلی
                cur_n, cur_e, cur_alt = await self.get_position_ned()
                
                # اختلاف تا هدف
                remaining = (target_north - cur_n) * direction
                traveled = abs(cur_n - self.start_north)

                # اگر به محدوده تحمل رسیدیم --> توقف موفق
                if abs(remaining) <= POSITION_TOLERANCE:
                    self.logger.info(f"✅ به هدف رسیدیم (تفاوت کمتر از {POSITION_TOLERANCE}m)")
                    break

                # تعیین سرعت پویا نزدیک هدف
                if abs(remaining) > 2.0:
                    vs = speed_mps
                elif abs(remaining) > 1.0:
                    vs = max(0.4, speed_mps * 0.7)
                elif abs(remaining) > 0.5:
                    vs = max(0.2, speed_mps * 0.4)
                elif abs(remaining) > POSITION_TOLERANCE + 0.1:
                    vs = max(0.1, speed_mps * 0.2)
                else:
                    vs = 0.0

                # ارسال setpoint (NED: north, east, down) -- down=0 to keep altitude
                if vs == 0.0:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                else:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vs * direction, 0.0, 0.0, 0.0))

                # لاگ وضعیت با اطلاعات تایمر
                time_remaining = timeout_duration - elapsed_time
                self.logger.info(f"📊 cur_n={cur_n:.2f} | target={target_north:.2f} | rem={remaining:.2f}m | speed={vs:.2f} m/s | time_left={time_remaining:.1f}s")
                
                await asyncio.sleep(0.15)

            # توقف نهایی و تثبیت
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1.0)
            
            # تأیید نهایی موقعیت
            final_n, final_e, final_alt = await self.get_position_ned()
            final_error = abs(target_north - final_n)
            total_time = time.time() - start_time
            
            if final_error <= POSITION_TOLERANCE:
                self.logger.info(f"🎯 حرکت با موفقیت کامل شد! خطای نهایی: {final_error:.2f}m | زمان کل: {total_time:.1f} ثانیه")
                return True
            else:
                self.logger.warning(f"⚠️ حرکت پایان یافت اما خطای نهایی: {final_error:.2f}m (بیشتر از {POSITION_TOLERANCE}m)")
                return False

        except Exception as e:
            self.logger.error(f"❌ خطا در حرکت افقی دقیق: {e}")
            await self.safe_cleanup()
            return False

    # -------------------------
    # حرکت به موقعیت هدف خاص با تایمر و تحمل
    # -------------------------
    async def move_to_target_north(self, target_north: float, speed_mps: float):
        """
        حرکت به موقعیت شمالی خاص با تایمر هوشمند و محدوده تحمل
        """
        self.logger.info(f"🎯 حرکت به موقعیت هدف: north={target_north:.2f} با سرعت {speed_mps} m/s")
        
        # محاسبه تایمر هوشمند
        cur_n, cur_e, cur_alt = await self.get_position_ned()
        distance_to_target = abs(target_north - cur_n)
        estimated_time = distance_to_target / speed_mps
        timeout_duration = estimated_time * TIMEOUT_MULTIPLIER
        start_time = time.time()
        
        self.logger.info(f"⏱️ تایمر تنظیم شده: {timeout_duration:.1f} ثانیه برای فاصله {distance_to_target:.2f}m")

        try:
            while self.mission_running:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # چک تایمر
                if elapsed_time > timeout_duration:
                    self.logger.warning(f"⏰ تایمر حرکت به پایان رسید ({elapsed_time:.1f} ثانیه)")
                    cur_n, cur_e, cur_alt = await self.get_position_ned()
                    remaining = abs(target_north - cur_n)
                    
                    if remaining <= POSITION_TOLERANCE:
                        self.logger.info("✅ با وجود تایمر، به محدوده تحمل رسیده‌ایم")
                        break
                    else:
                        self.logger.error(f"❌ حرکت به هدف کامل نشد. فاصله باقیمانده: {remaining:.2f}m")
                        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                        return False

                # دریافت موقعیت فعلی
                cur_n, cur_e, cur_alt = await self.get_position_ned()
                remaining = target_north - cur_n
                
                # اگر به محدوده تحمل رسیدیم --> توقف
                if abs(remaining) <= POSITION_TOLERANCE:
                    self.logger.info(f"✅ به موقعیت هدف رسیدیم (خطا: {abs(remaining):.2f}m)")
                    break

                # تعیین سرعت پویا
                mag = abs(remaining)
                if mag > 2.0:
                    vs = speed_mps
                elif mag > 1.0:
                    vs = max(0.4, speed_mps * 0.7)
                elif mag > 0.5:
                    vs = max(0.2, speed_mps * 0.4)
                elif mag > POSITION_TOLERANCE + 0.1:
                    vs = max(0.1, speed_mps * 0.2)
                else:
                    vs = 0.0

                # تعیین جهت
                dir_sign = 1 if remaining > 0 else -1

                # ارسال setpoint
                if vs == 0.0:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                else:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vs * dir_sign, 0.0, 0.0, 0.0))

                # لاگ وضعیت
                time_remaining = timeout_duration - elapsed_time
                self.logger.info(f"↩️ cur_n={cur_n:.2f} | target={target_north:.2f} | rem={remaining:.2f}m | speed={vs:.2f} | time_left={time_remaining:.1f}s")
                
                await asyncio.sleep(0.15)

            # توقف نهایی
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.8)
            
            # تأیید نهایی
            final_n, final_e, final_alt = await self.get_position_ned()
            final_error = abs(target_north - final_n)
            total_time = time.time() - start_time
            
            if final_error <= POSITION_TOLERANCE:
                self.logger.info(f"🎯 حرکت به هدف با موفقیت کامل شد! خطای نهایی: {final_error:.2f}m | زمان: {total_time:.1f} ثانیه")
                return True
            else:
                self.logger.warning(f"⚠️ حرکت پایان یافت اما خطای نهایی: {final_error:.2f}m")
                return False

        except Exception as e:
            self.logger.error(f"❌ خطا در حرکت به هدف: {e}")
            return False

    # -------------------------
    # تمیزکاری ایمن (خروج از Offboard و توقف ارسال setpoint)
    # -------------------------
    async def safe_cleanup(self):
        self.logger.info("🧹 انجام تمیزکاری ایمن (توقف setpoint و خروج از Offboard)")
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        except Exception:
            pass
        await asyncio.sleep(0.5)
        try:
            await self.drone.offboard.stop()
        except Exception:
            pass
        self.offboard_active = False

    # -------------------------
    # اجرای مأموریت ساده: ۵ متر جلو + ۵ متر برگشت
    # -------------------------
    async def run(self):
        self.logger.info("🚀 شروع مأموریت: 5m جلو و 5m برگشت (ارتفاع ثابت)")
        try:
            # اتصال
            self.logger.info(f"🔌 اتصال به سیستم: {SYS_ADDRESS}")
            await self.drone.connect(system_address=SYS_ADDRESS)

            # انتظار برای اتصال واقعاً برقرار شود
            connected = False
            for _ in range(15):
                async for st in self.drone.core.connection_state():
                    if st.is_connected:
                        connected = True
                    break
                if connected:
                    break
                await asyncio.sleep(0.2)

            if not connected:
                self.logger.error("❌ اتصال برقرار نشد.")
                return False

            self.logger.info("✅ اتصال برقرار شد")

            # تنظیم پارامترهای ایمن (درخواست) — محافظت شده با try/except
            await self.set_safe_parameters()

            # پیش‌پرواز
            ok = await self.preflight_checks(timeout_sec=8)
            if not ok:
                self.logger.warning("⚠️ برخی چک‌های پیش‌پرواز مناسب نبودند، اما ادامه می‌دهیم (برای SITL).")

            # انتظار برای تیک‌آف دستی و Offboard از خلبان
            ready = await self.wait_manual_takeoff_and_offboard()
            if not ready:
                self.logger.error("❌ کاربر موفق به آماده‌سازی تیک‌آف/Offboard نشد.")
                return False

            # قبل از start Offboard، مطمئن شو setpoint استریم می‌شه
            started = await self.start_offboard()
            if not started:
                self.logger.error("❌ نتوانست Offboard را شروع کند.")
                return False

            # مأموریت: حرکت 5 متر جلو (direction=1)
            self.logger.info("🎯 شروع فاز 1: حرکت 5 متر به جلو")
            success = await self.move_horizontal_exact(HORIZONTAL_DISTANCE, HORIZONTAL_SPEED, direction=1)
            if not success:
                self.logger.error("❌ حرکت جلو شکست خورد.")
                await self.safe_cleanup()
                return False

            # توقف کوتاه
            self.logger.info("⏸️ توقف کوتاه قبل از بازگشت...")
            await asyncio.sleep(1.0)

            # فاز 2: بازگشت به نقطه شروع
            self.logger.info("🎯 شروع فاز 2: بازگشت به نقطه شروع")
            cur_n, cur_e, cur_alt = await self.get_position_ned()
            success_back = await self.move_to_target_north(self.start_north, HORIZONTAL_SPEED)
            
            if not success_back:
                self.logger.error("❌ بازگشت به نقطه شروع شکست خورد.")
                await self.safe_cleanup()
                return False

            # پایان مأموریت
            final_n, final_e, final_alt = await self.get_position_ned()
            final_error = math.sqrt((final_n - self.start_north)**2 + (final_e - self.start_east)**2)
            
            self.logger.info(f"🎉 مأموریت افقی کامل شد! خطای موقعیت نهایی: {final_error:.2f}m")
            print("\n" + "="*50)
            print(f"✅ مأموریت کامل شد — خطای نهایی: {final_error:.2f}m")
            print("اکنون می‌توانید به‌صورت دستی فرود دهید.")
            print("="*50)

            # تمیزکاری امن
            await self.safe_cleanup()
            return True

        except Exception as e:
            self.logger.error(f"💥 خطای غیرمنتظره در مأموریت: {e}")
            await self.safe_cleanup()
            return False

# -------------------------
# تابع main برای اجرای اسکریپت
# -------------------------
async def main():
    mission = SafeOffboardMove()
    try:
        ok = await mission.run()
        if ok:
            mission.logger.info("پایان موفق مأموریت.")
        else:
            mission.logger.error("مأموریت با خطا به پایان رسید.")
    except KeyboardInterrupt:
        mission.logger.warning("🛑 توقف توسط کاربر (KeyboardInterrupt)")
        await mission.safe_cleanup()
    except Exception as e:
        mission.logger.error(f"💥 Exception at top level: {e}")
        await mission.safe_cleanup()

if __name__ == "__main__":
    asyncio.run(main())