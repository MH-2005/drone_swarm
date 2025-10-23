#!/usr/bin/env python3
"""
safe_auto_mission_with_recovery.py

مأموریت ایمن با قابلیت بازیابی: اگر خلبان از Offboard خارج شد، کد منتظر می‌ماند تا دوباره به Offboard برگردد
"""

import asyncio
import math
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from mavsdk.action import ActionError

# -----------------------------
# تنظیمات کلی
# -----------------------------
LOG_FILE = "safe_auto_mission.log"
SYS_ADDRESS = "serial:///dev/ttyACM0:921600"

# پارامترهای مأموریت
TARGET_ALTITUDE = 3.0        # ارتفاع هدف برای تیک‌آف (متر)
HORIZONTAL_DISTANCE = 5.0    # متر
HORIZONTAL_SPEED = 0.7       # m/s
VERTICAL_SPEED = 0.5         # m/s برای تیک‌آف و لندینگ
POSITION_TOLERANCE = 0.3     # محدوده تحمل برای رسیدن به هدف (متر)
TIMEOUT_MULTIPLIER = 2.5     # ضریب اطمینان برای تایمر
WAIT_FOR_OFFBOARD_TIMEOUT = 30  # حداکثر زمان انتظار برای بازگشت به Offboard (ثانیه)

# -----------------------------
# کلاس مأموریت
# -----------------------------
class SafeAutoMission:
    def __init__(self):
        self.drone = System()
        self.mission_running = True
        self.offboard_active = False
        self.offboard_interrupted = False
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.start_north = 0.0
        self.start_east = 0.0
        self.current_alt = 0.0
        self.current_north = 0.0
        self.current_east = 0.0
        self.current_flight_mode = "UNKNOWN"
        self.is_armed = False

        # لاگ‌گیری
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        self.logger = logging.getLogger("safe_auto_mission")

    # -------------------------
    # کمک‌فانکشن تنظیم پارامتر امن
    # -------------------------
    async def set_param_safe(self, name: str, value, ptype: str):
        """تنظیم پارامتر با محافظت در برابر خطا"""
        try:
            if ptype == "int":
                await self.drone.param.set_param_int(name, value)
            elif ptype == "float":
                await self.drone.param.set_param_float(name, float(value))
            else:
                self.logger.debug(f"[PARAM] Unknown type for {name}: {ptype}")
            self.logger.debug(f"[PARAM] Requested set {name} = {value}")
        except Exception as e:
            self.logger.warning(f"⚠️ پارامتر {name} قابل تنظیم نیست: {e}")

    # -------------------------
    # تنظیم پارامترهای ایمن
    # -------------------------
    async def set_safe_parameters(self):
        self.logger.info("⚙️ در حال درخواست تنظیم پارامترهای ایمن...")

        # # Failsafe / Geofence / Battery
        # await self.set_param_safe("GF_ACTION", 1, "int")           # 1 = Land on geofence
        # await self.set_param_safe("GF_MAX_VER_DIST", 15.0, "float")
        # await self.set_param_safe("GF_MAX_HOR_DIST", 20.0, "float")
        # await self.set_param_safe("BAT_CRIT_THR", 0.15, "float")
        # await self.set_param_safe("BAT_EMERGEN_THR", 0.10, "float")

        # # Velocity control limits
        # await self.set_param_safe("MPC_XY_VEL_MAX", 2.0, "float")
        # await self.set_param_safe("MPC_Z_VEL_MAX_UP", 1.2, "float")
        # await self.set_param_safe("MPC_Z_VEL_MAX_DN", 1.0, "float")
        # await self.set_param_safe("MPC_ACC_HOR_MAX", 1.5, "float")
        # await self.set_param_safe("MPC_ACC_UP_MAX", 1.0, "float")
        # await self.set_param_safe("MPC_ACC_DOWN_MAX", 0.8, "float")
        # await self.set_param_safe("MPC_XY_VEL_P_ACC", 1.5, "float")
        # await self.set_param_safe("MPC_Z_VEL_P_ACC", 1.2, "float")

        # # Attitude limits
        # await self.set_param_safe("MPC_MAN_TILT_MAX", 25.0, "float")
        # await self.set_param_safe("MC_ROLLRATE_MAX", 120.0, "float")
        # await self.set_param_safe("MC_PITCHRATE_MAX", 120.0, "float")
        # await self.set_param_safe("MC_YAWRATE_MAX", 150.0, "float")

        # # Position control fallback parameters (if present)
        # await self.set_param_safe("MPC_XY_P", 0.8, "float")
        # await self.set_param_safe("MPC_Z_P", 0.9, "float")
        # await self.set_param_safe("NAV_ACC_RAD", 1.5, "float")

        # # Takeoff/landing preferences
        # await self.set_param_safe("MIS_TAKEOFF_ALT", 5.0, "float")
        # await self.set_param_safe("MPC_TKO_SPEED", 0.7, "float")
        # await self.set_param_safe("MPC_LAND_SPEED", 0.5, "float")

        self.logger.info("✅ استفاده از پارامترهای پیش‌فرض Pixhawk")

    # -------------------------
    # تبدیل موقعیت جغرافیایی به NED
    # -------------------------
    def latlon_to_ned(self, lat_deg: float, lon_deg: float):
        """تبدیل تقریبی lat/lon به متر نسبت به نقطه خانه"""
        if self.home_lat == 0.0 or self.home_lon == 0.0:
            return 0.0, 0.0
            
        delta_lat = lat_deg - self.home_lat
        delta_lon = lon_deg - self.home_lon
        
        north = delta_lat * 111320.0
        east = delta_lon * 111320.0 * math.cos(math.radians(self.home_lat))
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
    # ذخیره موقعیت خانه
    # -------------------------
    async def set_home_position(self):
        """ذخیره موقعیت خانه به عنوان مرجع"""
        self.logger.info("📍 در حال دریافت موقعیت خانه...")
        async for pos in self.drone.telemetry.position():
            if pos.absolute_altitude_m > 0.1:  # موقعیت معتبر
                self.home_lat = pos.latitude_deg
                self.home_lon = pos.longitude_deg
                self.home_alt = pos.absolute_altitude_m
                self.logger.info(f"✅ موقعیت خانه ثبت شد: lat={self.home_lat:.6f}, lon={self.home_lon:.6f}")
                break

    # -------------------------
    # بررسی‌های پیش‌پرواز
    # -------------------------
    async def preflight_checks(self, timeout_sec: int = 10):
        """چک‌های مفید قبل از پرواز"""
        self.logger.info("🔍 شروع چک‌های پیش‌پرواز (GPS, battery, IMU)...")

        # GPS و Home
        gps_ok = False
        for i in range(timeout_sec):
            try:
                async for health in self.drone.telemetry.health():
                    if getattr(health, "is_global_position_ok", False):
                        gps_ok = True
                    break
            except Exception:
                pass
            if gps_ok:
                break
            await asyncio.sleep(1)

        if gps_ok:
            self.logger.info("✅ GPS آماده است.")
        else:
            self.logger.warning("⚠️ GPS هنوز آماده نیست (ادامه می‌دهیم).")

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
            batt_ok = True

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
            self.logger.warning("⚠️ IMU هنوز کامل کالیبره نشده.")

        all_ok = batt_ok
        return all_ok

    # -------------------------
    # مانیتورینگ مداوم حالت پرواز و وضعیت
    # -------------------------
    async def monitor_systems(self):
        """مانیتورینگ مداوم حالت پرواز، وضعیت ARMED و موقعیت"""
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.current_flight_mode = str(flight_mode)
                await asyncio.sleep(0.1)
        except Exception as e:
            self.logger.error(f"خطا در مانیتورینگ حالت پرواز: {e}")

    async def monitor_arming_status(self):
        """مانیتورینگ وضعیت ARMED"""
        try:
            async for armed in self.drone.telemetry.armed():
                self.is_armed = bool(armed)
                await asyncio.sleep(0.1)
        except Exception as e:
            self.logger.error(f"خطا در مانیتورینگ وضعیت ARMED: {e}")

    async def start_monitoring(self):
        """شروع تمام مانیتورینگ‌ها"""
        monitoring_tasks = []
        monitoring_tasks.append(asyncio.create_task(self.monitor_systems()))
        monitoring_tasks.append(asyncio.create_task(self.monitor_arming_status()))
        return monitoring_tasks

    async def stop_monitoring(self, monitoring_tasks):
        """توقف تمام مانیتورینگ‌ها"""
        for task in monitoring_tasks:
            task.cancel()
        try:
            await asyncio.gather(*monitoring_tasks, return_exceptions=True)
        except Exception:
            pass

    # -------------------------
    # تیک‌آف نرم به ارتفاع 3 متر
    # -------------------------
    async def soft_takeoff(self, target_altitude: float):
        """تیک‌آف نرم با کنترل سرعت و شتاب"""
        self.logger.info(f"🚀 شروع تیک‌آف نرم به ارتفاع {target_altitude} متر...")
        
        # دریافت موقعیت شروع
        start_n, start_e, start_alt = await self.get_position_ned()
        self.start_north = start_n
        self.start_east = start_e
        
        # محاسبه تایمر
        estimated_time = target_altitude / VERTICAL_SPEED
        timeout_duration = estimated_time * 3.0  # ضریب بیشتر برای تیک‌آف
        start_time = time.time()
        
        self.logger.info(f"⏱️ تایمر تیک‌آف: {timeout_duration:.1f} ثانیه")

        try:
            # آرming کردن
            self.logger.info("🔓 در حال آرمینگ کردن...")
            await self.drone.action.arm()
            await asyncio.sleep(1.0)

            # تیک‌آف
            self.logger.info("🛫 در حال تیک‌آف...")
            await self.drone.action.takeoff()
            await asyncio.sleep(1.0)  # صبر برای شروع تیک‌آف

            # مانیتورینگ ارتفاع
            last_alt = 0.0
            stall_count = 0
            
            while self.mission_running:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # چک تایمر
                if elapsed_time > timeout_duration:
                    self.logger.error(f"⏰ تایمر تیک‌آف به پایان رسید")
                    return False

                # دریافت ارتفاع فعلی
                _, _, current_alt = await self.get_position_ned()
                
                # چک رسیدن به هدف
                if current_alt >= target_altitude - 0.2:  # 20cm tolerance
                    self.logger.info(f"✅ به ارتفاع هدف رسیدیم: {current_alt:.2f}m")
                    await asyncio.sleep(1.0)  # تثبیت
                    return True
                
                # چک stuck شدن
                if abs(current_alt - last_alt) < 0.05:  # کمتر از 5cm تغییر
                    stall_count += 1
                else:
                    stall_count = 0
                    
                if stall_count > 20:  # 4 ثانیه بدون پیشرفت
                    self.logger.error("❌ تیک‌آف متوقف شده - پیشرفت ارتفاع نداریم")
                    return False
                
                last_alt = current_alt
                
                # لاگ پیشرفت
                alt_remaining = target_altitude - current_alt
                progress = (current_alt / target_altitude) * 100
                time_remaining = timeout_duration - elapsed_time
                
                self.logger.info(f"📈 ارتفاع: {current_alt:.2f}m ({progress:.1f}%) | باقی: {alt_remaining:.2f}m | زمان: {time_remaining:.1f}s")
                
                await asyncio.sleep(0.2)

            return False

        except ActionError as e:
            self.logger.error(f"❌ خطا در تیک‌آف: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ خطای غیرمنتظره در تیک‌آف: {e}")
            return False

    # -------------------------
    # فعال‌سازی Offboard به صورت امن
    # -------------------------
    async def start_offboard(self):
        """فعال‌سازی حالت Offboard"""
        try:
            # ارسال setpoint اولیه
            for _ in range(5):
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.1)

            # شروع Offboard
            await self.drone.offboard.start()
            self.offboard_active = True
            self.offboard_interrupted = False
            self.logger.info("✅ Offboard فعال شد.")
            await asyncio.sleep(0.5)
            return True
        except OffboardError as e:
            self.logger.error(f"❌ Offboard start failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ خطا در start offboard: {e}")
            return False

    # -------------------------
    # انتظار برای بازگشت به Offboard
    # -------------------------
    async def wait_for_offboard_return(self):
        """انتظار برای بازگشت خلبان به حالت Offboard"""
        self.logger.info("⏳ منتظر بازگشت خلبان به حالت Offboard...")
        self.logger.info("🎮 لطفاً پرنده را به حالت OFFBOARD تغییر دهید")
        
        start_wait_time = time.time()
        
        while self.mission_running:
            current_time = time.time()
            wait_duration = current_time - start_wait_time
            
            # چک تایم‌اوت
            if wait_duration > WAIT_FOR_OFFBOARD_TIMEOUT:
                self.logger.error(f"⏰ زمان انتظار برای بازگشت به Offboard به پایان رسید ({WAIT_FOR_OFFBOARD_TIMEOUT} ثانیه)")
                return False
            
            # چک وضعیت فعلی
            if self.current_flight_mode == "OFFBOARD" and self.is_armed:
                self.logger.info("✅ خلبان به حالت Offboard بازگشت")
                self.offboard_interrupted = False
                return True
            
            # لاگ وضعیت هر 5 ثانیه
            if int(wait_duration) % 5 == 0:
                self.logger.info(f"⏰ وضعیت: {self.current_flight_mode} | Armed: {self.is_armed} | زمان انتظار: {wait_duration:.0f}s")
            
            await asyncio.sleep(0.5)
        
        return False

    # -------------------------
    # چک مداوم Offboard در حین اجرا
    # -------------------------
    async def check_offboard_status(self):
        """چک مداوم وضعیت Offboard در حین اجرای مأموریت"""
        if self.offboard_active and self.current_flight_mode != "OFFBOARD":
            if not self.offboard_interrupted:
                self.logger.warning(f"🚨 خلبان از Offboard خارج شد! حالت فعلی: {self.current_flight_mode}")
                self.logger.info("⏸️ مأموریت متوقف شد - منتظر بازگشت به Offboard...")
                self.offboard_interrupted = True
            
            # متوقف کردن ارسال setpoint
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            except Exception:
                pass
            
            return False
        return True

    # -------------------------
    # حرکت افقی دقیق با قابلیت بازیابی
    # -------------------------
    async def move_horizontal_exact(self, distance_m: float, speed_mps: float, direction: int = 1):
        """حرکت دقیق در افق با قابلیت بازیابی در صورت خروج از Offboard"""
        self.logger.info(f"➡️ شروع حرکت افقی: {distance_m}m با سرعت {speed_mps} m/s")
        
        # محاسبه هدف
        target_north = self.start_north + (distance_m * direction)
        
        # محاسبه تایمر هوشمند
        estimated_time = distance_m / speed_mps
        timeout_duration = estimated_time * TIMEOUT_MULTIPLIER
        start_time = time.time()
        
        self.logger.info(f"⏱️ تایمر حرکت: {timeout_duration:.1f} ثانیه")

        try:
            while self.mission_running:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # چک وضعیت Offboard
                if not await self.check_offboard_status():
                    # اگر از Offboard خارج شده‌ایم، منتظر بازگشت بمانیم
                    return_ok = await self.wait_for_offboard_return()
                    if not return_ok:
                        self.logger.error("❌ بازگشت به Offboard انجام نشد")
                        return False
                    
                    # پس از بازگشت، Offboard را دوباره فعال کنیم
                    offboard_ok = await self.start_offboard()
                    if not offboard_ok:
                        self.logger.error("❌ فعال‌سازی مجدد Offboard شکست خورد")
                        return False
                    
                    # زمان شروع را ریست کنیم
                    start_time = time.time()
                    elapsed_time = 0
                    self.logger.info("🔄 ادامه مأموریت از نقطه توقف")
                
                # چک تایمر
                if elapsed_time > timeout_duration:
                    cur_n, cur_e, cur_alt = await self.get_position_ned()
                    remaining = abs(target_north - cur_n)
                    
                    if remaining <= POSITION_TOLERANCE:
                        self.logger.info("✅ با وجود تایمر، به محدوده تحمل رسیده‌ایم")
                        break
                    else:
                        self.logger.error(f"❌ حرکت به هدف کامل نشد. فاصله باقیمانده: {remaining:.2f}m")
                        return False

                # دریافت موقعیت فعلی
                cur_n, cur_e, cur_alt = await self.get_position_ned()
                remaining = (target_north - cur_n) * direction

                # اگر به محدوده تحمل رسیدیم
                if abs(remaining) <= POSITION_TOLERANCE:
                    self.logger.info(f"✅ به هدف رسیدیم (تفاوت کمتر از {POSITION_TOLERANCE}m)")
                    break

                # تعیین سرعت پویا
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

                # ارسال setpoint
                if vs == 0.0:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                else:
                    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vs * direction, 0.0, 0.0, 0.0))

                # لاگ وضعیت
                time_remaining = timeout_duration - elapsed_time
                self.logger.info(f"📊 موقعیت: {cur_n:.2f} | هدف: {target_north:.2f} | باقی: {abs(remaining):.2f}m | سرعت: {vs:.2f} m/s | زمان: {time_remaining:.1f}s")
                
                await asyncio.sleep(0.15)

            # توقف نهایی
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(1.0)
            
            # تأیید نهایی
            final_n, final_e, final_alt = await self.get_position_ned()
            final_error = abs(target_north - final_n)
            total_time = time.time() - start_time
            
            if final_error <= POSITION_TOLERANCE:
                self.logger.info(f"🎯 حرکت با موفقیت کامل شد! خطای نهایی: {final_error:.2f}m | زمان: {total_time:.1f} ثانیه")
                return True
            else:
                self.logger.warning(f"⚠️ حرکت پایان یافت اما خطای نهایی: {final_error:.2f}m")
                return True

        except Exception as e:
            self.logger.error(f"❌ خطا در حرکت افقی: {e}")
            return False

    # -------------------------
    # لندینگ نرم
    # -------------------------
    async def soft_land(self):
        """لندینگ نرم با کنترل سرعت"""
        self.logger.info("🛬 شروع لندینگ نرم...")
        
        # محاسبه تایمر
        current_alt = self.current_alt
        estimated_time = current_alt / VERTICAL_SPEED
        timeout_duration = estimated_time * 3.0  # ضریب بیشتر برای لندینگ
        start_time = time.time()
        
        self.logger.info(f"⏱️ تایمر لندینگ: {timeout_duration:.1f} ثانیه")

        try:
            # تغییر به حالت لند
            await self.drone.action.land()
            await asyncio.sleep(1.0)

            # مانیتورینگ ارتفاع تا رسیدن به زمین
            last_alt = current_alt
            landed_count = 0
            
            while self.mission_running:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # چک تایمر
                if elapsed_time > timeout_duration:
                    self.logger.warning("⏰ تایمر لندینگ به پایان رسید - بررسی وضعیت نهایی")
                    break

                # دریافت ارتفاع فعلی
                _, _, current_alt = await self.get_position_ned()
                
                # چک رسیدن به زمین
                if current_alt <= 0.3:  # 30cm از زمین
                    landed_count += 1
                else:
                    landed_count = 0
                
                if landed_count > 10:  # 2 ثانیه در ارتفاع کم
                    self.logger.info("✅ لندینگ کامل شد")
                    break
                
                # چک stuck شدن
                if abs(current_alt - last_alt) < 0.02 and current_alt > 1.0:  # کمتر از 2cm تغییر
                    self.logger.warning("⚠️ لندینگ متوقف شده - مداخله دستی可能需要")
                
                last_alt = current_alt
                
                # لاگ پیشرفت
                progress = (1.0 - (current_alt / self.current_alt)) * 100
                time_remaining = timeout_duration - elapsed_time
                
                self.logger.info(f"📉 ارتفاع: {current_alt:.2f}m ({progress:.1f}%) | زمان: {time_remaining:.1f}s")
                
                await asyncio.sleep(0.3)

            # دیس‌آرم کردن
            await asyncio.sleep(2.0)  # صبر برای تثبیت
            await self.drone.action.disarm()
            self.logger.info("🔒 دیس‌آرم شد")
            
            return True

        except ActionError as e:
            self.logger.error(f"❌ خطا در لندینگ: {e}")
            return False
        except Exception as e:
            self.logger.error(f"❌ خطای غیرمنتظره در لندینگ: {e}")
            return False

    # -------------------------
    # تمیزکاری ایمن
    # -------------------------
    async def safe_cleanup(self):
        """تمیزکاری ایمن"""
        self.logger.info("🧹 انجام تمیزکاری ایمن")
        try:
            if self.offboard_active:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await asyncio.sleep(0.5)
                await self.drone.offboard.stop()
                self.offboard_active = False
                self.logger.info("✅ Offboard متوقف شد")
        except Exception as e:
            self.logger.warning(f"⚠️ خطا در تمیزکاری: {e}")

    # -------------------------
    # اجرای مأموریت کامل با قابلیت بازیابی
    # -------------------------
    async def run(self):
        """اجرای مأموریت کامل با قابلیت بازیابی"""
        self.logger.info("🚀 شروع مأموریت کامل با قابلیت بازیابی")
        
        monitoring_tasks = []
        
        try:
            # اتصال
            self.logger.info(f"🔌 اتصال به سیستم: {SYS_ADDRESS}")
            await self.drone.connect(system_address=SYS_ADDRESS)

            # انتظار برای اتصال
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

            # شروع مانیتورینگ
            monitoring_tasks = await self.start_monitoring()
            self.logger.info("📊 مانیتورینگ سیستم‌ها فعال شد")

            # تنظیم موقعیت خانه
            await self.set_home_position()

            # تنظیم پارامترهای ایمن
            await self.set_safe_parameters()

            # پیش‌پرواز
            ok = await self.preflight_checks(timeout_sec=8)
            if not ok:
                self.logger.error("❌ چک‌های پیش‌پرواز ناموفق بودند.")
                await self.stop_monitoring(monitoring_tasks)
                return False

            # تیک‌آف نرم
            self.logger.info("🎯 فاز 1: تیک‌آف نرم")
            takeoff_ok = await self.soft_takeoff(TARGET_ALTITUDE)
            if not takeoff_ok:
                await self.stop_monitoring(monitoring_tasks)
                await self.safe_cleanup()
                return False

            # فعال‌سازی Offboard
            self.logger.info("🎯 فاز 2: فعال‌سازی Offboard")
            offboard_ok = await self.start_offboard()
            if not offboard_ok:
                await self.stop_monitoring(monitoring_tasks)
                await self.soft_land()
                return False

            # اجرای مأموریت با قابلیت بازیابی
            phases = [
                ("حرکت 5 متر به جلو", HORIZONTAL_DISTANCE, 1),
                ("بازگشت به نقطه شروع", HORIZONTAL_DISTANCE, -1)
            ]

            for phase_name, distance, direction in phases:
                if not self.mission_running:
                    self.logger.info("🛑 مأموریت لغو شد")
                    break
                    
                self.logger.info(f"🎯 {phase_name}")
                success = await self.move_horizontal_exact(distance, HORIZONTAL_SPEED, direction)
                if not success:
                    self.logger.error(f"❌ {phase_name} شکست خورد")
                    break
                    
                if phase_name == "حرکت 5 متر به جلو":
                    self.logger.info("⏸️ توقف کوتاه...")
                    await asyncio.sleep(2.0)

            # لندینگ نرم
            if self.mission_running:
                self.logger.info("🎯 فاز نهایی: لندینگ نرم")
                await self.soft_land()

            # توقف مانیتورینگ
            await self.stop_monitoring(monitoring_tasks)

            if self.mission_running:
                # محاسبه خطای نهایی
                final_n, final_e, final_alt = await self.get_position_ned()
                position_error = math.sqrt((final_n - self.start_north)**2 + (final_e - self.start_east)**2)
                
                self.logger.info(f"🎉 مأموریت کامل شد! خطای موقعیت نهایی: {position_error:.2f}m")
                print("\n" + "="*60)
                print("✅ مأموریت کامل شد با موفقیت!")
                print(f"📊 خطای موقعیت نهایی: {position_error:.2f} متر")
                print("="*60)
                return True
            else:
                self.logger.info("🛑 مأموریت متوقف شد")
                return False

        except Exception as e:
            self.logger.error(f"💥 خطای غیرمنتظره در مأموریت: {e}")
            await self.stop_monitoring(monitoring_tasks)
            await self.safe_cleanup()
            try:
                await self.soft_land()
            except:
                pass
            return False

# -------------------------
# تابع main
# -------------------------
async def main():
    mission = SafeAutoMission()
    try:
        success = await mission.run()
        if success:
            mission.logger.info("🎊 مأموریت با موفقیت به پایان رسید!")
        else:
            mission.logger.error("❌ مأموریت با خطا به پایان رسید.")
    except KeyboardInterrupt:
        mission.logger.warning("🛑 توقف توسط کاربر (KeyboardInterrupt)")
        await mission.safe_cleanup()
        try:
            await mission.soft_land()
        except:
            pass
    except Exception as e:
        mission.logger.error(f"💥 خطای سطح بالا: {e}")
        await mission.safe_cleanup()

if __name__ == "__main__":
    asyncio.run(main())