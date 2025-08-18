"""
پایه‌ی اصلی سیستم کنترل پهپاد با MAVSDK
نویسنده: تیم توسعه پرواز گروهی
"""

import asyncio
import math
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass
from enum import Enum

from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from mavsdk.mission_raw import MissionItem


class DroneState(Enum):
    """حالات مختلف پهپاد"""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ARMED = "armed" 
    FLYING = "flying"
    MISSION = "mission"
    RETURNING = "returning"
    LANDED = "landed"
    ERROR = "error"


@dataclass
class Position:
    """موقعیت سه‌بعدی پهپاد"""
    latitude: float
    longitude: float
    altitude: float
    
    def distance_to(self, other: 'Position') -> float:
        """محاسبه فاصله تا موقعیت دیگر (متر)"""
        R = 6371000  # شعاع زمین
        lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        lat2, lon2 = math.radians(other.latitude), math.radians(other.longitude)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        horizontal_dist = R * c
        vertical_dist = abs(self.altitude - other.altitude)
        
        return math.sqrt(horizontal_dist**2 + vertical_dist**2)


@dataclass
class DroneStatus:
    """وضعیت کامل پهپاد"""
    position: Position
    heading: float
    speed: float
    battery_level: float
    is_armed: bool
    flight_mode: str
    state: DroneState


class Drone:
    """کلاس اصلی کنترل پهپاد"""
    
    def __init__(self, drone_id: int, connection_string: str):
        self.id = drone_id
        self.connection_string = connection_string
        self.system = System()
        self.state = DroneState.DISCONNECTED
        
        # متغیرهای داخلی
        self._home_position: Optional[Position] = None
        self._current_position: Optional[Position] = None
        self._target_position: Optional[Position] = None
        self._status: Optional[DroneStatus] = None
        
        # تنظیمات پرواز
        self.max_speed = 10.0  # m/s
        self.min_altitude = 5.0  # متر
        self.max_altitude = 100.0  # متر
        self.position_tolerance = 1.0  # متر
        
        # لاگ رویدادها
        self.event_log = []
        
    async def connect(self) -> bool:
        """اتصال به پهپاد"""
        try:
            self._log_event(f"درحال اتصال به {self.connection_string}")
            await self.system.connect(system_address=self.connection_string)
            
            # انتظار برای اتصال
            async for state in self.system.core.connection_state():
                if state.is_connected:
                    self.state = DroneState.CONNECTED
                    self._log_event("اتصال موفق")
                    
                    # دریافت موقعیت خانه
                    await self._get_home_position()
                    return True
                    
        except Exception as e:
            self._log_event(f"خطا در اتصال: {str(e)}")
            self.state = DroneState.ERROR
            return False
            
    async def _get_home_position(self):
        """دریافت موقعیت خانه"""
        try:
            async for position in self.system.telemetry.home():
                self._home_position = Position(
                    position.latitude_deg,
                    position.longitude_deg, 
                    position.absolute_altitude_m
                )
                break
        except Exception as e:
            self._log_event(f"خطا در دریافت موقعیت خانه: {str(e)}")
            
    async def arm(self) -> bool:
        """آماده‌سازی برای پرواز"""
        try:
            if self.state != DroneState.CONNECTED:
                return False
                
            self._log_event("آماده‌سازی پهپاد")
            await self.system.action.arm()
            self.state = DroneState.ARMED
            
            # تأیید آماده‌سازی
            await asyncio.sleep(1)
            return await self.is_armed()
            
        except Exception as e:
            self._log_event(f"خطا در آماده‌سازی: {str(e)}")
            return False
            
    async def disarm(self) -> bool:
        """خاموش کردن پهپاد"""
        try:
            self._log_event("خاموش کردن پهپاد")
            await self.system.action.disarm()
            self.state = DroneState.CONNECTED
            return True
        except Exception as e:
            self._log_event(f"خطا در خاموش کردن: {str(e)}")
            return False
            
    async def takeoff(self, altitude: float = 20.0) -> bool:
        """برخاستن به ارتفاع مشخص"""
        try:
            if not await self.is_armed():
                if not await self.arm():
                    return False
                    
            self._log_event(f"برخاستن به ارتفاع {altitude} متر")
            await self.system.action.set_takeoff_altitude(altitude)
            await self.system.action.takeoff()
            
            self.state = DroneState.FLYING
            
            # انتظار تا رسیدن به ارتفاع هدف
            await self._wait_for_altitude(altitude)
            return True
            
        except Exception as e:
            self._log_event(f"خطا در برخاستن: {str(e)}")
            return False
            
    async def land(self) -> bool:
        """فرود آمدن"""
        try:
            self._log_event("شروع فرود")
            await self.system.action.land()
            self.state = DroneState.RETURNING
            
            # انتظار تا فرود کامل
            await self._wait_for_landing()
            self.state = DroneState.LANDED
            return True
            
        except Exception as e:
            self._log_event(f"خطا در فرود: {str(e)}")
            return False
            
    async def goto_position(self, position: Position, speed: float = None) -> bool:
        """حرکت به موقعیت مشخص"""
        try:
            if speed and speed != self.max_speed:
                await self.system.action.set_maximum_speed(speed)
                
            self._target_position = position
            self._log_event(f"حرکت به موقعیت {position.latitude:.6f}, {position.longitude:.6f}")
            
            # استفاده از offboard mode برای کنترل دقیق‌تر
            await self._goto_position_offboard(position)
            return True
            
        except Exception as e:
            self._log_event(f"خطا در حرکت: {str(e)}")
            return False
            
    async def _goto_position_offboard(self, target: Position):
        """حرکت با استفاده از حالت offboard"""
        current_pos = await self.get_current_position()
        if not current_pos:
            raise Exception("نمی‌توان موقعیت فعلی را دریافت کرد")
            
        # محاسبه موقعیت نسبی NED
        ned_north = (target.latitude - current_pos.latitude) * 111319.9  # تقریبی
        ned_east = (target.longitude - current_pos.longitude) * 111319.9 * math.cos(math.radians(current_pos.latitude))
        ned_down = -(target.altitude - current_pos.altitude)
        
        # شروع حالت offboard
        await self.system.offboard.set_position_ned(
            PositionNedYaw(ned_north, ned_east, ned_down, 0.0)
        )
        await self.system.offboard.start()
        
        # انتظار تا رسیدن به مقصد
        await self._wait_for_position(target)
        
        # توقف حالت offboard
        await self.system.offboard.stop()
        
    async def _wait_for_position(self, target: Position, timeout: float = 30.0):
        """انتظار تا رسیدن به موقعیت هدف"""
        start_time = asyncio.get_event_loop().time()
        
        while True:
            if asyncio.get_event_loop().time() - start_time > timeout:
                raise Exception(f"زمان انتظار برای رسیدن به موقعیت تمام شد")
                
            current = await self.get_current_position()
            if current and current.distance_to(target) < self.position_tolerance:
                break
                
            await asyncio.sleep(0.5)
            
    async def _wait_for_altitude(self, target_altitude: float, timeout: float = 30.0):
        """انتظار تا رسیدن به ارتفاع هدف"""
        start_time = asyncio.get_event_loop().time()
        
        while True:
            if asyncio.get_event_loop().time() - start_time > timeout:
                break
                
            current = await self.get_current_position()
            if current and abs(current.altitude - target_altitude) < 2.0:
                break
                
            await asyncio.sleep(1.0)
            
    async def _wait_for_landing(self, timeout: float = 60.0):
        """انتظار تا فرود کامل"""
        start_time = asyncio.get_event_loop().time()
        
        while True:
            if asyncio.get_event_loop().time() - start_time > timeout:
                break
                
            if not await self.is_armed():
                break
                
            await asyncio.sleep(2.0)
            
    async def get_current_position(self) -> Optional[Position]:
        """دریافت موقعیت فعلی"""
        try:
            async for position in self.system.telemetry.position():
                self._current_position = Position(
                    position.latitude_deg,
                    position.longitude_deg,
                    position.absolute_altitude_m
                )
                return self._current_position
        except:
            return None
            
    async def get_status(self) -> Optional[DroneStatus]:
        """دریافت وضعیت کامل پهپاد"""
        try:
            position = await self.get_current_position()
            if not position:
                return None
                
            # دریافت اطلاعات اضافی
            heading = 0.0
            speed = 0.0
            battery = 100.0
            is_armed_status = await self.is_armed()
            flight_mode = "UNKNOWN"
            
            # سعی در دریافت اطلاعات تکمیلی
            try:
                async for att in self.system.telemetry.attitude_euler():
                    heading = att.yaw_deg
                    break
                    
                async for vel in self.system.telemetry.velocity_ned():
                    speed = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2 + vel.down_m_s**2)
                    break
                    
                async for bat in self.system.telemetry.battery():
                    battery = bat.remaining_percent
                    break
                    
                async for mode in self.system.telemetry.flight_mode():
                    flight_mode = str(mode)
                    break
            except:
                pass
                
            self._status = DroneStatus(
                position=position,
                heading=heading,
                speed=speed,
                battery_level=battery,
                is_armed=is_armed_status,
                flight_mode=flight_mode,
                state=self.state
            )
            
            return self._status
            
        except Exception as e:
            self._log_event(f"خطا در دریافت وضعیت: {str(e)}")
            return None
            
    async def is_armed(self) -> bool:
        """بررسی آماده بودن پهپاد"""
        try:
            async for armed in self.system.telemetry.armed():
                return armed
        except:
            return False
            
    async def return_to_home(self) -> bool:
        """بازگشت به خانه"""
        try:
            if not self._home_position:
                await self._get_home_position()
                
            if self._home_position:
                self.state = DroneState.RETURNING
                self._log_event("بازگشت به خانه")
                await self.system.action.return_to_launch()
                return True
            return False
            
        except Exception as e:
            self._log_event(f"خطا در بازگشت به خانه: {str(e)}")
            return False
            
    def get_home_position(self) -> Optional[Position]:
        """دریافت موقعیت خانه"""
        return self._home_position
        
    def _log_event(self, message: str):
        """ثبت رویداد در لاگ"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] پهپاد {self.id}: {message}"
        self.event_log.append(log_entry)
        print(log_entry)  # نمایش در کنسول
        
    def get_logs(self) -> list:
        """دریافت لاگ رویدادها"""
        return self.event_log.copy()
        
    async def emergency_stop(self):
        """توقف اضطراری"""
        try:
            self._log_event("توقف اضطراری فعال شد")
            await self.system.action.kill()
            self.state = DroneState.ERROR
        except Exception as e:
            self._log_event(f"خطا در توقف اضطراری: {str(e)}")
            
    def is_alive(self) -> bool:
        """بررسی فعال بودن پهپاد"""
        return self.state not in [DroneState.DISCONNECTED, DroneState.ERROR, DroneState.LANDED]
        
    def __str__(self) -> str:
        """نمایش اطلاعات پهپاد"""
        return f"پهپاد {self.id} [{self.state.value}] @ {self.connection_string}"