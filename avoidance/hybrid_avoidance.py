"""
سیستم مانع‌گریزی ترکیبی
ترکیب ArduPilot BendyRuler با الگوریتم‌های هوشمند
برای کسب امتیاز تشویقی مطابق شیوه‌نامه
"""

import asyncio
import math
import time
from typing import List, Dict, Optional, Tuple, Set
from dataclasses import dataclass
from enum import Enum

from ..core.drone import Drone, Position, DroneState
from ..core.swarm_manager import SwarmManager


class ObstacleType(Enum):
    """انواع موانع"""
    STATIC = "static"      # موانع ثابت
    DYNAMIC = "dynamic"    # موانع متحرک
    SWARM = "swarm"       # پهپادهای دیگر


class AvoidanceStrategy(Enum):
    """استراتژی‌های مانع‌گریزی"""
    ARDUCOPTER_ONLY = "arducopter_only"     # فقط ArduPilot
    HYBRID = "hybrid"                       # ترکیبی
    FORMATION_AWARE = "formation_aware"     # آگاه از آرایش
    INTELLIGENT = "intelligent"             # هوشمند


@dataclass
class ObstacleInfo:
    """اطلاعات مانع"""
    obstacle_id: str
    position: Position
    size: float  # شعاع مانع (متر)
    type: ObstacleType
    last_seen: float
    velocity: Optional[Tuple[float, float, float]] = None  # برای موانع متحرک


@dataclass
class AvoidanceConfig:
    """تنظیمات مانع‌گریزی"""
    # فاصله‌ها
    detection_range: float = 15.0      # برد تشخیص (متر)
    safe_distance: float = 5.0         # فاصله ایمن (متر)
    critical_distance: float = 3.0     # فاصله بحرانی (متر)
    
    # سرعت‌ها
    normal_speed: float = 5.0          # سرعت عادی
    avoidance_speed: float = 3.0       # سرعت مانع‌گریزی
    emergency_speed: float = 1.0       # سرعت اضطراری
    
    # زمان‌ها
    obstacle_timeout: float = 10.0     # زمان انقضای مانع
    recovery_time: float = 5.0         # زمان بازیابی آرایش
    
    # ArduPilot تنظیمات
    enable_arducopter_avoidance: bool = True
    arducopter_margin: float = 2.0


class HybridAvoidanceController:
    """کنترلر مانع‌گریزی ترکیبی"""
    
    def __init__(self, swarm_manager: SwarmManager):
        self.swarm = swarm_manager
        self.config = AvoidanceConfig()
        
        # وضعیت سیستم
        self.strategy = AvoidanceStrategy.HYBRID
        self.is_active = False
        self.avoidance_task: Optional[asyncio.Task] = None
        
        # اطلاعات موانع
        self.detected_obstacles: Dict[str, ObstacleInfo] = {}
        self.obstacle_history = []
        
        # وضعیت پهپادها
        self.drones_in_avoidance: Set[int] = set()
        self.formation_backup = {}
        
        print("کنترلر مانع‌گریزی ترکیبی آماده شد")
        
    async def initialize_avoidance_system(self) -> bool:
        """راه‌اندازی سیستم مانع‌گریزی"""
        try:
            print("راه‌اندازی سیستم مانع‌گریزی...")
            
            # فعال‌سازی سیستم ArduPilot
            if self.config.enable_arducopter_avoidance:
                await self._setup_arducopter_avoidance()
                
            # شروع نظارت
            self.is_active = True
            self.avoidance_task = asyncio.create_task(self._obstacle_monitoring_loop())
            
            print("✅ سیستم مانع‌گریزی فعال شد")
            return True
            
        except Exception as e:
            print(f"❌ خطا در راه‌اندازی مانع‌گریزی: {str(e)}")
            return False
            
    async def _setup_arducopter_avoidance(self):
        """تنظیم مانع‌گریزی ArduPilot"""
        for drone in self.swarm.get_active_drones():
            try:
                # فعال‌سازی تمام انواع مانع‌گریزی
                await drone.system.param.set_param_float("AVOID_ENABLE", 7.0)
                
                # تنظیم فاصله تشخیص
                await drone.system.param.set_param_float("AVOID_DIST_MAX", self.config.detection_range)
                
                # تنظیم حاشیه ایمنی
                await drone.system.param.set_param_float("AVOID_MARGIN", self.config.arducopter_margin)
                
                # فعال‌سازی BendyRuler
                await drone.system.param.set_param_float("AVOID_BEHAVE", 0.0)  # Slide
                
                print(f"ArduPilot avoidance برای پهپاد {drone.id} فعال شد")
                
            except Exception as e:
                print(f"خطا در تنظیم ArduPilot avoidance برای پهپاد {drone.id}: {str(e)}")
                
    async def _obstacle_monitoring_loop(self):
        """حلقه نظارت بر موانع"""
        try:
            while self.is_active:
                await self._scan_for_obstacles()
                await self._process_detected_obstacles()
                await self._cleanup_old_obstacles()
                
                await asyncio.sleep(0.5)  # نظارت با فرکانس 2Hz
                
        except asyncio.CancelledError:
            print("نظارت موانع متوقف شد")
        except Exception as e:
            print(f"خطا در نظارت موانع: {str(e)}")
            
    async def _scan_for_obstacles(self):
        """اسکن برای یافتن موانع"""
        try:
            # در اینجا معمولاً از سنسورهای پهپاد استفاده می‌شود
            # برای شبیه‌سازی، موانع تصادفی یا از محیط Gazebo تشخیص می‌دهیم
            
            # تشخیص برخورد بالقوه بین پهپادهای گروه
            await self._detect_swarm_collisions()
            
            # در صورت وجود سنسور، اینجا موانع محیطی را تشخیص می‌دهیم
            # await self._detect_environmental_obstacles()
            
        except Exception as e:
            print(f"خطا در اسکن موانع: {str(e)}")
            
    async def _detect_swarm_collisions(self):
        """تشخیص برخورد احتمالی بین پهپادهای گروه"""
        active_drones = self.swarm.get_active_drones()
        
        for i in range(len(active_drones)):
            for j in range(i + 1, len(active_drones)):
                drone1, drone2 = active_drones[i], active_drones[j]
                
                pos1 = await drone1.get_current_position()
                pos2 = await drone2.get_current_position()
                
                if pos1 and pos2:
                    distance = pos1.distance_to(pos2)
                    
                    if distance < self.config.safe_distance:
                        # ثبت مانع (پهپاد دیگر)
                        obstacle_id = f"drone_{drone2.id}_for_{drone1.id}"
                        
                        self.detected_obstacles[obstacle_id] = ObstacleInfo(
                            obstacle_id=obstacle_id,
                            position=pos2,
                            size=1.5,  # شعاع ایمن پهپاد
                            type=ObstacleType.SWARM,
                            last_seen=time.time()
                        )
                        
    async def _process_detected_obstacles(self):
        """پردازش موانع تشخیص داده شده"""
        if not self.detected_obstacles:
            return
            
        for obstacle_id, obstacle in self.detected_obstacles.items():
            await self._handle_obstacle(obstacle)
            
    async def _handle_obstacle(self, obstacle: ObstacleInfo):
        """مدیریت یک مانع مشخص"""
        try:
            # یافتن پهپادهای در معرض خطر
            threatened_drones = await self._find_threatened_drones(obstacle)
            
            for drone in threatened_drones:
                if drone.id not in self.drones_in_avoidance:
                    # شروع مانع‌گریزی برای این پهپاد
                    await self._start_avoidance_for_drone(drone, obstacle)
                    
        except Exception as e:
            print(f"خطا در مدیریت مانع {obstacle.obstacle_id}: {str(e)}")
            
    async def _find_threatened_drones(self, obstacle: ObstacleInfo) -> List[Drone]:
        """یافتن پهپادهای در معرض خطر"""
        threatened = []
        
        for drone in self.swarm.get_active_drones():
            drone_pos = await drone.get_current_position()
            if not drone_pos:
                continue
                
            distance = drone_pos.distance_to(obstacle.position)
            
            if distance <= self.config.detection_range:
                # بررسی مسیر حرکت (پیش‌بینی برخورد)
                collision_risk = await self._calculate_collision_risk(drone, obstacle)
                
                if collision_risk > 0.3:  # آستانه خطر
                    threatened.append(drone)
                    
        return threatened
        
    async def _calculate_collision_risk(self, drone: Drone, obstacle: ObstacleInfo) -> float:
        """محاسبه ریسک برخورد"""
        try:
            drone_pos = await drone.get_current_position()
            if not drone_pos:
                return 0.0
                
            # فاصله فعلی
            current_distance = drone_pos.distance_to(obstacle.position)
            
            # ریسک بر اساس فاصله
            if current_distance <= self.config.critical_distance:
                return 1.0  # خطر بحرانی
            elif current_distance <= self.config.safe_distance:
                return 0.7  # خطر بالا
            elif current_distance <= self.config.detection_range:
                return 0.3  # خطر متوسط
            else:
                return 0.0  # بدون خطر
                
        except Exception as e:
            print(f"خطا در محاسبه ریسک برخورد: {str(e)}")
            return 0.0
            
    async def _start_avoidance_for_drone(self, drone: Drone, obstacle: ObstacleInfo):
        """شروع مانع‌گریزی برای یک پهپاد"""
        try:
            print(f"🚨 شروع مانع‌گریزی برای پهپاد {drone.id}")
            
            self.drones_in_avoidance.add(drone.id)
            
            # ذخیره موقعیت فعلی برای بازیابی
            current_pos = await drone.get_current_position()
            if current_pos:
                self.formation_backup[drone.id] = current_pos
                
            # انتخاب استراتژی مانع‌گریزی
            if self.strategy == AvoidanceStrategy.FORMATION_AWARE:
                await self._formation_aware_avoidance(drone, obstacle)
            elif self.strategy == AvoidanceStrategy.INTELLIGENT:
                await self._intelligent_avoidance(drone, obstacle)
            else:
                await self._simple_avoidance(drone, obstacle)
                
        except Exception as e:
            print(f"خطا در شروع مانع‌گریزی برای پهپاد {drone.id}: {str(e)}")
            
    async def _formation_aware_avoidance(self, drone: Drone, obstacle: ObstacleInfo):
        """مانع‌گریزی با حفظ آرایش"""
        try:
            # محاسبه موقعیت جایگزین با حفظ آرایش کلی
            safe_position = await self._calculate_formation_safe_position(drone, obstacle)
            
            if safe_position:
                # حرکت به موقعیت ایمن
                await drone.goto_position(safe_position, self.config.avoidance_speed)
                
                # برنامه‌ریزی بازگشت به آرایش
                asyncio.create_task(self._schedule_formation_recovery(drone))
                
        except Exception as e:
            print(f"خطا در مانع‌گریزی آگاه از آرایش: {str(e)}")
            
    async def _calculate_formation_safe_position(self, drone: Drone, obstacle: ObstacleInfo) -> Optional[Position]:
        """محاسبه موقعیت ایمن با حفظ آرایش"""
        try:
            drone_pos = await drone.get_current_position()
            if not drone_pos:
                return None
                
            # بردار از مانع به پهپاد
            lat_diff = drone_pos.latitude - obstacle.position.latitude
            lon_diff = drone_pos.longitude - obstacle.position.longitude
            alt_diff = drone_pos.altitude - obstacle.position.altitude
            
            # نرمال‌سازی
            distance = math.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)
            if distance == 0:
                return None
                
            unit_lat = lat_diff / distance
            unit_lon = lon_diff / distance
            unit_alt = alt_diff / distance
            
            # محاسبه موقعیت ایمن
            safe_distance = self.config.safe_distance + obstacle.size
            
            safe_lat = obstacle.position.latitude + unit_lat * safe_distance * (111319.9)
            safe_lon = obstacle.position.longitude + unit_lon * safe_distance * (111319.9 * math.cos(math.radians(obstacle.position.latitude)))
            safe_alt = max(self.swarm.config.min_altitude, obstacle.position.altitude + unit_alt * safe_distance)
            
            return Position(safe_lat, safe_lon, safe_alt)
            
        except Exception as e:
            print(f"خطا در محاسبه موقعیت ایمن: {str(e)}")
            return None
            
    async def _intelligent_avoidance(self, drone: Drone, obstacle: ObstacleInfo):
        """مانع‌گریزی هوشمند"""
        try:
            # در نظر گیری چندین فاکتور:
            # 1. مسیر بهینه دور از مانع
            # 2. هماهنگی با سایر پهپادها
            # 3. کمینه انحراف از مسیر اصلی
            
            optimal_path = await self._calculate_optimal_avoidance_path(drone, obstacle)
            
            if optimal_path:
                for waypoint in optimal_path:
                    await drone.goto_position(waypoint, self.config.avoidance_speed)
                    
        except Exception as e:
            print(f"خطا در مانع‌گریزی هوشمند: {str(e)}")
            
    async def _simple_avoidance(self, drone: Drone, obstacle: ObstacleInfo):
        """مانع‌گریزی ساده"""
        try:
            # حرکت عمودی برای دوری از مانع
            drone_pos = await drone.get_current_position()
            if not drone_pos:
                return
                
            # افزایش ارتفاع
            safe_altitude = max(
                obstacle.position.altitude + self.config.safe_distance,
                drone_pos.altitude + 5.0
            )
            
            safe_position = Position(
                drone_pos.latitude,
                drone_pos.longitude,
                min(safe_altitude, self.swarm.config.max_altitude)
            )
            
            await drone.goto_position(safe_position, self.config.avoidance_speed)
            
        except Exception as e:
            print(f"خطا در مانع‌گریزی ساده: {str(e)}")
            
    async def _schedule_formation_recovery(self, drone: Drone):
        """برنامه‌ریزی بازیابی آرایش"""
        try:
            # انتظار برای گذشتن از مانع
            await asyncio.sleep(self.config.recovery_time)
            
            # بررسی امنیت برای بازگشت
            if await self._is_safe_to_recover(drone):
                await self._recover_drone_to_formation(drone)
                
        except Exception as e:
            print(f"خطا در بازیابی آرایش برای پهپاد {drone.id}: {str(e)}")
            
    async def _is_safe_to_recover(self, drone: Drone) -> bool:
        """بررسی امنیت برای بازگشت به آرایش"""
        try:
            # بررسی عدم وجود مانع در مسیر بازگشت
            for obstacle in self.detected_obstacles.values():
                if time.time() - obstacle.last_seen > self.config.obstacle_timeout:
                    continue
                    
                # اگر مانع همچنان فعال است
                drone_pos = await drone.get_current_position()
                if drone_pos and drone_pos.distance_to(obstacle.position) < self.config.safe_distance:
                    return False
                    
            return True
            
        except Exception as e:
            print(f"خطا در بررسی امنیت بازیابی: {str(e)}")
            return False
            
    async def _recover_drone_to_formation(self, drone: Drone):
        """بازیابی پهپاد به آرایش"""
        try:
            print(f"🔄 بازیابی پهپاد {drone.id} به آرایش")
            
            # حذف از لیست مانع‌گریزی
            self.drones_in_avoidance.discard(drone.id)
            
            # بازگشت به موقعیت آرایش (اگر امکان پذیر باشد)
            # این کار معمولاً توسط کنترلر رهبر-پیرو انجام می‌شود
            
            # پاکسازی backup
            if drone.id in self.formation_backup:
                del self.formation_backup[drone.id]
                
        except Exception as e:
            print(f"خطا در بازیابی پهپاد {drone.id}: {str(e)}")
            
    async def _cleanup_old_obstacles(self):
        """پاکسازی موانع قدیمی"""
        current_time = time.time()
        expired_obstacles = []
        
        for obstacle_id, obstacle in self.detected_obstacles.items():
            if current_time - obstacle.last_seen > self.config.obstacle_timeout:
                expired_obstacles.append(obstacle_id)
                
        for obstacle_id in expired_obstacles:
            del self.detected_obstacles[obstacle_id]
            
    def set_avoidance_strategy(self, strategy: AvoidanceStrategy):
        """تنظیم استراتژی مانع‌گریزی"""
        self.strategy = strategy
        print(f"استراتژی مانع‌گریزی تغییر کرد: {strategy.value}")
        
    async def emergency_stop_all(self):
        """توقف اضطراری تمام پهپادها"""
        print("🚨 توقف اضطراری به دلیل مانع خطرناک")
        
        emergency_tasks = []
        for drone in self.swarm.get_active_drones():
            task = drone.system.action.hold()
            emergency_tasks.append(task)
            
        await asyncio.gather(*emergency_tasks, return_exceptions=True)
        
    def get_avoidance_status(self) -> Dict:
        """وضعیت سیستم مانع‌گریزی"""
        return {
            'is_active': self.is_active,
            'strategy': self.strategy.value,
            'detected_obstacles': len(self.detected_obstacles),
            'drones_in_avoidance': len(self.drones_in_avoidance),
            'obstacle_types': {
                obs_type.value: len([obs for obs in self.detected_obstacles.values() if obs.type == obs_type])
                for obs_type in ObstacleType
            },
            'config': {
                'detection_range': self.config.detection_range,
                'safe_distance': self.config.safe_distance,
                'arducopter_enabled': self.config.enable_arducopter_avoidance
            }
        }
        
    async def shutdown(self):
        """خاموش کردن سیستم مانع‌گریزی"""
        print("خاموش کردن سیستم مانع‌گریزی...")
        
        self.is_active = False
        
        if self.avoidance_task:
            self.avoidance_task.cancel()
            try:
                await self.avoidance_task
            except asyncio.CancelledError:
                pass
                
        # پاکسازی وضعیت
        self.detected_obstacles.clear()
        self.drones_in_avoidance.clear()
        self.formation_backup.clear()
        
        print("سیستم مانع‌گریزی خاموش شد")