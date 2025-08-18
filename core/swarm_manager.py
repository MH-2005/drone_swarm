"""
مدیر اصلی گروه پهپادها
مسئول هماهنگی و کنترل کلی عملیات
"""

import asyncio
import time
import json
from typing import List, Dict, Optional, Tuple, Any
from dataclasses import dataclass, asdict
from enum import Enum

from .drone import Drone, Position, DroneState


class SwarmState(Enum):
    """حالات مختلف گروه"""
    INITIALIZING = "initializing"
    READY = "ready"
    FORMATION = "formation"
    MISSION = "mission"
    EMERGENCY = "emergency"


@dataclass
class SwarmConfig:
    """تنظیمات گروه"""
    num_drones: int = 4
    base_port: int = 14540
    formation_spacing: float = 5.0  # فاصله پیش‌فرض در آرایش
    min_altitude: float = 10.0
    max_altitude: float = 50.0
    safety_distance: float = 3.0  # حداقل فاصله ایمنی
    max_distance: float = 10.0  # حداکثر فاصله مجاز
    

class SwarmManager:
    """مدیر اصلی گروه پهپادها"""
    
    def __init__(self, config: SwarmConfig = None):
        self.config = config or SwarmConfig()
        self.drones: List[Drone] = []
        self.leader_id: Optional[int] = 0  # پیش‌فرض اولین پهپاد
        self.state = SwarmState.INITIALIZING
        
        # آمار و نظارت
        self.mission_start_time: Optional[float] = None
        self.total_distance_traveled = 0.0
        self.formation_changes = 0
        self.leader_changes = 0
        
        # تنظیمات ایمنی
        self.emergency_land = False
        self.collision_detected = False
        
        print(f"مدیر گروه با {self.config.num_drones} پهپاد ایجاد شد")
        
    async def initialize_swarm(self) -> bool:
        """راه‌اندازی اولیه گروه"""
        try:
            print("شروع راه‌اندازی گروه...")
            
            # ایجاد پهپادها
            for i in range(self.config.num_drones):
                connection = f"udp://:{self.config.base_port + i}"
                drone = Drone(drone_id=i, connection_string=connection)
                self.drones.append(drone)
                
            # اتصال همزمان همه پهپادها
            connection_tasks = [drone.connect() for drone in self.drones]
            results = await asyncio.gather(*connection_tasks, return_exceptions=True)
            
            # بررسی نتایج اتصال
            connected_count = 0
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    print(f"خطا در اتصال پهپاد {i}: {result}")
                elif result:
                    connected_count += 1
                    
            if connected_count < 2:  # حداقل 2 پهپاد نیاز است
                print(f"تنها {connected_count} پهپاد متصل شد. حداقل 2 پهپاد نیاز است.")
                return False
                
            print(f"{connected_count} پهپاد از {self.config.num_drones} با موفقیت متصل شد")
            
            # انتظار برای پایدار شدن اتصالات
            await asyncio.sleep(3)
            
            self.state = SwarmState.READY
            return True
            
        except Exception as e:
            print(f"خطا در راه‌اندازی گروه: {str(e)}")
            self.state = SwarmState.EMERGENCY
            return False
            
    async def arm_all_drones(self) -> bool:
        """آماده‌سازی همه پهپادها"""
        try:
            print("آماده‌سازی همه پهپادها...")
            
            arm_tasks = []
            for drone in self.get_active_drones():
                arm_tasks.append(drone.arm())
                
            results = await asyncio.gather(*arm_tasks, return_exceptions=True)
            
            armed_count = sum(1 for result in results if result is True)
            print(f"{armed_count} پهپاد آماده شد")
            
            return armed_count >= len(self.get_active_drones())
            
        except Exception as e:
            print(f"خطا در آماده‌سازی: {str(e)}")
            return False
            
    async def takeoff_all_drones(self, altitude: float = 20.0) -> bool:
        """برخاستن همه پهپادها"""
        try:
            # اطمینان از آماده بودن
            if not await self.arm_all_drones():
                return False
                
            print(f"برخاستن همه پهپادها به ارتفاع {altitude} متر...")
            
            takeoff_tasks = []
            for drone in self.get_active_drones():
                takeoff_tasks.append(drone.takeoff(altitude))
                
            results = await asyncio.gather(*takeoff_tasks, return_exceptions=True)
            
            success_count = sum(1 for result in results if result is True)
            print(f"{success_count} پهپاد با موفقیت برخاست")
            
            if success_count >= len(self.get_active_drones()):
                self.state = SwarmState.FORMATION
                self.mission_start_time = time.time()
                return True
                
            return False
            
        except Exception as e:
            print(f"خطا در برخاستن: {str(e)}")
            return False
            
    async def land_all_drones(self) -> bool:
        """فرود همه پهپادها"""
        try:
            print("فرود همه پهپادها...")
            
            land_tasks = []
            for drone in self.get_active_drones():
                land_tasks.append(drone.land())
                
            results = await asyncio.gather(*land_tasks, return_exceptions=True)
            
            success_count = sum(1 for result in results if result is True)
            print(f"{success_count} پهپاد با موفقیت فرود آمد")
            
            self.state = SwarmState.READY
            return True
            
        except Exception as e:
            print(f"خطا در فرود: {str(e)}")
            return False
            
    def get_active_drones(self) -> List[Drone]:
        """دریافت پهپادهای فعال"""
        return [drone for drone in self.drones if drone.is_alive()]
        
    def get_leader(self) -> Optional[Drone]:
        """دریافت پهپاد رهبر"""
        if self.leader_id is not None and self.leader_id < len(self.drones):
            leader = self.drones[self.leader_id]
            if leader.is_alive():
                return leader
                
        # اگر رهبر فعلی موجود نیست، رهبر جدید انتخاب کن
        self._select_new_leader()
        
        if self.leader_id is not None:
            return self.drones[self.leader_id]
        return None
        
    def get_followers(self) -> List[Drone]:
        """دریافت پهپادهای پیرو"""
        followers = []
        for drone in self.get_active_drones():
            if drone.id != self.leader_id:
                followers.append(drone)
        return followers
        
    def _select_new_leader(self) -> bool:
        """انتخاب رهبر جدید به صورت هوشمند"""
        active_drones = self.get_active_drones()
        
        if not active_drones:
            self.leader_id = None
            return False
            
        # الگوریتم انتخاب رهبر هوشمند
        best_candidate = None
        best_score = -1
        
        for drone in active_drones:
            if drone.id == self.leader_id:
                continue  # رهبر فعلی را در نظر نگیر
                
            score = self._calculate_leadership_score(drone)
            if score > best_score:
                best_score = score
                best_candidate = drone
                
        if best_candidate:
            old_leader = self.leader_id
            self.leader_id = best_candidate.id
            self.leader_changes += 1
            
            print(f"رهبری از پهپاد {old_leader} به پهپاد {self.leader_id} منتقل شد")
            return True
            
        # اگر هیچ کاندید مناسبی نبود، اولین پهپاد فعال را انتخاب کن
        self.leader_id = active_drones[0].id
        return True
        
    def _calculate_leadership_score(self, drone: Drone) -> float:
        """محاسبه امتیاز رهبری پهپاد"""
        score = 0.0
        
        # امتیاز بر اساس ID (پهپادهای کم‌شماره ترجیح دارند)
        score += (10 - drone.id) * 10
        
        # امتیاز بر اساس وضعیت پرواز
        if drone.state == DroneState.FLYING:
            score += 50
        elif drone.state == DroneState.ARMED:
            score += 30
            
        # در آینده می‌توان امتیازات بیشتری اضافه کرد:
        # - فاصله تا مرکز گروه
        # - سطح باتری
        # - کیفیت سیگنال
        
        return score
        
    async def disarm_leader(self) -> bool:
        """حذف رهبر فعلی (دستور مطابق شیوه‌نامه)"""
        try:
            leader = self.get_leader()
            if not leader:
                print("هیچ رهبری برای حذف وجود ندارد")
                return False
                
            print(f"حذف رهبر پهپاد {leader.id}")
            
            # خاموش کردن رهبر فعلی
            await leader.disarm()
            
            # بازگشت رهبر به خانه (امتیاز تشویقی)
            asyncio.create_task(leader.return_to_home())
            
            # انتخاب رهبر جدید
            self._select_new_leader()
            
            new_leader = self.get_leader()
            if new_leader:
                print(f"رهبر جدید: پهپاد {new_leader.id}")
                return True
                
            return False
            
        except Exception as e:
            print(f"خطا در حذف رهبر: {str(e)}")
            return False
            
    async def check_safety_constraints(self) -> bool:
        """بررسی قیدهای ایمنی"""
        try:
            active_drones = self.get_active_drones()
            
            # دریافت موقعیت همه پهپادها
            positions = {}
            for drone in active_drones:
                pos = await drone.get_current_position()
                if pos:
                    positions[drone.id] = pos
                    
            # بررسی فاصله بین پهپادها
            drone_ids = list(positions.keys())
            for i in range(len(drone_ids)):
                for j in range(i + 1, len(drone_ids)):
                    id1, id2 = drone_ids[i], drone_ids[j]
                    distance = positions[id1].distance_to(positions[id2])
                    
                    if distance < self.config.safety_distance:
                        print(f"⚠️ فاصله خطرناک بین پهپاد {id1} و {id2}: {distance:.2f}m")
                        self.collision_detected = True
                        return False
                        
                    if distance > self.config.max_distance:
                        print(f"⚠️ فاصله بیش از حد بین پهپاد {id1} و {id2}: {distance:.2f}m")
                        
            self.collision_detected = False
            return True
            
        except Exception as e:
            print(f"خطا در بررسی ایمنی: {str(e)}")
            return False
            
    async def emergency_procedures(self):
        """اقدامات اضطراری"""
        print("🚨 فعال‌سازی پروتکل اضطراری")
        self.state = SwarmState.EMERGENCY
        self.emergency_land = True
        
        # فرود اضطراری همه پهپادها
        emergency_tasks = []
        for drone in self.get_active_drones():
            emergency_tasks.append(drone.land())
            
        await asyncio.gather(*emergency_tasks, return_exceptions=True)
        print("فرود اضطراری کامل شد")
        
    async def get_swarm_status(self) -> Dict[str, Any]:
        """دریافت وضعیت کامل گروه"""
        try:
            active_drones = self.get_active_drones()
            
            # جمع‌آوری اطلاعات پهپادها
            drone_statuses = {}
            total_battery = 0
            flying_count = 0
            
            for drone in active_drones:
                status = await drone.get_status()
                if status:
                    drone_statuses[drone.id] = {
                        'position': asdict(status.position),
                        'state': status.state.value,
                        'battery': status.battery_level,
                        'is_armed': status.is_armed,
                        'speed': status.speed
                    }
                    total_battery += status.battery_level
                    if status.state == DroneState.FLYING:
                        flying_count += 1
                        
            # محاسبه آمار کلی
            avg_battery = total_battery / len(active_drones) if active_drones else 0
            mission_duration = (time.time() - self.mission_start_time) if self.mission_start_time else 0
            
            return {
                'swarm_state': self.state.value,
                'total_drones': len(self.drones),
                'active_drones': len(active_drones),
                'flying_drones': flying_count,
                'leader_id': self.leader_id,
                'average_battery': round(avg_battery, 1),
                'mission_duration': round(mission_duration, 2),
                'formation_changes': self.formation_changes,
                'leader_changes': self.leader_changes,
                'emergency_active': self.emergency_land,
                'collision_detected': self.collision_detected,
                'drone_details': drone_statuses
            }
            
        except Exception as e:
            print(f"خطا در دریافت وضعیت گروه: {str(e)}")
            return {'error': str(e)}
            
    async def monitor_swarm(self, interval: float = 2.0):
        """نظارت مداوم بر گروه"""
        print("شروع نظارت مداوم...")
        
        while not self.emergency_land:
            try:
                # بررسی ایمنی
                if not await self.check_safety_constraints():
                    if self.collision_detected:
                        await self.emergency_procedures()
                        break
                        
                # نمایش وضعیت
                status = await self.get_swarm_status()
                print(f"وضعیت گروه: {status['active_drones']}/{status['total_drones']} فعال، "
                      f"رهبر: {status['leader_id']}, باتری: {status['average_battery']}%")
                      
                await asyncio.sleep(interval)
                
            except Exception as e:
                print(f"خطا در نظارت: {str(e)}")
                await asyncio.sleep(interval)
                
    def log_formation_change(self):
        """ثبت تغییر آرایش"""
        self.formation_changes += 1
        print(f"تغییر آرایش #{self.formation_changes}")
        
    async def shutdown_swarm(self):
        """خاتمه کار گروه"""
        print("شروع خاتمه کار گروه...")
        
        try:
            # فرود همه پهپادها
            await self.land_all_drones()
            
            # خاموش کردن همه پهپادها
            disarm_tasks = []
            for drone in self.drones:
                if drone.is_alive():
                    disarm_tasks.append(drone.disarm())
                    
            await asyncio.gather(*disarm_tasks, return_exceptions=True)
            
            # نمایش آمار نهایی
            final_stats = await self.get_swarm_status()
            print("\n📊 آمار نهایی مأموریت:")
            print(f"   مدت زمان: {final_stats.get('mission_duration', 0):.2f} ثانیه")
            print(f"   تعداد تغییرات آرایش: {final_stats.get('formation_changes', 0)}")
            print(f"   تعداد تغییرات رهبر: {final_stats.get('leader_changes', 0)}")
            print(f"   متوسط باتری پایانی: {final_stats.get('average_battery', 0):.1f}%")
            
            self.state = SwarmState.READY
            print("خاتمه کار گروه با موفقیت انجام شد")
            
        except Exception as e:
            print(f"خطا در خاتمه کار: {str(e)}")
            
    def __str__(self) -> str:
        """نمایش اطلاعات گروه"""
        active_count = len(self.get_active_drones())
        return (f"گروه پهپاد [{self.state.value}]: "
                f"{active_count}/{len(self.drones)} فعال، "
                f"رهبر: {self.leader_id}")