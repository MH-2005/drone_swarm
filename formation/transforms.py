"""
ماژول دوران و جابه‌جایی آرایش‌ها
مطابق با مرحله دوم شیوه‌نامه
"""

import math
import asyncio
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from ..core.drone import Drone, Position
from ..core.swarm_manager import SwarmManager


class TransformationType(Enum):
    """انواع تبدیلات"""
    TRANSLATE = "translate"  # جابه‌جایی
    ROTATE = "rotate"        # دوران
    SCALE = "scale"          # تغییر مقیاس
    COMBINED = "combined"    # ترکیبی


@dataclass
class TransformationParameters:
    """پارامترهای تبدیل"""
    # جابه‌جایی (متر)
    move_x: float = 0.0
    move_y: float = 0.0  
    move_z: float = 0.0
    
    # دوران (درجه)
    rotate_x: float = 0.0
    rotate_y: float = 0.0
    rotate_z: float = 0.0
    
    # تغییر مقیاس
    scale_factor: float = 1.0
    
    # پارامترهای اجرا
    max_speed: float = 5.0      # حداکثر سرعت حرکت
    coordination_delay: float = 0.1  # تاخیر هماهنگی
    timeout: float = 180.0      # حداکثر زمان اجرا (3 دقیقه)


class FormationTransformer:
    """کلاس تبدیل آرایش‌ها"""
    
    def __init__(self, swarm_manager: SwarmManager):
        self.swarm = swarm_manager
        self.transformation_history = []
        
    async def apply_transformation(self, params: TransformationParameters) -> bool:
        """اعمال تبدیل به آرایش فعلی"""
        try:
            print(f"شروع تبدیل آرایش با پارامترها: {params}")
            
            # دریافت موقعیت‌های فعلی
            current_positions = await self._get_current_formation()
            if not current_positions:
                print("نمی‌توان موقعیت‌های فعلی را دریافت کرد")
                return False
                
            # محاسبه موقعیت‌های جدید
            new_positions = self._calculate_new_positions(current_positions, params)
            
            # اعتبارسنجی موقعیت‌های جدید
            if not self._validate_new_positions(new_positions):
                print("موقعیت‌های جدید نامعتبر هستند")
                return False
                
            # اجرای تبدیل
            success = await self._execute_transformation(new_positions, params)
            
            if success:
                self.transformation_history.append({
                    'timestamp': asyncio.get_event_loop().time(),
                    'parameters': params,
                    'old_positions': current_positions,
                    'new_positions': new_positions
                })
                self.swarm.log_formation_change()
                print("تبدیل آرایش با موفقیت انجام شد")
            else:
                print("خطا در اجرای تبدیل")
                
            return success
            
        except Exception as e:
            print(f"خطا در تبدیل آرایش: {str(e)}")
            return False
            
    async def _get_current_formation(self) -> Optional[Dict[int, Position]]:
        """دریافت موقعیت‌های فعلی پهپادها"""
        positions = {}
        active_drones = self.swarm.get_active_drones()
        
        for drone in active_drones:
            pos = await drone.get_current_position()
            if pos:
                positions[drone.id] = pos
            else:
                return None
                
        return positions
        
    def _calculate_new_positions(self, 
                               current_positions: Dict[int, Position], 
                               params: TransformationParameters) -> Dict[int, Position]:
        """محاسبه موقعیت‌های جدید بعد از تبدیل"""
        
        # یافتن مرکز آرایش فعلی
        center = self._calculate_formation_center(current_positions)
        
        new_positions = {}
        
        for drone_id, current_pos in current_positions.items():
            # تبدیل به مختصات نسبی نسبت به مرکز
            rel_x, rel_y = self._position_to_relative(current_pos, center)
            
            # اعمال دوران
            if params.rotate_z != 0:
                rel_x, rel_y = self._rotate_point(rel_x, rel_y, params.rotate_z)
                
            # اعمال تغییر مقیاس
            if params.scale_factor != 1.0:
                rel_x *= params.scale_factor
                rel_y *= params.scale_factor
                
            # تبدیل مجدد به مختصات جغرافیایی
            new_pos = self._relative_to_position(rel_x, rel_y, center)
            
            # اعمال جابه‌جایی
            new_pos = Position(
                latitude=new_pos.latitude + self._meters_to_lat(params.move_y),
                longitude=new_pos.longitude + self._meters_to_lon(params.move_x, center.latitude),
                altitude=new_pos.altitude + params.move_z
            )
            
            new_positions[drone_id] = new_pos
            
        return new_positions
        
    def _calculate_formation_center(self, positions: Dict[int, Position]) -> Position:
        """محاسبه مرکز آرایش"""
        if not positions:
            return Position(0, 0, 0)
            
        total_lat = sum(pos.latitude for pos in positions.values())
        total_lon = sum(pos.longitude for pos in positions.values()) 
        total_alt = sum(pos.altitude for pos in positions.values())
        
        count = len(positions)
        
        return Position(
            latitude=total_lat / count,
            longitude=total_lon / count, 
            altitude=total_alt / count
        )
        
    def _position_to_relative(self, position: Position, center: Position) -> Tuple[float, float]:
        """تبدیل موقعیت جغرافیایی به مختصات نسبی (متر)"""
        x = (position.longitude - center.longitude) * 111319.9 * math.cos(math.radians(center.latitude))
        y = (position.latitude - center.latitude) * 111319.9
        return x, y
        
    def _relative_to_position(self, x: float, y: float, center: Position) -> Position:
        """تبدیل مختصات نسبی به موقعیت جغرافیایی"""
        lat = center.latitude + y / 111319.9
        lon = center.longitude + x / (111319.9 * math.cos(math.radians(center.latitude)))
        return Position(lat, lon, center.altitude)
        
    def _rotate_point(self, x: float, y: float, angle_deg: float) -> Tuple[float, float]:
        """چرخش نقطه حول مبدأ"""
        angle_rad = math.radians(angle_deg)
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)
        
        new_x = x * cos_angle - y * sin_angle
        new_y = x * sin_angle + y * cos_angle
        
        return new_x, new_y
        
    def _meters_to_lat(self, meters: float) -> float:
        """تبدیل متر به درجه عرض جغرافیایی"""
        return meters / 111319.9
        
    def _meters_to_lon(self, meters: float, latitude: float) -> float:
        """تبدیل متر به درجه طول جغرافیایی"""
        return meters / (111319.9 * math.cos(math.radians(latitude)))
        
    def _validate_new_positions(self, positions: Dict[int, Position]) -> bool:
        """اعتبارسنجی موقعیت‌های جدید"""
        if not positions:
            return False
            
        # بررسی حداقل و حداکثر ارتفاع
        for pos in positions.values():
            if pos.altitude < self.swarm.config.min_altitude:
                print(f"ارتفاع {pos.altitude}m کمتر از حد مجاز {self.swarm.config.min_altitude}m")
                return False
            if pos.altitude > self.swarm.config.max_altitude:
                print(f"ارتفاع {pos.altitude}m بیشتر از حد مجاز {self.swarm.config.max_altitude}m") 
                return False
                
        # بررسی فاصله بین پهپادها
        drone_ids = list(positions.keys())
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                id1, id2 = drone_ids[i], drone_ids[j]
                distance = positions[id1].distance_to(positions[id2])
                
                if distance < self.swarm.config.safety_distance:
                    print(f"فاصله {distance:.2f}m بین پهپاد {id1} و {id2} کمتر از حد ایمن")
                    return False
                if distance > self.swarm.config.max_distance:
                    print(f"فاصله {distance:.2f}m بین پهپاد {id1} و {id2} بیشتر از حد مجاز")
                    return False
                    
        return True
        
    async def _execute_transformation(self, 
                                    new_positions: Dict[int, Position], 
                                    params: TransformationParameters) -> bool:
        """اجرای تبدیل با حرکت هماهنگ پهپادها"""
        try:
            active_drones = self.swarm.get_active_drones()
            
            # آماده‌سازی پهپادها برای حرکت
            for drone in active_drones:
                if drone.id in new_positions:
                    await drone.system.action.set_maximum_speed(params.max_speed)
                    
            # اجرای حرکت به صورت همزمان
            movement_tasks = []
            for drone in active_drones:
                if drone.id in new_positions:
                    target_pos = new_positions[drone.id]
                    task = self._move_drone_to_position(drone, target_pos, params.timeout)
                    movement_tasks.append(task)
                    
            # اجرای همزمان با تاخیر کوچک برای هماهنگی
            if params.coordination_delay > 0:
                await asyncio.sleep(params.coordination_delay)
                
            # انتظار برای اتمام همه حرکات
            results = await asyncio.gather(*movement_tasks, return_exceptions=True)
            
            # بررسی نتایج
            success_count = sum(1 for result in results if result is True)
            total_count = len(movement_tasks)
            
            print(f"حرکت {success_count}/{total_count} پهپاد با موفقیت انجام شد")
            
            return success_count >= total_count * 0.8  # حداقل 80% موفقیت
            
        except Exception as e:
            print(f"خطا در اجرای تبدیل: {str(e)}")
            return False
            
    async def _move_drone_to_position(self, 
                                    drone: Drone, 
                                    target: Position, 
                                    timeout: float) -> bool:
        """حرکت یک پهپاد به موقعیت هدف"""
        try:
            return await drone.goto_position(target)
        except Exception as e:
            print(f"خطا در حرکت پهپاد {drone.id}: {str(e)}")
            return False
            
    async def translate_formation(self, move_x: float, move_y: float, move_z: float = 0.0) -> bool:
        """جابه‌جایی ساده آرایش"""
        params = TransformationParameters(
            move_x=move_x,
            move_y=move_y, 
            move_z=move_z
        )
        return await self.apply_transformation(params)
        
    async def rotate_formation(self, axis: str, angle: float) -> bool:
        """دوران ساده آرایش حول محور مشخص"""
        params = TransformationParameters()
        
        if axis.lower() == 'x':
            params.rotate_x = angle
        elif axis.lower() == 'y':
            params.rotate_y = angle
        elif axis.lower() == 'z':
            params.rotate_z = angle
        else:
            print(f"محور نامعتبر: {axis}. از x، y یا z استفاده کنید")
            return False
            
        return await self.apply_transformation(params)
        
    async def scale_formation(self, factor: float) -> bool:
        """تغییر اندازه آرایش"""
        if factor <= 0:
            print("ضریب مقیاس باید مثبت باشد")
            return False
            
        params = TransformationParameters(scale_factor=factor)
        return await self.apply_transformation(params)
        
    async def combined_transformation(self, 
                                    move_x: float = 0, move_y: float = 0, move_z: float = 0,
                                    rotate_x: float = 0, rotate_y: float = 0, rotate_z: float = 0,
                                    scale: float = 1.0) -> bool:
        """تبدیل ترکیبی (جابه‌جایی + دوران + مقیاس)"""
        params = TransformationParameters(
            move_x=move_x, move_y=move_y, move_z=move_z,
            rotate_x=rotate_x, rotate_y=rotate_y, rotate_z=rotate_z,
            scale_factor=scale
        )
        return await self.apply_transformation(params)
        
    def get_transformation_history(self) -> List[Dict]:
        """دریافت تاریخچه تبدیلات"""
        return self.transformation_history.copy()
        
    async def undo_last_transformation(self) -> bool:
        """بازگردانی آخرین تبدیل"""
        if not self.transformation_history:
            print("هیچ تبدیلی برای بازگردانی وجود ندارد")
            return False
            
        try:
            last_transform = self.transformation_history[-1]
            old_positions = last_transform['old_positions']
            
            # حرکت پهپادها به موقعیت‌های قبلی
            movement_tasks = []
            for drone in self.swarm.get_active_drones():
                if drone.id in old_positions:
                    target_pos = old_positions[drone.id]
                    task = self._move_drone_to_position(drone, target_pos, 180.0)
                    movement_tasks.append(task)
                    
            results = await asyncio.gather(*movement_tasks, return_exceptions=True)
            success_count = sum(1 for result in results if result is True)
            
            if success_count >= len(movement_tasks) * 0.8:
                self.transformation_history.pop()  # حذف آخرین تبدیل از تاریخچه
                print("بازگردانی آخرین تبدیل با موفقیت انجام شد")
                return True
            else:
                print("خطا در بازگردانی تبدیل")
                return False
                
        except Exception as e:
            print(f"خطا در بازگردانی: {str(e)}")
            return False
            
    def calculate_formation_metrics(self, positions: Dict[int, Position]) -> Dict[str, float]:
        """محاسبه معیارهای آرایش"""
        if len(positions) < 2:
            return {}
            
        # محاسبه مرکز
        center = self._calculate_formation_center(positions)
        
        # محاسبه فاصله‌ها از مرکز
        distances_from_center = []
        for pos in positions.values():
            dist = pos.distance_to(center)
            distances_from_center.append(dist)
            
        # محاسبه فاصله‌های بین پهپادها
        inter_drone_distances = []
        drone_ids = list(positions.keys())
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                dist = positions[drone_ids[i]].distance_to(positions[drone_ids[j]])
                inter_drone_distances.append(dist)
                
        return {
            'center_lat': center.latitude,
            'center_lon': center.longitude,
            'center_alt': center.altitude,
            'avg_distance_from_center': sum(distances_from_center) / len(distances_from_center),
            'max_distance_from_center': max(distances_from_center) if distances_from_center else 0,
            'min_inter_drone_distance': min(inter_drone_distances) if inter_drone_distances else 0,
            'max_inter_drone_distance': max(inter_drone_distances) if inter_drone_distances else 0,
            'avg_inter_drone_distance': sum(inter_drone_distances) / len(inter_drone_distances) if inter_drone_distances else 0,
            'formation_spread': max(distances_from_center) - min(distances_from_center) if len(distances_from_center) > 1 else 0
        }


def parse_transformation_command(command_args: Dict) -> TransformationParameters:
    """تجزیه دستور تبدیل از CLI arguments"""
    return TransformationParameters(
        move_x=float(command_args.get('move_x', 0.0)),
        move_y=float(command_args.get('move_y', 0.0)),
        move_z=float(command_args.get('move_z', 0.0)),
        rotate_x=float(command_args.get('rotate_x', 0.0)),
        rotate_y=float(command_args.get('rotate_y', 0.0)),
        rotate_z=float(command_args.get('rotate_z', 0.0)),
        scale_factor=float(command_args.get('scale', 1.0)),
        max_speed=float(command_args.get('speed', 5.0)),
        timeout=float(command_args.get('timeout', 180.0))
    )