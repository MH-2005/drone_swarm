"""
مدیریت مسیر رهبر از فایل CSV
مطابق با الزامات مرحله سوم شیوه‌نامه
"""

import asyncio
import csv
import time
import math
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

from ..core.drone import Drone, Position


class PathStatus(Enum):
    """وضعیت اجرای مسیر"""
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    PAUSED = "paused"
    ERROR = "error"


@dataclass
class PathPoint:
    """نقطه‌ای از مسیر"""
    index: int
    latitude: float
    longitude: float
    altitude: float
    timestamp: float = 0.0
    formation_type: Optional[str] = None
    formation_size: Optional[float] = None
    speed: Optional[float] = None
    wait_time: float = 0.0  # زمان انتظار در این نقطه (ثانیه)


@dataclass
class PathMetrics:
    """معیارهای مسیر"""
    total_distance: float = 0.0
    estimated_duration: float = 0.0
    actual_duration: float = 0.0
    points_visited: int = 0
    current_point_index: int = 0
    average_speed: float = 0.0


class PathManager:
    """مدیر اجرای مسیر"""
    
    def __init__(self, csv_file_path: str = "leader_path.csv"):
        self.csv_file = csv_file_path
        self.path_points: List[PathPoint] = []
        self.current_drone: Optional[Drone] = None
        
        # وضعیت اجرا
        self.status = PathStatus.NOT_STARTED
        self.current_point_index = 0
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        
        # تنظیمات
        self.default_speed = 5.0  # m/s
        self.position_tolerance = 2.0  # متر
        self.max_wait_at_point = 10.0  # ثانیه
        
        # آمار
        self.metrics = PathMetrics()
        self.visited_points = []
        
    def load_path_from_csv(self, csv_file: str = None) -> bool:
        """بارگذاری مسیر از فایل CSV"""
        try:
            file_path = csv_file or self.csv_file
            print(f"بارگذاری مسیر از {file_path}")
            
            self.path_points.clear()
            
            with open(file_path, 'r', encoding='utf-8') as file:
                reader = csv.DictReader(file)
                
                for index, row in enumerate(reader):
                    point = self._parse_csv_row(index, row)
                    if point:
                        self.path_points.append(point)
                        
            if len(self.path_points) < 3:
                print(f"⚠️ مسیر باید حداقل 3 نقطه داشته باشد. تعداد فعلی: {len(self.path_points)}")
                return False
                
            # محاسبه معیارهای اولیه
            self._calculate_path_metrics()
            
            print(f"✅ {len(self.path_points)} نقطه مسیر بارگذاری شد")
            print(f"   مسافت کل: {self.metrics.total_distance:.2f}m")
            print(f"   زمان تخمینی: {self.metrics.estimated_duration:.2f}s")
            
            return True
            
        except FileNotFoundError:
            print(f"❌ فایل مسیر {file_path} یافت نشد")
            return False
        except Exception as e:
            print(f"❌ خطا در بارگذاری مسیر: {str(e)}")
            return False
            
    def _parse_csv_row(self, index: int, row: Dict[str, str]) -> Optional[PathPoint]:
        """تجزیه یک سطر از CSV"""
        try:
            # فیلدهای اجباری
            lat = float(row.get('latitude', row.get('lat', 0)))
            lon = float(row.get('longitude', row.get('lon', row.get('lng', 0))))
            alt = float(row.get('altitude', row.get('alt', 20.0)))
            
            # فیلدهای اختیاری
            timestamp = float(row.get('timestamp', row.get('time', 0.0)))
            formation_type = row.get('formation_type', row.get('formation'))
            formation_size = row.get('formation_size', row.get('size'))
            speed = row.get('speed', row.get('velocity'))
            wait_time = float(row.get('wait_time', row.get('wait', 0.0)))
            
            # اعتبارسنجی
            if abs(lat) > 90 or abs(lon) > 180:
                print(f"⚠️ مختصات نامعتبر در سطر {index + 1}: {lat}, {lon}")
                return None
                
            if alt < 5 or alt > 100:
                print(f"⚠️ ارتفاع نامعتبر در سطر {index + 1}: {alt}m")
                alt = max(5, min(100, alt))  # اصلاح خودکار
                
            return PathPoint(
                index=index,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                timestamp=timestamp,
                formation_type=formation_type.strip() if formation_type else None,
                formation_size=float(formation_size) if formation_size else None,
                speed=float(speed) if speed else None,
                wait_time=wait_time
            )
            
        except (ValueError, KeyError) as e:
            print(f"❌ خطا در تجزیه سطر {index + 1}: {str(e)}")
            return None
            
    def _calculate_path_metrics(self):
        """محاسبه معیارهای مسیر"""
        if len(self.path_points) < 2:
            return
            
        total_distance = 0.0
        estimated_time = 0.0
        
        for i in range(1, len(self.path_points)):
            prev_point = self.path_points[i-1]
            curr_point = self.path_points[i]
            
            # محاسبه فاصله بین نقاط
            prev_pos = Position(prev_point.latitude, prev_point.longitude, prev_point.altitude)
            curr_pos = Position(curr_point.latitude, curr_point.longitude, curr_point.altitude)
            
            segment_distance = prev_pos.distance_to(curr_pos)
            total_distance += segment_distance
            
            # محاسبه زمان تخمینی
            speed = curr_point.speed or self.default_speed
            segment_time = segment_distance / speed
            estimated_time += segment_time + curr_point.wait_time
            
        self.metrics.total_distance = total_distance
        self.metrics.estimated_duration = estimated_time
        
    def set_current_drone(self, drone: Drone):
        """تنظیم پهپاد فعلی برای اجرای مسیر"""
        self.current_drone = drone
        print(f"پهپاد {drone.id} برای اجرای مسیر تنظیم شد")
        
    async def execute_path(self) -> bool:
        """اجرای کامل مسیر"""
        if not self.current_drone:
            print("❌ پهپادی برای اجرای مسیر تنظیم نشده")
            return False
            
        if not self.path_points:
            print("❌ مسیری برای اجرا وجود ندارد")
            return False
            
        try:
            print(f"🚀 شروع اجرای مسیر با {len(self.path_points)} نقطه")
            
            self.status = PathStatus.IN_PROGRESS
            self.start_time = time.time()
            self.current_point_index = 0
            self.visited_points.clear()
            
            # اجرای نقطه به نقطه
            for i, point in enumerate(self.path_points):
                self.current_point_index = i
                
                # بررسی زندگی پهپاد
                if not self.current_drone.is_alive():
                    print("❌ پهپاد رهبر از کار افتاده")
                    self.status = PathStatus.ERROR
                    return False
                    
                print(f"📍 حرکت به نقطه {i+1}/{len(self.path_points)}")
                
                # حرکت به نقطه
                success = await self._move_to_point(point)
                if not success:
                    print(f"❌ خطا در رسیدن به نقطه {i+1}")
                    self.status = PathStatus.ERROR
                    return False
                    
                # ثبت نقطه بازدید شده
                self.visited_points.append({
                    'point': point,
                    'arrival_time': time.time(),
                    'actual_position': await self.current_drone.get_current_position()
                })
                
                self.metrics.points_visited += 1
                
                # انتظار در صورت نیاز
                if point.wait_time > 0:
                    print(f"⏳ انتظار {point.wait_time} ثانیه در نقطه {i+1}")
                    await asyncio.sleep(point.wait_time)
                    
            # پایان موفق مسیر
            self.end_time = time.time()
            self.status = PathStatus.COMPLETED
            self.metrics.actual_duration = self.end_time - self.start_time
            
            if self.metrics.actual_duration > 0:
                self.metrics.average_speed = self.metrics.total_distance / self.metrics.actual_duration
                
            print(f"✅ مسیر با موفقیت تکمیل شد در {self.metrics.actual_duration:.2f} ثانیه")
            return True
            
        except Exception as e:
            print(f"❌ خطا در اجرای مسیر: {str(e)}")
            self.status = PathStatus.ERROR
            return False
            
    async def _move_to_point(self, point: PathPoint) -> bool:
        """حرکت به یک نقطه مشخص"""
        try:
            target_position = Position(point.latitude, point.longitude, point.altitude)
            
            # تنظیم سرعت
            speed = point.speed or self.default_speed
            
            # حرکت به نقطه
            success = await self.current_drone.goto_position(target_position, speed)
            
            if success:
                # انتظار برای رسیدن دقیق
                await self._wait_for_arrival(target_position)
                
            return success
            
        except Exception as e:
            print(f"خطا در حرکت به نقطه: {str(e)}")
            return False
            
    async def _wait_for_arrival(self, target: Position, timeout: float = 30.0):
        """انتظار برای رسیدن دقیق به نقطه هدف"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            current_pos = await self.current_drone.get_current_position()
            if current_pos:
                distance = current_pos.distance_to(target)
                if distance <= self.position_tolerance:
                    break
                    
            await asyncio.sleep(0.5)
            
    async def pause_path(self):
        """توقف موقت مسیر"""
        if self.status == PathStatus.IN_PROGRESS:
            self.status = PathStatus.PAUSED
            print("⏸️ مسیر موقتاً متوقف شد")
            
    async def resume_path(self):
        """ادامه مسیر"""
        if self.status == PathStatus.PAUSED:
            self.status = PathStatus.IN_PROGRESS
            print("▶️ مسیر ادامه یافت")
            
    async def abort_path(self):
        """لغو مسیر"""
        self.status = PathStatus.ERROR
        print("🛑 مسیر لغو شد")
        
    def get_current_target(self) -> Optional[PathPoint]:
        """دریافت نقطه هدف فعلی"""
        if 0 <= self.current_point_index < len(self.path_points):
            return self.path_points[self.current_point_index]
        return None
        
    def get_next_target(self) -> Optional[PathPoint]:
        """دریافت نقطه هدف بعدی"""
        next_index = self.current_point_index + 1
        if next_index < len(self.path_points):
            return self.path_points[next_index]
        return None
        
    def get_progress_percentage(self) -> float:
        """درصد پیشرفت مسیر"""
        if not self.path_points:
            return 0.0
            
        return (self.current_point_index / len(self.path_points)) * 100
        
    def get_remaining_distance(self) -> float:
        """مسافت باقی‌مانده مسیر"""
        if self.current_point_index >= len(self.path_points) - 1:
            return 0.0
            
        remaining_distance = 0.0
        
        for i in range(self.current_point_index, len(self.path_points) - 1):
            curr_point = self.path_points[i]
            next_point = self.path_points[i + 1]
            
            curr_pos = Position(curr_point.latitude, curr_point.longitude, curr_point.altitude)
            next_pos = Position(next_point.latitude, next_point.longitude, next_point.altitude)
            
            remaining_distance += curr_pos.distance_to(next_pos)
            
        return remaining_distance
        
    def get_path_status(self) -> Dict:
        """وضعیت کامل مسیر"""
        return {
            'status': self.status.value,
            'total_points': len(self.path_points),
            'current_point': self.current_point_index + 1,
            'progress_percentage': round(self.get_progress_percentage(), 1),
            'points_visited': self.metrics.points_visited,
            'total_distance': round(self.metrics.total_distance, 2),
            'remaining_distance': round(self.get_remaining_distance(), 2),
            'estimated_duration': round(self.metrics.estimated_duration, 2),
            'actual_duration': round(self.metrics.actual_duration, 2) if self.metrics.actual_duration > 0 else None,
            'average_speed': round(self.metrics.average_speed, 2) if self.metrics.average_speed > 0 else None,
            'start_time': self.start_time,
            'end_time': self.end_time
        }
        
    def export_visited_points(self, filename: str = "visited_points.csv"):
        """صادرات نقاط بازدید شده"""
        try:
            with open(filename, 'w', newline='', encoding='utf-8') as file:
                fieldnames = ['point_index', 'latitude', 'longitude', 'altitude', 
                            'arrival_time', 'actual_lat', 'actual_lon', 'actual_alt', 
                            'position_error']
                            
                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()
                
                for visit in self.visited_points:
                    point = visit['point']
                    actual_pos = visit['actual_position']
                    
                    target_pos = Position(point.latitude, point.longitude, point.altitude)
                    position_error = actual_pos.distance_to(target_pos) if actual_pos else 0
                    
                    writer.writerow({
                        'point_index': point.index + 1,
                        'latitude': point.latitude,
                        'longitude': point.longitude,
                        'altitude': point.altitude,
                        'arrival_time': visit['arrival_time'],
                        'actual_lat': actual_pos.latitude if actual_pos else None,
                        'actual_lon': actual_pos.longitude if actual_pos else None,
                        'actual_alt': actual_pos.altitude if actual_pos else None,
                        'position_error': round(position_error, 2)
                    })
                    
            print(f"✅ نقاط بازدید شده در {filename} ذخیره شد")
            return True
            
        except Exception as e:
            print(f"❌ خطا در صادرات: {str(e)}")
            return False


def create_sample_path(filename: str = "leader_path.csv", 
                      center_lat: float = -35.363261, 
                      center_lon: float = 149.165230) -> bool:
    """ایجاد مسیر نمونه"""
    try:
        # مسیر نمونه: مستطیل + تغییرات آرایش
        sample_points = [
            # شروع
            {
                'latitude': center_lat,
                'longitude': center_lon,
                'altitude': 20.0,
                'timestamp': 0.0,
                'formation_type': 'triangle',
                'formation_size': 8.0,
                'speed': 5.0,
                'wait_time': 2.0
            },
            # گوشه شمال شرقی
            {
                'latitude': center_lat + 0.0005,
                'longitude': center_lon + 0.0005,
                'altitude': 25.0,
                'timestamp': 30.0,
                'formation_type': 'triangle',
                'formation_size': 8.0,
                'speed': 4.0,
                'wait_time': 3.0
            },
            # گوشه شمال غربی
            {
                'latitude': center_lat + 0.0005,
                'longitude': center_lon - 0.0005,
                'altitude': 30.0,
                'timestamp': 60.0,
                'formation_type': 'square',
                'formation_size': 10.0,
                'speed': 6.0,
                'wait_time': 2.0
            },
            # گوشه جنوب غربی
            {
                'latitude': center_lat - 0.0005,
                'longitude': center_lon - 0.0005,
                'altitude': 25.0,
                'timestamp': 90.0,
                'formation_type': 'line',
                'formation_size': 12.0,
                'speed': 5.0,
                'wait_time': 3.0
            },
            # بازگشت به مرکز
            {
                'latitude': center_lat,
                'longitude': center_lon,
                'altitude': 20.0,
                'timestamp': 120.0,
                'formation_type': 'triangle',
                'formation_size': 8.0,
                'speed': 4.0,
                'wait_time': 5.0
            }
        ]
        
        with open(filename, 'w', newline='', encoding='utf-8') as file:
            fieldnames = ['latitude', 'longitude', 'altitude', 'timestamp', 
                         'formation_type', 'formation_size', 'speed', 'wait_time']
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            
            writer.writeheader()
            for point in sample_points:
                writer.writerow(point)
                
        print(f"✅ مسیر نمونه در {filename} ایجاد شد")
        return True
        
    except Exception as e:
        print(f"❌ خطا در ایجاد مسیر نمونه: {str(e)}")
        return False