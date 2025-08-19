"""
ابزارهای کمکی و توابع عمومی
برای استفاده مشترک در سراسر پروژه
"""

import math
import time
import asyncio
from typing import List, Dict, Tuple, Optional, Any
import json
import csv


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    محاسبه فاصله بین دو نقطه جغرافیایی (متر)
    از فرمول haversine استفاده می‌کند
    """
    R = 6371000  # شعاع زمین به متر
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    a = (math.sin(delta_lat / 2) ** 2 + 
         math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


def bearing_between_points(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    محاسبه جهت (bearing) بین دو نقطه جغرافیایی (درجه)
    0 درجه = شمال، 90 درجه = شرق
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
    
    bearing = math.atan2(y, x)
    bearing_degrees = math.degrees(bearing)
    
    # تبدیل به 0-360 درجه
    return (bearing_degrees + 360) % 360


def meters_to_degrees_lat(meters: float) -> float:
    """تبدیل متر به درجه عرض جغرافیایی"""
    return meters / 111319.9


def meters_to_degrees_lon(meters: float, latitude: float) -> float:
    """تبدیل متر به درجه طول جغرافیایی"""
    return meters / (111319.9 * math.cos(math.radians(latitude)))


def degrees_to_meters_lat(degrees: float) -> float:
    """تبدیل درجه عرض جغرافیایی به متر"""
    return degrees * 111319.9


def degrees_to_meters_lon(degrees: float, latitude: float) -> float:
    """تبدیل درجه طول جغرافیایی به متر"""
    return degrees * 111319.9 * math.cos(math.radians(latitude))


def rotate_point_2d(x: float, y: float, angle_degrees: float) -> Tuple[float, float]:
    """چرخش نقطه دوبعدی حول مبدأ"""
    angle_rad = math.radians(angle_degrees)
    cos_angle = math.cos(angle_rad)
    sin_angle = math.sin(angle_rad)
    
    new_x = x * cos_angle - y * sin_angle
    new_y = x * sin_angle + y * cos_angle
    
    return new_x, new_y


def clamp(value: float, min_value: float, max_value: float) -> float:
    """محدود کردن مقدار به بازه مشخص"""
    return max(min_value, min(value, max_value))


def lerp(a: float, b: float, t: float) -> float:
    """درون‌یابی خطی بین دو مقدار"""
    return a + t * (b - a)


def calculate_centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    """محاسبه مرکز هندسی مجموعه نقاط"""
    if not points:
        return 0.0, 0.0
        
    x_sum = sum(point[0] for point in points)
    y_sum = sum(point[1] for point in points)
    
    return x_sum / len(points), y_sum / len(points)


def is_point_in_circle(point_x: float, point_y: float, 
                      circle_x: float, circle_y: float, radius: float) -> bool:
    """بررسی قرار گیری نقطه در دایره"""
    distance = math.sqrt((point_x - circle_x)**2 + (point_y - circle_y)**2)
    return distance <= radius


def normalize_angle(angle_degrees: float) -> float:
    """نرمال‌سازی زاویه به بازه 0-360 درجه"""
    return angle_degrees % 360


def angle_difference(angle1: float, angle2: float) -> float:
    """محاسبه کوتاه‌ترین فاصله زاویه‌ای بین دو زاویه"""
    diff = angle2 - angle1
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


class Timer:
    """کلاس ساده برای اندازه‌گیری زمان"""
    
    def __init__(self):
        self.start_time = None
        self.end_time = None
        
    def start(self):
        """شروع تایمر"""
        self.start_time = time.time()
        
    def stop(self):
        """توقف تایمر"""
        self.end_time = time.time()
        
    def elapsed(self) -> float:
        """مدت زمان سپری شده (ثانیه)"""
        if self.start_time is None:
            return 0.0
        end = self.end_time or time.time()
        return end - self.start_time
        
    def __enter__(self):
        self.start()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


class RateLimiter:
    """کلاس محدود کردن نرخ اجرا"""
    
    def __init__(self, max_calls: int, time_window: float):
        self.max_calls = max_calls
        self.time_window = time_window
        self.calls = []
        
    def can_proceed(self) -> bool:
        """بررسی امکان اجرای عمل بعدی"""
        current_time = time.time()
        
        # حذف فراخوانی‌های قدیمی
        self.calls = [call_time for call_time in self.calls 
                     if current_time - call_time < self.time_window]
        
        if len(self.calls) < self.max_calls:
            self.calls.append(current_time)
            return True
        return False
        
    async def wait_if_needed(self):
        """انتظار در صورت رسیدن به حد نرخ"""
        while not self.can_proceed():
            await asyncio.sleep(0.1)


class MovingAverage:
    """کلاس محاسبه میانگین متحرک"""
    
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = []
        
    def add(self, value: float):
        """اضافه کردن مقدار جدید"""
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
            
    def average(self) -> float:
        """محاسبه میانگین"""
        return sum(self.values) / len(self.values) if self.values else 0.0
        
    def reset(self):
        """ریست کردن داده‌ها"""
        self.values.clear()


def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    """تقسیم ایمن با مقدار پیش‌فرض در صورت صفر بودن مخرج"""
    return numerator / denominator if denominator != 0 else default


def format_duration(seconds: float) -> str:
    """فرمت کردن مدت زمان به رشته خوانا"""
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.1f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = seconds % 60
        return f"{hours}h {minutes}m {secs:.1f}s"


def format_distance(meters: float) -> str:
    """فرمت کردن فاصله به رشته خوانا"""
    if meters < 1000:
        return f"{meters:.1f}m"
    else:
        kilometers = meters / 1000
        return f"{kilometers:.2f}km"


def validate_coordinates(lat: float, lon: float) -> bool:
    """اعتبارسنجی مختصات جغرافیایی"""
    return -90 <= lat <= 90 and -180 <= lon <= 180


def validate_altitude(altitude: float, min_alt: float = 0, max_alt: float = 500) -> bool:
    """اعتبارسنجی ارتفاع"""
    return min_alt <= altitude <= max_alt


async def retry_async(func, max_attempts: int = 3, delay: float = 1.0, backoff: float = 2.0):
    """تلاش مجدد اجرای تابع async در صورت خطا"""
    for attempt in range(max_attempts):
        try:
            return await func()
        except Exception as e:
            if attempt == max_attempts - 1:
                raise e
            await asyncio.sleep(delay * (backoff ** attempt))


def load_json_config(filename: str, default: Dict = None) -> Dict:
    """بارگذاری فایل تنظیمات JSON"""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        if default is not None:
            return default.copy()
        return {}
    except json.JSONDecodeError as e:
        print(f"خطا در تجزیه فایل JSON {filename}: {str(e)}")
        return default.copy() if default else {}


def save_json_config(data: Dict, filename: str):
    """ذخیره داده‌ها در فایل JSON"""
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print(f"خطا در ذخیره فایل JSON {filename}: {str(e)}")


def load_csv_data(filename: str) -> List[Dict]:
    """بارگذاری داده‌های CSV"""
    try:
        data = []
        with open(filename, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data.append(row)
        return data
    except FileNotFoundError:
        print(f"فایل {filename} یافت نشد")
        return []
    except Exception as e:
        print(f"خطا در بارگذاری CSV {filename}: {str(e)}")
        return []


def save_csv_data(data: List[Dict], filename: str, fieldnames: List[str] = None):
    """ذخیره داده‌ها در فایل CSV"""
    try:
        if not data:
            return
            
        if fieldnames is None:
            fieldnames = list(data[0].keys())
            
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(data)
            
    except Exception as e:
        print(f"خطا در ذخیره CSV {filename}: {str(e)}")


class PerformanceMonitor:
    """کلاس نظارت بر عملکرد"""
    
    def __init__(self):
        self.metrics = {}
        self.start_times = {}
        
    def start_timing(self, operation: str):
        """شروع اندازه‌گیری زمان عملیات"""
        self.start_times[operation] = time.time()
        
    def end_timing(self, operation: str):
        """پایان اندازه‌گیری زمان عملیات"""
        if operation in self.start_times:
            duration = time.time() - self.start_times[operation]
            
            if operation not in self.metrics:
                self.metrics[operation] = []
            self.metrics[operation].append(duration)
            
            del self.start_times[operation]
            return duration
        return 0.0
        
    def get_stats(self, operation: str) -> Dict[str, float]:
        """دریافت آمار عملکرد"""
        if operation not in self.metrics or not self.metrics[operation]:
            return {}
            
        times = self.metrics[operation]
        return {
            'count': len(times),
            'total': sum(times),
            'average': sum(times) / len(times),
            'min': min(times),
            'max': max(times)
        }
        
    def get_all_stats(self) -> Dict[str, Dict[str, float]]:
        """دریافت آمار تمام عملیات"""
        return {op: self.get_stats(op) for op in self.metrics.keys()}
        
    def reset(self):
        """ریست آمار"""
        self.metrics.clear()
        self.start_times.clear()


# نمونه‌ای از monitor سراسری
perf_monitor = PerformanceMonitor()


def log_performance(operation_name: str):
    """دکوریتر برای اندازه‌گیری خودکار عملکرد"""
    def decorator(func):
        if asyncio.iscoroutinefunction(func):
            async def async_wrapper(*args, **kwargs):
                perf_monitor.start_timing(operation_name)
                try:
                    result = await func(*args, **kwargs)
                    return result
                finally:
                    perf_monitor.end_timing(operation_name)
            return async_wrapper
        else:
            def sync_wrapper(*args, **kwargs):
                perf_monitor.start_timing(operation_name)
                try:
                    result = func(*args, **kwargs)
                    return result
                finally:
                    perf_monitor.end_timing(operation_name)
            return sync_wrapper
    return decorator


# توابع کمکی برای debugging
def print_dict_formatted(data: Dict, title: str = "Data"):
    """نمایش فرمت شده dictionary"""
    print(f"\n=== {title} ===")
    for key, value in data.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    print("=" * (len(title) + 8))


def memory_usage():
    """دریافت میزان استفاده از حافظه (اگر psutil موجود باشد)"""
    try:
        import psutil
        import os
        
        process = psutil.Process(os.getpid())
        memory_info = process.memory_info()
        
        return {
            'rss_mb': memory_info.rss / 1024 / 1024,  # حافظه فیزیکی
            'vms_mb': memory_info.vms / 1024 / 1024,  # حافظه مجازی
            'cpu_percent': process.cpu_percent()
        }
    except ImportError:
        return {'error': 'psutil not available'}


# ثوابت مفید
EARTH_RADIUS_M = 6371000  # شعاع زمین به متر
KNOTS_TO_MS = 0.514444    # تبدیل knot به m/s
MS_TO_KNOTS = 1.943844    # تبدیل m/s به knot
FEET_TO_METERS = 0.3048   # تبدیل فوت به متر
METERS_TO_FEET = 3.28084  # تبدیل متر به فوت