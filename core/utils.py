"""
ماژول ابزارهای مشترک و توابع کمکی
توابعی که در بخش‌های مختلف پروژه کاربرد دارند در اینجا قرار می‌گیرند.
"""

import math
from typing import Tuple

from .drone import Position

def geographic_to_cartesian(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """
    تبدیل مختصات جغرافیایی (Latitude, Longitude, Altitude) به دکارتی (x, y, z).
    این تبدیل برای محاسبات ساده‌تر در فضای سه‌بعدی مفید است.
    """
    R = 6371000  # شعاع تقریبی زمین بر حسب متر
    x = R * math.cos(math.radians(lat)) * math.cos(math.radians(lon))
    y = R * math.cos(math.radians(lat)) * math.sin(math.radians(lon))
    z = alt  # برای سادگی، ارتفاع را معادل محور z در نظر می‌گیریم
    return x, y, z

def calculate_heading(from_pos: Position, to_pos: Position) -> float:
    """
    محاسبه جهت (bearing/heading) از یک نقطه به نقطه دیگر بر حسب درجه.
    این تابع مشخص می‌کند که برای رفتن از نقطه اول به دوم، باید به کدام سمت (شمال، شرق و...) حرکت کرد.
    """
    lat1, lon1 = math.radians(from_pos.latitude), math.radians(from_pos.longitude)
    lat2, lon2 = math.radians(to_pos.latitude), math.radians(to_pos.longitude)

    dLon = lon2 - lon1
    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    
    # نرمال‌سازی به بازه ۰ تا ۳۶۰ درجه
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def meters_to_lat_lon_diff(meters_x: float, meters_y: float, current_lat: float) -> Tuple[float, float]:
    """
    تبدیل جابه‌جایی بر حسب متر (در جهت شرق-غرب و شمال-جنوب) به تفاوت درجه در طول و عرض جغرافیایی.
    """
    lat_diff = meters_y / 111319.9
    lon_diff = meters_x / (111319.9 * math.cos(math.radians(current_lat)))
    return lat_diff, lon_diff