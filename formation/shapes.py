"""
تعریف شکل‌های مختلف آرایش پهپادها
مطابق با الزامات مرحله اول شیوه‌نامه
"""

import math
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from ..core.drone import Position


class FormationType(Enum):
    """انواع آرایش‌ها"""
    LINE = "line"
    TRIANGLE = "triangle" 
    SQUARE = "square"
    DIAMOND = "diamond"
    CIRCLE = "circle"


class Orientation(Enum):
    """جهت‌گیری آرایش"""
    HORIZONTAL = "horizontal"
    VERTICAL = "vertical"


@dataclass
class FormationParameters:
    """پارامترهای آرایش"""
    formation_type: FormationType
    size: float  # اندازه آرایش (متر)
    altitude: float  # ارتفاع پرواز (متر)
    orientation: Orientation = Orientation.HORIZONTAL
    center_position: Optional[Position] = None
    rotation_angle: float = 0.0  # زاویه دوران (درجه)


class FormationGenerator:
    """ژنراتور شکل‌های آرایش"""
    
    def __init__(self):
        self.formation_patterns = {
            FormationType.LINE: self._generate_line_formation,
            FormationType.TRIANGLE: self._generate_triangle_formation,
            FormationType.SQUARE: self._generate_square_formation,
            FormationType.DIAMOND: self._generate_diamond_formation,
            FormationType.CIRCLE: self._generate_circle_formation
        }
        
    def generate_formation_positions(self, 
                                   num_drones: int, 
                                   params: FormationParameters) -> List[Position]:
        """تولید موقعیت‌های آرایش برای تعداد مشخص پهپاد"""
        
        if params.formation_type not in self.formation_patterns:
            raise ValueError(f"نوع آرایش نامعتبر: {params.formation_type}")
            
        # تولید موقعیت‌های نسبی
        relative_positions = self.formation_patterns[params.formation_type](num_drones, params)
        
        # اعمال تغییرات
        if params.orientation == Orientation.VERTICAL:
            relative_positions = self._make_vertical(relative_positions)
            
        if params.rotation_angle != 0:
            relative_positions = self._rotate_positions(relative_positions, params.rotation_angle)
            
        # تبدیل به موقعیت‌های جغرافیایی
        if params.center_position:
            absolute_positions = self._convert_to_geographic(
                relative_positions, params.center_position, params.altitude
            )
        else:
            # استفاده از موقعیت پیش‌فرض
            default_center = Position(latitude=-35.363261, longitude=149.165230, altitude=params.altitude)
            absolute_positions = self._convert_to_geographic(
                relative_positions, default_center, params.altitude
            )
            
        return absolute_positions
        
    def _generate_line_formation(self, num_drones: int, params: FormationParameters) -> List[Tuple[float, float]]:
        """تولید آرایش خطی"""
        positions = []
        spacing = params.size / max(1, num_drones - 1) if num_drones > 1 else 0
        start_x = -params.size / 2
        
        for i in range(num_drones):
            x = start_x + i * spacing
            y = 0.0
            positions.append((x, y))
            
        return positions
        
    def _generate_triangle_formation(self, num_drones: int, params: FormationParameters) -> List[Tuple[float, float]]:
        """تولید آرایش مثلثی"""
        positions = []
        
        if num_drones <= 0:
            return positions
            
        # اولین پهپاد در راس مثلث
        positions.append((0.0, params.size / 2))
        
        if num_drones == 1:
            return positions
            
        # پهپادهای باقی‌مانده در قاعده
        remaining = num_drones - 1
        if remaining == 1:
            positions.append((0.0, -params.size / 2))
        else:
            base_spacing = params.size / (remaining - 1) if remaining > 1 else 0
            start_x = -params.size / 2
            
            for i in range(remaining):
                x = start_x + i * base_spacing
                y = -params.size / 2
                positions.append((x, y))
                
        return positions
        
    def _generate_square_formation(self, num_drones: int, params: FormationParameters) -> List[Tuple[float, float]]:
        """تولید آرایش مربعی"""
        positions = []
        half_size = params.size / 2
        
        if num_drones <= 0:
            return positions
            
        # موقعیت‌های گوشه‌ها
        corners = [
            (-half_size, half_size),   # بالا چپ
            (half_size, half_size),    # بالا راست  
            (half_size, -half_size),   # پایین راست
            (-half_size, -half_size)   # پایین چپ
        ]
        
        # قرار دادن پهپادها در گوشه‌ها
        for i in range(min(num_drones, 4)):
            positions.append(corners[i])
            
        # اگر پهپاد بیشتری داریم، در وسط اضلاع قرار می‌دهیم
        if num_drones > 4:
            edges = [
                (0.0, half_size),      # بالا وسط
                (half_size, 0.0),      # راست وسط
                (0.0, -half_size),     # پایین وسط
                (-half_size, 0.0)      # چپ وسط
            ]
            
            for i in range(min(num_drones - 4, 4)):
                positions.append(edges[i])
                
        # اگر همچنان پهپاد اضافی داریم، در مرکز قرار می‌دهیم
        if num_drones > 8:
            remaining = num_drones - 8
            for i in range(remaining):
                # توزیع تصادفی در مرکز
                radius = params.size * 0.2
                angle = (2 * math.pi * i) / remaining
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                positions.append((x, y))
                
        return positions
        
    def _generate_diamond_formation(self, num_drones: int, params: FormationParameters) -> List[Tuple[float, float]]:
        """تولید آرایش الماسی"""
        positions = []
        half_size = params.size / 2
        
        if num_drones <= 0:
            return positions
            
        # موقعیت‌های اصلی الماس
        diamond_points = [
            (0.0, half_size),          # بالا
            (half_size, 0.0),          # راست
            (0.0, -half_size),         # پایین
            (-half_size, 0.0)          # چپ
        ]
        
        for i in range(min(num_drones, 4)):
            positions.append(diamond_points[i])
            
        # پهپادهای اضافی در مرکز
        if num_drones > 4:
            positions.append((0.0, 0.0))  # مرکز
            
        return positions
        
    def _generate_circle_formation(self, num_drones: int, params: FormationParameters) -> List[Tuple[float, float]]:
        """تولید آرایش دایره‌ای"""
        positions = []
        radius = params.size / 2
        
        if num_drones <= 0:
            return positions
            
        if num_drones == 1:
            positions.append((0.0, 0.0))
            return positions
            
        # توزیع یکنواخت روی دایره
        for i in range(num_drones):
            angle = (2 * math.pi * i) / num_drones
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            positions.append((x, y))
            
        return positions
        
    def _make_vertical(self, positions: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """تبدیل آرایش افقی به عمودی"""
        # چرخش 90 درجه: (x, y) -> (-y, x)
        return [(-y, x) for x, y in positions]
        
    def _rotate_positions(self, positions: List[Tuple[float, float]], angle_deg: float) -> List[Tuple[float, float]]:
        """چرخش موقعیت‌ها به اندازه زاویه مشخص"""
        angle_rad = math.radians(angle_deg)
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)
        
        rotated = []
        for x, y in positions:
            new_x = x * cos_angle - y * sin_angle
            new_y = x * sin_angle + y * cos_angle
            rotated.append((new_x, new_y))
            
        return rotated
        
    def _convert_to_geographic(self, 
                             relative_positions: List[Tuple[float, float]], 
                             center: Position, 
                             altitude: float) -> List[Position]:
        """تبدیل موقعیت‌های نسبی به مختصات جغرافیایی"""
        positions = []
        
        # ضرایب تبدیل متر به درجه (تقریبی)
        lat_per_meter = 1.0 / 111319.9  # درجه بر متر
        lon_per_meter = 1.0 / (111319.9 * math.cos(math.radians(center.latitude)))
        
        for x, y in relative_positions:
            # x -> تغییر طول جغرافیایی (شرق-غرب)
            # y -> تغییر عرض جغرافیایی (شمال-جنوب)
            new_lat = center.latitude + y * lat_per_meter
            new_lon = center.longitude + x * lon_per_meter
            
            positions.append(Position(new_lat, new_lon, altitude))
            
        return positions
        
    def validate_formation_parameters(self, params: FormationParameters, num_drones: int) -> bool:
        """اعتبارسنجی پارامترهای آرایش"""
        if params.size <= 0:
            return False
            
        if params.altitude < 5.0 or params.altitude > 100.0:
            return False
            
        if num_drones <= 0:
            return False
            
        # بررسی سازگاری نوع آرایش با تعداد پهپاد
        if params.formation_type == FormationType.TRIANGLE and num_drones < 2:
            return False
            
        return True
        
    def get_formation_info(self, formation_type: FormationType) -> Dict:
        """دریافت اطلاعات آرایش"""
        info = {
            FormationType.LINE: {
                "name": "خطی",
                "min_drones": 1,
                "description": "پهپادها در یک خط مستقیم"
            },
            FormationType.TRIANGLE: {
                "name": "مثلثی", 
                "min_drones": 2,
                "description": "آرایش مثلثی با رهبر در راس"
            },
            FormationType.SQUARE: {
                "name": "مربعی",
                "min_drones": 1,
                "description": "آرایش مربعی یا مستطیلی"
            },
            FormationType.DIAMOND: {
                "name": "الماسی",
                "min_drones": 1, 
                "description": "آرایش الماسی شکل"
            },
            FormationType.CIRCLE: {
                "name": "دایره‌ای",
                "min_drones": 1,
                "description": "آرایش دایره‌ای"
            }
        }
        
        return info.get(formation_type, {"name": "نامشخص", "min_drones": 1, "description": ""})


def parse_formation_command(command_args: Dict) -> FormationParameters:
    """تجزیه دستور آرایش از CLI arguments"""
    formation_type_map = {
        'line': FormationType.LINE,
        'triangle': FormationType.TRIANGLE,
        'square': FormationType.SQUARE, 
        'diamond': FormationType.DIAMOND,
        'circle': FormationType.CIRCLE
    }
    
    orientation_map = {
        'horizontal': Orientation.HORIZONTAL,
        'vertical': Orientation.VERTICAL
    }
    
    formation_type = formation_type_map.get(
        command_args.get('formation', 'square'), 
        FormationType.SQUARE
    )
    
    orientation = orientation_map.get(
        command_args.get('orientation', 'horizontal'),
        Orientation.HORIZONTAL  
    )
    
    return FormationParameters(
        formation_type=formation_type,
        size=float(command_args.get('size', 10.0)),
        altitude=float(command_args.get('altitude', 20.0)),
        orientation=orientation,
        rotation_angle=float(command_args.get('rotation', 0.0))
    )