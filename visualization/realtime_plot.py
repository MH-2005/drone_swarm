"""
نمایش بصری زنده برای پرواز گروهی
با استفاده از matplotlib برای نمایش 2D/3D
"""

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️ matplotlib در دسترس نیست - نمایش بصری غیرفعال")

import asyncio
import time
import threading
from typing import Dict, List, Optional, Tuple
from collections import deque
import math


class SwarmVisualizer:
    """کلاس نمایش بصری گروه پهپاد"""
    
    def __init__(self, enable_3d: bool = True, history_length: int = 100):
        if not MATPLOTLIB_AVAILABLE:
            raise ImportError("matplotlib برای نمایش بصری نیاز است")
            
        self.enable_3d = enable_3d
        self.history_length = history_length
        
        # داده‌های نمایش
        self.drone_positions: Dict[int, Tuple[float, float, float]] = {}
        self.position_history: Dict[int, deque] = {}
        self.leader_id: Optional[int] = None
        self.formation_type: str = "unknown"
        
        # تنظیمات نمایش
        self.colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']
        self.is_running = False
        self.update_thread = None
        
        # راه‌اندازی matplotlib
        self._setup_plot()
        
    def _setup_plot(self):
        """راه‌اندازی نمودار matplotlib"""
        plt.ion()  # حالت تعاملی
        
        if self.enable_3d:
            self.fig = plt.figure(figsize=(12, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('شرق-غرب (متر)')
            self.ax.set_ylabel('شمال-جنوب (متر)')
            self.ax.set_zlabel('ارتفاع (متر)')
        else:
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.ax.set_xlabel('شرق-غرب (متر)')
            self.ax.set_ylabel('شمال-جنوب (متر)')
            
        self.ax.set_title('نمایش زنده پرواز گروهی')
        self.ax.grid(True)
        
        # عناصر نمودار
        self.drone_scatters = {}
        self.drone_labels = {}
        self.path_lines = {}
        self.formation_lines = []
        
        plt.show(block=False)
        
    def start_realtime_display(self, update_interval: float = 0.5):
        """شروع نمایش زنده"""
        if self.is_running:
            return
            
        self.is_running = True
        self.update_thread = threading.Thread(
            target=self._update_loop, 
            args=(update_interval,),
            daemon=True
        )
        self.update_thread.start()
        print("📊 نمایش بصری زنده شروع شد")
        
    def _update_loop(self, interval: float):
        """حلقه به‌روزرسانی نمایش"""
        while self.is_running:
            try:
                self._update_display()
                plt.pause(interval)
            except Exception as e:
                print(f"خطا در به‌روزرسانی نمایش: {str(e)}")
                time.sleep(interval)
                
    def update_positions(self, positions: Dict[int, any], leader_id: Optional[int] = None, 
                        formation_type: str = "unknown"):
        """به‌روزرسانی موقعیت‌های پهپادها"""
        self.leader_id = leader_id
        self.formation_type = formation_type
        
        # تبدیل موقعیت‌ها به مختصات نسبی
        if positions:
            # یافتن مرکز برای تبدیل به مختصات نسبی
            center_lat = sum(pos.latitude if hasattr(pos, 'latitude') else pos['latitude'] 
                           for pos in positions.values()) / len(positions)
            center_lon = sum(pos.longitude if hasattr(pos, 'longitude') else pos['longitude'] 
                           for pos in positions.values()) / len(positions)
            
            for drone_id, pos in positions.items():
                if hasattr(pos, 'latitude'):
                    lat, lon, alt = pos.latitude, pos.longitude, pos.altitude
                else:
                    lat, lon, alt = pos['latitude'], pos['longitude'], pos['altitude']
                    
                # تبدیل به متر نسبت به مرکز
                x = (lon - center_lon) * 111319.9 * math.cos(math.radians(center_lat))
                y = (lat - center_lat) * 111319.9
                z = alt
                
                self.drone_positions[drone_id] = (x, y, z)
                
                # اضافه کردن به تاریخچه
                if drone_id not in self.position_history:
                    self.position_history[drone_id] = deque(maxlen=self.history_length)
                self.position_history[drone_id].append((x, y, z, time.time()))
                
    def _update_display(self):
        """به‌روزرسانی نمایش"""
        if not self.drone_positions:
            return
            
        # پاکسازی نمودار
        self.ax.clear()
        
        # تنظیم مجدد برچسب‌ها
        if self.enable_3d:
            self.ax.set_xlabel('شرق-غرب (متر)')
            self.ax.set_ylabel('شمال-جنوب (متر)')
            self.ax.set_zlabel('ارتفاع (متر)')
        else:
            self.ax.set_xlabel('شرق-غرب (متر)')
            self.ax.set_ylabel('شمال-جنوب (متر)')
            
        self.ax.set_title(f'پرواز گروهی - آرایش: {self.formation_type} | رهبر: {self.leader_id}')
        self.ax.grid(True)
        
        # رسم موقعیت‌های فعلی
        self._draw_current_positions()
        
        # رسم مسیرها
        self._draw_flight_paths()
        
        # رسم اتصالات آرایش
        self._draw_formation_connections()
        
        # تنظیم محدوده نمایش
        self._set_plot_limits()
        
        # اضافه کردن legend
        self._add_legend()
        
        plt.draw()
        
    def _draw_current_positions(self):
        """رسم موقعیت‌های فعلی پهپادها"""
        for i, (drone_id, (x, y, z)) in enumerate(self.drone_positions.items()):
            color = self.colors[i % len(self.colors)]
            size = 100 if drone_id == self.leader_id else 60
            marker = '^' if drone_id == self.leader_id else 'o'  # مثلث برای رهبر
            
            if self.enable_3d:
                self.ax.scatter([x], [y], [z], c=color, s=size, marker=marker, 
                               label=f'پهپاد {drone_id}' + (' (رهبر)' if drone_id == self.leader_id else ''))
                # اضافه کردن برچسب
                self.ax.text(x, y, z, f'  {drone_id}', fontsize=10)
            else:
                self.ax.scatter([x], [y], c=color, s=size, marker=marker,
                               label=f'پهپاد {drone_id}' + (' (رهبر)' if drone_id == self.leader_id else ''))
                # اضافه کردن برچسب
                self.ax.text(x, y, f'  {drone_id}', fontsize=10)
                
    def _draw_flight_paths(self):
        """رسم مسیرهای طی شده"""
        for drone_id, history in self.position_history.items():
            if len(history) > 1:
                positions = list(history)
                x_coords = [pos[0] for pos in positions]
                y_coords = [pos[1] for pos in positions]
                
                color = self.colors[drone_id % len(self.colors)]
                
                if self.enable_3d:
                    z_coords = [pos[2] for pos in positions]
                    self.ax.plot(x_coords, y_coords, z_coords, color=color, alpha=0.3, linewidth=1)
                else:
                    self.ax.plot(x_coords, y_coords, color=color, alpha=0.3, linewidth=1)
                    
    def _draw_formation_connections(self):
        """رسم خطوط اتصال آرایش"""
        if len(self.drone_positions) < 2:
            return
            
        positions = list(self.drone_positions.values())
        
        if self.formation_type == "line":
            # اتصال خطی
            sorted_positions = sorted(positions, key=lambda p: p[0])  # مرتب بر اساس x
            for i in range(len(sorted_positions) - 1):
                self._draw_connection_line(sorted_positions[i], sorted_positions[i+1])
                
        elif self.formation_type == "triangle":
            # اتصال مثلثی (اگر بیش از 2 پهپاد)
            if len(positions) >= 3:
                for i in range(3):
                    j = (i + 1) % 3
                    if i < len(positions) and j < len(positions):
                        self._draw_connection_line(positions[i], positions[j])
                        
        elif self.formation_type == "square":
            # اتصال مربعی
            if len(positions) >= 4:
                for i in range(4):
                    j = (i + 1) % 4
                    if i < len(positions) and j < len(positions):
                        self._draw_connection_line(positions[i], positions[j])
                        
    def _draw_connection_line(self, pos1: Tuple[float, float, float], 
                            pos2: Tuple[float, float, float]):
        """رسم خط اتصال بین دو نقطه"""
        if self.enable_3d:
            self.ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], 
                        'k--', alpha=0.5, linewidth=1)
        else:
            self.ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 
                        'k--', alpha=0.5, linewidth=1)
            
    def _set_plot_limits(self):
        """تنظیم محدوده نمایش"""
        if not self.drone_positions:
            return
            
        positions = list(self.drone_positions.values())
        x_coords = [pos[0] for pos in positions]
        y_coords = [pos[1] for pos in positions]
        z_coords = [pos[2] for pos in positions] if self.enable_3d else []
        
        # اضافه کردن حاشیه
        margin = 5.0
        
        x_min, x_max = min(x_coords) - margin, max(x_coords) + margin
        y_min, y_max = min(y_coords) - margin, max(y_coords) + margin
        
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        
        if self.enable_3d and z_coords:
            z_min, z_max = min(z_coords) - margin, max(z_coords) + margin
            self.ax.set_zlim(max(0, z_min), z_max)
            
    def _add_legend(self):
        """اضافه کردن راهنما"""
        if self.drone_positions:
            self.ax.legend(loc='upper right', fontsize=8)
            
    def add_waypoint_marker(self, x: float, y: float, z: float = None, 
                           label: str = "هدف", color: str = 'red'):
        """اضافه کردن نشانگر نقطه هدف"""
        if self.enable_3d and z is not None:
            self.ax.scatter([x], [y], [z], c=color, s=200, marker='*', 
                           label=label, edgecolors='black')
        else:
            self.ax.scatter([x], [y], c=color, s=200, marker='*', 
                           label=label, edgecolors='black')
            
    def highlight_collision_risk(self, drone_id1: int, drone_id2: int):
        """برجسته کردن خطر برخورد"""
        if drone_id1 in self.drone_positions and drone_id2 in self.drone_positions:
            pos1 = self.drone_positions[drone_id1]
            pos2 = self.drone_positions[drone_id2]
            
            if self.enable_3d:
                self.ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], 
                            'r-', linewidth=3, alpha=0.8, label='خطر برخورد')
            else:
                self.ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], 
                            'r-', linewidth=3, alpha=0.8, label='خطر برخورد')
                
    def save_current_plot(self, filename: str = None):
        """ذخیره نمودار فعلی"""
        if not filename:
            filename = f"swarm_plot_{int(time.time())}.png"
            
        try:
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"📸 نمودار در {filename} ذخیره شد")
        except Exception as e:
            print(f"خطا در ذخیره نمودار: {str(e)}")
            
    def close(self):
        """بستن نمایش"""
        self.is_running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=2)
            
        try:
            plt.close(self.fig)
        except:
            pass
            
        print("📊 نمایش بصری بسته شد")
        
    def get_statistics(self) -> Dict:
        """آمار نمایش"""
        total_points = sum(len(history) for history in self.position_history.values())
        
        return {
            'active_drones': len(self.drone_positions),
            'total_history_points': total_points,
            'display_mode': '3D' if self.enable_3d else '2D',
            'is_running': self.is_running,
            'formation_type': self.formation_type,
            'leader_id': self.leader_id
        }