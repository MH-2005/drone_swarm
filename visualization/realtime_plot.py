"""
نمایش بصری زنده گروه پهپادها با استفاده از Matplotlib
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Dict, Optional

class SwarmVisualizer:
    """کلاس نمایش زنده موقعیت پهپادها در یک نمودار دوبعدی."""

    def __init__(self):
        try:
            plt.style.use('seaborn-v0_8-darkgrid')
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.ax.set_aspect('equal', adjustable='box')
            self.ax.set_xlabel("موقعیت شرقی-غربی (متر)")
            self.ax.set_ylabel("موقعیت شمالی-جنوبی (متر)")
            self.ax.set_title("نمایش زنده گروه پهپادها")

            self.scatter_drones = self.ax.scatter([], [], s=100, c='blue', label="پیروها")
            self.scatter_leader = self.ax.scatter([], [], s=150, c='red', marker='*', label="رهبر")
            self.text_annotations = []
            
            self.positions_data: Dict[int, tuple] = {}
            self.leader_id: Optional[int] = None
            self.center_lat: Optional[float] = None
            self.center_lon: Optional[float] = None

            self.ani = FuncAnimation(self.fig, self._update_plot, interval=500, blit=False)
            plt.legend()
            plt.show(block=False)
            print("📊 نمایش بصری فعال شد.")
        except Exception as e:
            print(f"⚠️ خطا در راه‌اندازی نمایش بصری: {e}. ممکن است matplotlib نصب نباشد.")
            raise ImportError("Matplotlib is required for visualization.")

    def update_positions(self, positions: Dict[int, any], leader_id: Optional[int]):
        """
        به‌روزرسانی داده‌های موقعیت برای نمایش در فریم بعدی.
        
        positions: دیکشنری از شناسه پهپاد به موقعیت آن (کلاس Position)
        """
        if not self.center_lat and positions:
            # اولین موقعیت را به عنوان مرکز نسبی در نظر می‌گیریم
            first_pos = next(iter(positions.values()))
            self.center_lat = first_pos.latitude
            self.center_lon = first_pos.longitude
        
        self.leader_id = leader_id
        
        # تبدیل موقعیت جغرافیایی به موقعیت نسبی (متر)
        relative_positions = {}
        if self.center_lat:
            for drone_id, pos in positions.items():
                y = (pos.latitude - self.center_lat) * 111319.9
                x = (pos.longitude - self.center_lon) * 111319.9
                relative_positions[drone_id] = (x, y)
        
        self.positions_data = relative_positions

    def _update_plot(self, frame):
        """تابع داخلی که توسط FuncAnimation برای به‌روزرسانی نمودار فراخوانی می‌شود."""
        if not self.positions_data:
            return

        follower_points = [pos for i, pos in self.positions_data.items() if i != self.leader_id]
        leader_point = self.positions_data.get(self.leader_id)

        # به‌روزرسانی موقعیت پیروها
        if follower_points:
            self.scatter_drones.set_offsets([list(p) for p in follower_points])
        else:
            self.scatter_drones.set_offsets([])

        # به‌روزرسانی موقعیت رهبر
        if leader_point:
            self.scatter_leader.set_offsets(list(leader_point))
        else:
            self.scatter_leader.set_offsets([])

        # به‌روزرسانی شماره پهپادها
        for ann in self.text_annotations:
            ann.remove()
        self.text_annotations.clear()

        for drone_id, pos in self.positions_data.items():
            ann = self.ax.text(pos[0] + 1, pos[1] + 1, str(drone_id), color='black', fontsize=9)
            self.text_annotations.append(ann)
            
        # تنظیم مجدد محدوده نمودار
        self.ax.relim()
        self.ax.autoscale_view()

    def close(self):
        """بستن پنجره نمودار."""
        plt.close(self.fig)