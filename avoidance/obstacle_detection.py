"""
ماژول شبیه‌سازی تشخیص موانع
در یک سیستم واقعی، این ماژول داده‌ها را از سنسورهایی مانند LiDAR یا دوربین دریافت می‌کند.
در این شبیه‌سازی، ما موانع مجازی را در محیط ایجاد کرده و آن‌ها را به کنترلر مانع‌گریزی گزارش می‌دهیم.
"""

import asyncio
import random
import time
from typing import Dict, List, Optional
from dataclasses import dataclass

from ..core.drone import Position
from .hybrid_avoidance import ObstacleInfo, ObstacleType, HybridAvoidanceController

@dataclass
class SimulatedObstacle:
    """کلاسی برای نمایش یک مانع مجازی در محیط شبیه‌سازی."""
    obstacle_id: str
    position: Position
    size: float  # شعاع مانع بر حسب متر
    is_dynamic: bool = False
    velocity: Optional[tuple] = None  # (vx, vy, vz) in m/s

class ObstacleDetector:
    """
    کلاس شبیه‌ساز سنسور که موانع را در محیط "تشخیص" می‌دهد.
    """
    def __init__(self, avoidance_controller: HybridAvoidanceController, simulation_area: tuple):
        """
        avoidance_controller: کنترلری که موانع شناسایی شده به آن گزارش می‌شود.
        simulation_area: یک تاپل (center_pos, radius) که محدوده فعالیت شبیه‌ساز را مشخص می‌کند.
        """
        self.avoidance_controller = avoidance_controller
        self.center_pos, self.radius = simulation_area
        self.simulated_obstacles: Dict[str, SimulatedObstacle] = {}
        self.is_active = False
        self.detector_task: Optional[asyncio.Task] = None

    def start_simulation(self, num_static: int = 3, num_dynamic: int = 1):
        """شبیه‌سازی را با ایجاد تعدادی مانع ثابت و متحرک آغاز می‌کند."""
        print(f"🔎 شبیه‌ساز تشخیص مانع با {num_static} مانع ثابت و {num_dynamic} مانع متحرک فعال شد.")
        self._generate_random_obstacles(num_static, ObstacleType.STATIC)
        self._generate_random_obstacles(num_dynamic, ObstacleType.DYNAMIC)
        
        self.is_active = True
        self.detector_task = asyncio.create_task(self._simulation_loop())

    async def stop_simulation(self):
        """حلقه شبیه‌سازی را متوقف می‌کند."""
        self.is_active = False
        if self.detector_task:
            self.detector_task.cancel()
            try:
                await self.detector_task
            except asyncio.CancelledError:
                pass
        print("شبیه‌ساز تشخیص مانع متوقف شد.")

    def _generate_random_obstacles(self, count: int, obs_type: ObstacleType):
        """ایجاد موانع تصادفی در محدوده شبیه‌سازی."""
        for i in range(count):
            obstacle_id = f"{obs_type.value}_{i}"
            
            # ایجاد موقعیت تصادفی
            angle = random.uniform(0, 2 * 3.14159)
            distance = random.uniform(self.radius * 0.2, self.radius * 0.8)
            lat_diff = (distance * math.sin(angle)) / 111319.9
            lon_diff = (distance * math.cos(angle)) / (111319.9 * math.cos(math.radians(self.center_pos.latitude)))
            
            position = Position(
                latitude=self.center_pos.latitude + lat_diff,
                longitude=self.center_pos.longitude + lon_diff,
                altitude=random.uniform(15.0, 30.0)
            )
            
            velocity = (random.uniform(-1, 1), random.uniform(-1, 1), 0) if obs_type == ObstacleType.DYNAMIC else None
            
            self.simulated_obstacles[obstacle_id] = SimulatedObstacle(
                obstacle_id=obstacle_id,
                position=position,
                size=random.uniform(2.0, 4.0),
                is_dynamic=(obs_type == ObstacleType.DYNAMIC),
                velocity=velocity
            )

    async def _simulation_loop(self):
        """حلقه اصلی که به طور مداوم موانع را به‌روز کرده و گزارش می‌دهد."""
        while self.is_active:
            await self._update_dynamic_obstacles()
            await self._report_obstacles_to_controller()
            await asyncio.sleep(1.0) # هر یک ثانیه یکبار

    async def _update_dynamic_obstacles(self):
        """موقعیت موانع متحرک را به‌روز می‌کند."""
        for obs in self.simulated_obstacles.values():
            if obs.is_dynamic and obs.velocity:
                # به‌روزرسانی ساده موقعیت بر اساس سرعت
                lat_diff = (obs.velocity[1]) / 111319.9
                lon_diff = (obs.velocity[0]) / (111319.9 * math.cos(math.radians(obs.position.latitude)))
                
                obs.position.latitude += lat_diff
                obs.position.longitude += lon_diff
                # TODO: افزودن منطق بازگشت به محدوده در صورت خروج

    async def _report_obstacles_to_controller(self):
        """
        موانع شبیه‌سازی شده را در قالب ObstacleInfo به کنترلر مانع‌گریزی گزارش می‌دهد.
        """
        detected_obstacles_batch = {}
        for obs in self.simulated_obstacles.values():
            obstacle_info = ObstacleInfo(
                obstacle_id=obs.obstacle_id,
                position=obs.position,
                size=obs.size,
                type=ObstacleType.DYNAMIC if obs.is_dynamic else ObstacleType.STATIC,
                last_seen=time.time(),
                velocity=obs.velocity
            )
            detected_obstacles_batch[obs.obstacle_id] = obstacle_info
            
        # به‌روزرسانی دیکشنری موانع در کنترلر
        self.avoidance_controller.detected_obstacles.update(detected_obstacles_batch)