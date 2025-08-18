"""
اجراکننده ماموریت (Mission Executor)
این کلاس شامل منطق اصلی برای اجرای مراحل مختلف مسابقه است.
این کار باعث جداسازی منطق از اسکریپت اجرایی اصلی (run_competition.py) می‌شود.
"""

import asyncio
import os
import time
from typing import Optional

from ..core.swarm_manager import SwarmManager, SwarmConfig
from ..formation.shapes import FormationGenerator, FormationParameters
from ..formation.transforms import FormationTransformer, TransformationParameters
from ..navigation.leader_follower import LeaderFollowerController
from ..navigation.path_manager import PathManager, create_sample_path
from ..avoidance.hybrid_avoidance import HybridAvoidanceController
from ..visualization.realtime_plot import SwarmVisualizer
from ..visualization.logger import MissionLogger

class MissionExecutor:
    """کلاس اصلی اجرای سناریوهای مسابقه."""
    
    def __init__(self):
        # این متد تمام اجزای سیستم را مقداردهی اولیه می‌کند.
        self.swarm_manager: Optional[SwarmManager] = None
        self.formation_generator: Optional[FormationGenerator] = None
        self.transformer: Optional[FormationTransformer] = None
        self.leader_follower: Optional[LeaderFollowerController] = None
        self.path_manager: Optional[PathManager] = None
        self.avoidance_controller: Optional[HybridAvoidanceController] = None
        self.visualizer: Optional[SwarmVisualizer] = None
        self.logger: Optional[MissionLogger] = None
        self.is_running = False
        print("🚁 اجراکننده ماموریت آماده شد.")
        
    async def initialize_system(self, config: SwarmConfig, enable_visualization: bool = False) -> bool:
        """راه‌اندازی کامل سیستم."""
        try:
            print("📋 راه‌اندازی سیستم...")
            
            # ایجاد مدیر گروه و اتصال به پهپادها
            self.swarm_manager = SwarmManager(config)
            if not await self.swarm_manager.initialize_swarm():
                return False
                
            # ایجاد سایر کامپوننت‌ها
            self.formation_generator = FormationGenerator()
            self.transformer = FormationTransformer(self.swarm_manager)
            self.path_manager = PathManager()
            self.leader_follower = LeaderFollowerController(self.swarm_manager, self.path_manager)
            self.avoidance_controller = HybridAvoidanceController(self.swarm_manager)
            self.logger = MissionLogger()
            
            if enable_visualization:
                try:
                    self.visualizer = SwarmVisualizer()
                except ImportError:
                    print("⚠️ نمایش بصری در دسترس نیست (matplotlib نصب نشده).")
            
            await self.avoidance_controller.initialize_avoidance_system()
            self.is_running = True
            print("✅ سیستم با موفقیت راه‌اندازی شد.")
            return True
            
        except Exception as e:
            print(f"❌ خطا در راه‌اندازی سیستم: {e}")
            return False

    # --------------------------------------------------------------------------
    # ## بخش اجرای ماموریت‌ها ##
    # (این بخش دقیقاً همان متدهای execute_..._mission از فایل run_competition.py قبلی است)
    # --------------------------------------------------------------------------

    async def execute_formation_mission(self, params: FormationParameters) -> bool:
        """اجرای مأموریت تشکیل آرایش (مرحله 1)"""
        # ... (کد این متد بدون تغییر از run_competition.py منتقل می‌شود)
        print(f"\n🎯 مأموریت: تشکیل آرایش {params.formation_type.value}")
        if not await self.swarm_manager.takeoff_all_drones(params.altitude): return False
        await asyncio.sleep(3)
        num_drones = len(self.swarm_manager.get_active_drones())
        positions = self.formation_generator.generate_formation_positions(num_drones, params)
        # ... بقیه کد
        return True

    async def execute_transformation_mission(self, params: TransformationParameters) -> bool:
        """اجرای مأموریت تبدیل آرایش (مرحله 2)"""
        # ... (کد این متد بدون تغییر از run_competition.py منتقل می‌شود)
        print(f"\n🔄 مأموریت: تبدیل آرایش")
        success = await self.transformer.apply_transformation(params)
        # ... بقیه کد
        return success

    async def execute_leader_follower_mission(self, formation_type: str, formation_size: float, path_file: str) -> bool:
        """اجرای مأموریت رهبر-پیرو (مرحله 3)"""
        # ... (کد این متد بدون تغییر از run_competition.py منتقل می‌شود)
        print(f"\n👥 مأموریت: رهبر-پیرو")
        if not os.path.exists(path_file): create_sample_path(path_file)
        if not self.path_manager.load_path_from_csv(path_file): return False
        # ... بقیه کد
        return True
    
    # ... و سایر متدهای اجرایی و کمکی مانند execute_leader_removal_mission, run_complete_mission_sequence
    # ... shutdown_system, _update_visualizer, _log_formation_positions

    async def shutdown_system(self):
        """خاتمه امن سیستم."""
        # ... (کد این متد بدون تغییر از run_competition.py منتقل می‌شود)
        print("\n🛑 شروع خاتمه امن سیستم...")
        if self.leader_follower: await self.leader_follower.stop_following()
        if self.swarm_manager: await self.swarm_manager.shutdown_swarm()
        if self.logger: await self.logger.shutdown()
        if self.visualizer: self.visualizer.close()
        print("✅ سیستم با موفقیت خاموش شد.")