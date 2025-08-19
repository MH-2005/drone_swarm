#!/usr/bin/env python3
"""
اسکریپت اصلی مسابقه پرواز گروهی
قابل اجرا با دستورات مختلف مطابق شیوه‌نامه
نویسنده: تیم توسعه پرواز گروهی
"""

import asyncio
import argparse
import signal
import sys
import os
import time
import json
from typing import Optional, Dict, Any

# Import کلاس‌های اصلی
from core.swarm_manager import SwarmManager, SwarmConfig
from core.drone import Position
from formation.shapes import FormationGenerator, FormationType, Orientation, FormationParameters
from formation.transforms import FormationTransformer, TransformationParameters
from navigation.leader_follower import LeaderFollowerController
from navigation.path_manager import PathManager, create_sample_path
from avoidance.hybrid_avoidance import HybridAvoidanceController, AvoidanceStrategy
from visualization.realtime_plot import SwarmVisualizer
from visualization.logger import MissionLogger


class CompetitionRunner:
    """کلاس اصلی اجرای مسابقه"""
    
    def __init__(self):
        self.swarm_manager: Optional[SwarmManager] = None
        self.formation_generator: Optional[FormationGenerator] = None
        self.transformer: Optional[FormationTransformer] = None
        self.leader_follower: Optional[LeaderFollowerController] = None
        self.path_manager: Optional[PathManager] = None
        self.avoidance_controller: Optional[HybridAvoidanceController] = None
        self.visualizer: Optional[SwarmVisualizer] = None
        self.logger: Optional[MissionLogger] = None
        
        # وضعیت اجرا
        self.is_running = False
        self.mission_start_time = 0.0
        
        print("🚁 سیستم مسابقه پرواز گروهی آماده شد")
        
    async def initialize_system(self, config: SwarmConfig) -> bool:
        """راه‌اندازی کامل سیستم"""
        try:
            print("📋 راه‌اندازی سیستم...")
            
            # ایجاد مدیر گروه
            self.swarm_manager = SwarmManager(config)
            
            # راه‌اندازی گروه
            if not await self.swarm_manager.initialize_swarm():
                return False
                
            # ایجاد کامپوننت‌های اصلی
            self.formation_generator = FormationGenerator()
            self.transformer = FormationTransformer(self.swarm_manager)
            self.path_manager = PathManager()
            self.leader_follower = LeaderFollowerController(self.swarm_manager, self.path_manager)
            self.avoidance_controller = HybridAvoidanceController(self.swarm_manager)
            self.logger = MissionLogger()
            
            # راه‌اندازی visualizer (اختیاری)
            try:
                self.visualizer = SwarmVisualizer()
                print("📊 نمایش بصری فعال شد")
            except ImportError:
                print("⚠️ نمایش بصری در دسترس نیست")
                
            # راه‌اندازی مانع‌گریزی
            await self.avoidance_controller.initialize_avoidance_system()
            
            # تنظیم signal handlers
            self._setup_signal_handlers()
            
            self.is_running = True
            print("✅ سیستم با موفقیت راه‌اندازی شد")
            return True
            
        except Exception as e:
            print(f"❌ خطا در راه‌اندازی سیستم: {str(e)}")
            return False
            
    def _setup_signal_handlers(self):
        """تنظیم signal handlers برای خاتمه امن"""
        def signal_handler(signum, frame):
            print("\n🛑 دریافت سیگنال خروج، خاتمه امن...")
            asyncio.create_task(self.shutdown_system())
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
    async def execute_formation_mission(self, params: FormationParameters) -> bool:
        """اجرای مأموریت تشکیل آرایش (مرحله 1)"""
        try:
            print(f"\n🎯 مأموریت: تشکیل آرایش {params.formation_type.value}")
            self.mission_start_time = time.time()
            
            # برخاستن همه پهپادها
            if not await self.swarm_manager.takeoff_all_drones(params.altitude):
                return False
                
            await asyncio.sleep(3)  # انتظار برای پایدار شدن
            
            # تولید موقعیت‌های آرایش
            num_drones = len(self.swarm_manager.get_active_drones())
            formation_positions = self.formation_generator.generate_formation_positions(num_drones, params)
            
            # حرکت پهپادها به موقعیت‌های آرایش
            movement_tasks = []
            active_drones = self.swarm_manager.get_active_drones()
            
            for i, drone in enumerate(active_drones):
                if i < len(formation_positions):
                    target_pos = formation_positions[i]
                    task = drone.goto_position(target_pos, speed=params.size/2)  # سرعت متناسب با اندازه
                    movement_tasks.append(task)
                    
            # اجرای همزمان حرکات
            results = await asyncio.gather(*movement_tasks, return_exceptions=True)
            success_count = sum(1 for result in results if result is True)
            
            if success_count >= len(active_drones) * 0.8:  # 80% موفقیت
                print(f"✅ آرایش {params.formation_type.value} با موفقیت تشکیل شد")
                
                # لاگ موقعیت‌های نهایی
                await self._log_formation_positions("formation_completed")
                return True
            else:
                print(f"❌ تنها {success_count}/{len(active_drones)} پهپاد به موقعیت رسید")
                return False
                
        except Exception as e:
            print(f"❌ خطا در مأموریت آرایش: {str(e)}")
            return False
            
    async def execute_transformation_mission(self, params: TransformationParameters) -> bool:
        """اجرای مأموریت تبدیل آرایش (مرحله 2)"""
        try:
            print(f"\n🔄 مأموریت: تبدیل آرایش")
            print(f"   جابه‌جایی: ({params.move_x}, {params.move_y}, {params.move_z})m")
            print(f"   دوران: ({params.rotate_x}, {params.rotate_y}, {params.rotate_z})°")
            
            # اجرای تبدیل
            success = await self.transformer.apply_transformation(params)
            
            if success:
                print("✅ تبدیل آرایش با موفقیت انجام شد")
                await self._log_formation_positions("transformation_completed")
                return True
            else:
                print("❌ خطا در تبدیل آرایش")
                return False
                
        except Exception as e:
            print(f"❌ خطا در مأموریت تبدیل: {str(e)}")
            return False
            
    async def execute_leader_follower_mission(self, 
                                            formation_type: str = "triangle",
                                            formation_size: float = 8.0,
                                            path_file: str = "leader_path.csv") -> bool:
        """اجرای مأموریت رهبر-پیرو (مرحله 3)"""
        try:
            print(f"\n👥 مأموریت: رهبر-پیرو")
            print(f"   آرایش: {formation_type} (اندازه: {formation_size}m)")
            print(f"   مسیر: {path_file}")
            
            # بارگذاری مسیر
            if not os.path.exists(path_file):
                print(f"⚠️ فایل مسیر {path_file} وجود ندارد، ایجاد مسیر نمونه...")
                create_sample_path(path_file)
                
            if not self.path_manager.load_path_from_csv(path_file):
                return False
                
            # تنظیم رهبر
            leader = self.swarm_manager.get_leader()
            if not leader:
                print("❌ رهبری برای مأموریت وجود ندارد")
                return False
                
            self.path_manager.set_current_drone(leader)
            
            # تنظیم آرایش پیروها
            self.leader_follower.setup_formation(formation_type, formation_size)
            
            # شروع پیروی
            if not await self.leader_follower.start_following():
                return False
                
            # نظارت بر مأموریت
            monitor_task = asyncio.create_task(self._monitor_leader_follower_mission())
            
            # اجرای مسیر
            path_success = await self.path_manager.execute_path()
            
            # توقف نظارت
            monitor_task.cancel()
            
            # توقف پیروی
            await self.leader_follower.stop_following()
            
            if path_success:
                print("✅ مأموریت رهبر-پیرو با موفقیت انجام شد")
                
                # آمار نهایی
                stats = self.leader_follower.get_statistics()
                print(f"   تغییرات رهبر: {stats['leader_changes']}")
                print(f"   خطاهای آرایش: {stats['total_formation_errors']}")
                
                return True
            else:
                print("❌ خطا در اجرای مسیر")
                return False
                
        except Exception as e:
            print(f"❌ خطا در مأموریت رهبر-پیرو: {str(e)}")
            return False
            
    async def execute_leader_removal_mission(self) -> bool:
        """اجرای مأموریت حذف رهبر (مرحله 4)"""
        try:
            print(f"\n🔄 مأموریت: حذف رهبر و ادامه حرکت")
            
            # اطمینان از وجود رهبر
            initial_leader = self.swarm_manager.get_leader()
            if not initial_leader:
                print("❌ رهبری برای حذف وجود ندارد")
                return False
                
            print(f"رهبر فعلی: پهپاد {initial_leader.id}")
            
            # شروع مأموریت رهبر-پیرو
            leader_follower_success = await self.execute_leader_follower_mission()
            if not leader_follower_success:
                return False
                
            # انتظار کوتاه
            await asyncio.sleep(5)
            
            # حذف رهبر
            print("🔴 حذف رهبر فعلی...")
            removal_success = await self.swarm_manager.disarm_leader()
            
            if removal_success:
                new_leader = self.swarm_manager.get_leader()
                if new_leader:
                    print(f"✅ رهبر جدید انتخاب شد: پهپاد {new_leader.id}")
                    
                    # ادامه مأموریت با رهبر جدید
                    await asyncio.sleep(2)
                    remaining_path_success = await self._continue_mission_with_new_leader()
                    
                    return remaining_path_success
                else:
                    print("❌ نمی‌توان رهبر جدید انتخاب کرد")
                    return False
            else:
                print("❌ خطا در حذف رهبر")
                return False
                
        except Exception as e:
            print(f"❌ خطا در مأموریت حذف رهبر: {str(e)}")
            return False
            
    async def _continue_mission_with_new_leader(self) -> bool:
        """ادامه مأموریت با رهبر جدید"""
        try:
            new_leader = self.swarm_manager.get_leader()
            if not new_leader:
                return False
                
            # بارگذاری مجدد مسیر از نقطه فعلی
            remaining_points = len(self.path_manager.path_points) - self.path_manager.current_point_index
            
            if remaining_points > 0:
                print(f"ادامه مسیر با {remaining_points} نقطه باقی‌مانده")
                
                # تنظیم رهبر جدید
                self.path_manager.set_current_drone(new_leader)
                
                # ادامه مسیر
                remaining_success = await self.path_manager.execute_path()
                return remaining_success
            else:
                print("مسیر قبلاً تکمیل شده")
                return True
                
        except Exception as e:
            print(f"خطا در ادامه مأموریت: {str(e)}")
            return False
            
    async def _monitor_leader_follower_mission(self):
        """نظارت بر مأموریت رهبر-پیرو"""
        try:
            while True:
                # بررسی سلامت گروه
                if not await self.swarm_manager.check_safety_constraints():
                    print("⚠️ نقض قیدهای ایمنی تشخیص داده شد")
                    
                # نمایش پیشرفت مسیر
                path_status = self.path_manager.get_path_status()
                formation_status = self.leader_follower.get_formation_status()
                
                print(f"📊 پیشرفت: {path_status['progress_percentage']}% | "
                      f"پیروهای فعال: {formation_status['active_followers']} | "
                      f"تغییرات رهبر: {formation_status['leader_changes']}")
                      
                # به‌روزرسانی visualizer
                if self.visualizer:
                    await self._update_visualizer()
                    
                await asyncio.sleep(2)
                
        except asyncio.CancelledError:
            print("نظارت مأموریت متوقف شد")
            
    async def _update_visualizer(self):
        """به‌روزرسانی نمایش بصری"""
        try:
            if not self.visualizer:
                return
                
            # جمع‌آوری موقعیت‌های فعلی
            positions = {}
            for drone in self.swarm_manager.get_active_drones():
                pos = await drone.get_current_position()
                if pos:
                    positions[drone.id] = pos
                    
            # به‌روزرسانی نمایش
            leader_id = self.swarm_manager.leader_id
            formation_type = getattr(self.leader_follower.config, 'formation_type', 'unknown')
            
            self.visualizer.update_positions(positions, leader_id, formation_type)
            
        except Exception as e:
            print(f"خطا در به‌روزرسانی نمایش: {str(e)}")
            
    async def _log_formation_positions(self, event_name: str):
        """لاگ موقعیت‌های آرایش"""
        if not self.logger:
            return
            
        try:
            positions = {}
            for drone in self.swarm_manager.get_active_drones():
                pos = await drone.get_current_position()
                if pos:
                    positions[drone.id] = {
                        'latitude': pos.latitude,
                        'longitude': pos.longitude,
                        'altitude': pos.altitude
                    }
                    
            await self.logger.log_event(event_name, {
                'timestamp': time.time(),
                'positions': positions,
                'leader_id': self.swarm_manager.leader_id
            })
            
        except Exception as e:
            print(f"خطا در لاگ‌گیری: {str(e)}")
            
    async def run_complete_mission_sequence(self) -> bool:
        """اجرای کامل تمام مراحل مسابقه"""
        try:
            print("\n🚀 شروع دنباله کامل مأموریت‌ها")
            
            # مرحله 1: تشکیل آرایش
            formation_params = FormationParameters(
                formation_type=FormationType.TRIANGLE,
                size=8.0,
                altitude=20.0,
                orientation=Orientation.HORIZONTAL
            )
            
            if not await self.execute_formation_mission(formation_params):
                print("❌ مرحله 1 ناموفق")
                return False
                
            await asyncio.sleep(3)
            
            # مرحله 2: تبدیل آرایش
            transform_params = TransformationParameters(
                move_x=10.0,
                move_y=5.0,
                rotate_z=45.0
            )
            
            if not await self.execute_transformation_mission(transform_params):
                print("❌ مرحله 2 ناموفق")
                return False
                
            await asyncio.sleep(3)
            
            # مرحله 3: رهبر-پیرو
            if not await self.execute_leader_follower_mission():
                print("❌ مرحله 3 ناموفق")
                return False
                
            await asyncio.sleep(3)
            
            # مرحله 4: حذف رهبر
            if not await self.execute_leader_removal_mission():
                print("❌ مرحله 4 ناموفق")
                return False
                
            print("🎉 تمام مراحل مسابقه با موفقیت انجام شد!")
            
            # نمایش آمار نهایی
            await self._display_final_statistics()
            return True
            
        except Exception as e:
            print(f"❌ خطا در اجرای دنباله مأموریت‌ها: {str(e)}")
            return False
            
    async def _display_final_statistics(self):
        """نمایش آمار نهایی"""
        try:
            print("\n📊 آمار نهایی مأموریت:")
            
            # آمار گروه
            swarm_status = await self.swarm_manager.get_swarm_status()
            print(f"   زمان کل: {swarm_status.get('mission_duration', 0):.2f}s")
            print(f"   تغییرات آرایش: {swarm_status.get('formation_changes', 0)}")
            print(f"   تغییرات رهبر: {swarm_status.get('leader_changes', 0)}")
            
            # آمار رهبر-پیرو
            if self.leader_follower:
                lf_stats = self.leader_follower.get_statistics()
                print(f"   خطاهای آرایش: {lf_stats.get('total_formation_errors', 0)}")
                print(f"   میانگین خطا: {lf_stats.get('average_formation_error', 0):.2f}m")
                
            # آمار مسیر
            if self.path_manager:
                path_status = self.path_manager.get_path_status()
                print(f"   نقاط بازدید شده: {path_status.get('points_visited', 0)}")
                print(f"   مسافت طی شده: {path_status.get('total_distance', 0):.2f}m")
                
            # آمار مانع‌گریزی
            if self.avoidance_controller:
                avoid_status = self.avoidance_controller.get_avoidance_status()
                print(f"   موانع تشخیص داده شده: {avoid_status.get('detected_obstacles', 0)}")
                
        except Exception as e:
            print(f"خطا در نمایش آمار: {str(e)}")
            
    async def shutdown_system(self):
        """خاتمه امن سیستم"""
        try:
            print("\n🛑 شروع خاتمه امن سیستم...")
            self.is_running = False
            
            # توقف کامپوننت‌ها
            if self.leader_follower:
                await self.leader_follower.stop_following()
                await self.leader_follower.cleanup()
                
            if self.avoidance_controller:
                await self.avoidance_controller.shutdown()
                
            if self.visualizer:
                self.visualizer.close()
                
            # فرود و خاموش کردن گروه
            if self.swarm_manager:
                await self.swarm_manager.shutdown_swarm()
                
            # ذخیره لاگ‌ها
            if self.logger:
                await self.logger.save_logs("mission_logs")
                
            print("✅ سیستم با موفقیت خاموش شد")
            
        except Exception as e:
            print(f"خطا در خاتمه سیستم: {str(e)}")


def parse_arguments():
    """تجزیه آرگومان‌های خط فرمان"""
    parser = argparse.ArgumentParser(description='سیستم مسابقه پرواز گروهی')
    
    # تنظیمات کلی
    parser.add_argument('--num-drones', type=int, default=4, help='تعداد پهپادها')
    parser.add_argument('--base-port', type=int, default=14540, help='پورت پایه اتصال')
    
    # نوع مأموریت
    parser.add_argument('--mission', choices=['formation', 'transform', 'leader_follower', 
                                            'leader_removal', 'complete'], 
                       default='complete', help='نوع مأموریت')
    
    # پارامترهای آرایش
    parser.add_argument('--formation', choices=['line', 'triangle', 'square', 'diamond'], 
                       default='triangle', help='نوع آرایش')
    parser.add_argument('--size', type=float, default=8.0, help='اندازه آرایش (متر)')
    parser.add_argument('--altitude', type=float, default=20.0, help='ارتفاع پرواز (متر)')
    parser.add_argument('--orientation', choices=['horizontal', 'vertical'], 
                       default='horizontal', help='جهت آرایش')
    
    # پارامترهای تبدیل
    parser.add_argument('--move-x', type=float, default=0.0, help='جابه‌جایی X (متر)')
    parser.add_argument('--move-y', type=float, default=0.0, help='جابه‌جایی Y (متر)')
    parser.add_argument('--move-z', type=float, default=0.0, help='جابه‌جایی Z (متر)')
    parser.add_argument('--rotate-x', type=float, default=0.0, help='دوران X (درجه)')
    parser.add_argument('--rotate-y', type=float, default=0.0, help='دوران Y (درجه)')
    parser.add_argument('--rotate-z', type=float, default=0.0, help='دوران Z (درجه)')
    
    # مسیر
    parser.add_argument('--path-file', default='leader_path.csv', help='فایل مسیر CSV')
    
    # تنظیمات اضافی
    parser.add_argument('--enable-visualization', action='store_true', help='فعال‌سازی نمایش بصری')
    parser.add_argument('--enable-avoidance', action='store_true', help='فعال‌سازی مانع‌گریزی')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'], 
                       default='INFO', help='سطح لاگ')
    
    return parser.parse_args()


async def main():
    """تابع اصلی برنامه"""
    print("🚁 سیستم مسابقه پرواز گروهی")
    print("=" * 50)
    
    args = parse_arguments()
    
    try:
        # ایجاد تنظیمات گروه
        config = SwarmConfig(
            num_drones=args.num_drones,
            base_port=args.base_port,
            formation_spacing=args.size,
            min_altitude=max(5.0, args.altitude - 10),
            max_altitude=args.altitude + 20
        )
        
        # ایجاد runner
        runner = CompetitionRunner()
        
        # راه‌اندازی سیستم
        if not await runner.initialize_system(config):
            print("❌ خطا در راه‌اندازی")
            return 1
            
        # اجرای مأموریت بر اساس نوع
        success = False
        
        if args.mission == 'formation':
            # مأموریت آرایش
            formation_params = FormationParameters(
                formation_type=FormationType(args.formation),
                size=args.size,
                altitude=args.altitude,
                orientation=Orientation(args.orientation)
            )
            success = await runner.execute_formation_mission(formation_params)
            
        elif args.mission == 'transform':
            # مأموریت تبدیل
            transform_params = TransformationParameters(
                move_x=args.move_x, move_y=args.move_y, move_z=args.move_z,
                rotate_x=args.rotate_x, rotate_y=args.rotate_y, rotate_z=args.rotate_z
            )
            success = await runner.execute_transformation_mission(transform_params)
            
        elif args.mission == 'leader_follower':
            # مأموریت رهبر-پیرو
            success = await runner.execute_leader_follower_mission(
                args.formation, args.size, args.path_file
            )
            
        elif args.mission == 'leader_removal':
            # مأموریت حذف رهبر
            success = await runner.execute_leader_removal_mission()
            
        elif args.mission == 'complete':
            # مأموریت کامل
            success = await runner.run_complete_mission_sequence()
            
        # خاتمه سیستم
        await runner.shutdown_system()
        
        if success:
            print("\n🎉 مأموریت با موفقیت انجام شد!")
            return 0
        else:
            print("\n❌ مأموریت ناموفق")
            return 1
            
    except KeyboardInterrupt:
        print("\n⏸️ متوقف شد توسط کاربر")
        return 0
    except Exception as e:
        print(f"\n❌ خطای غیرمنتظره: {str(e)}")
        return 1


if __name__ == "__main__":
    # تنظیم event loop policy برای Windows
    if sys.platform.startswith('win'):
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
        
    # اجرای برنامه اصلی
    exit_code = asyncio.run(main())
    sys.exit(exit_code)