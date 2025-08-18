#!/usr/bin/env python3
"""
اسکریپت اصلی و نقطه شروع مسابقه پرواز گروهی.
این اسکریپت آرگومان‌های خط فرمان را تجزیه کرده و اجراکننده ماموریت را فراخوانی می‌کند.
"""

import asyncio
import argparse
import signal
import sys

from ..core.swarm_manager import SwarmConfig
from ..formation.shapes import FormationParameters, FormationType, Orientation
from ..formation.transforms import TransformationParameters
from .mission_executor import MissionExecutor

def parse_arguments():
    """تجزیه آرگومان‌های خط فرمان."""
    parser = argparse.ArgumentParser(description='سیستم مسابقه پرواز گروهی')
    
    # ... (بخش پارسر آرگومان‌ها بدون تغییر باقی می‌ماند)
    parser.add_argument('--num-drones', type=int, default=4, help='تعداد پهپادها')
    parser.add_argument('--mission', choices=['formation', 'transform', 'leader_follower', 'leader_removal', 'complete'], 
                       default='complete', help='نوع مأموریت')
    parser.add_argument('--formation', choices=['line', 'triangle', 'square', 'diamond', 'circle'], default='triangle')
    # ... بقیه آرگومان‌ها
    
    return parser.parse_args()

async def main():
    """تابع اصلی برنامه."""
    print("="*50)
    print("🚁 سیستم مسابقه پرواز گروهی 🚁")
    print("="*50)
    
    args = parse_arguments()
    
    # ایجاد اجراکننده ماموریت
    executor = MissionExecutor()

    # تنظیم سیگنال‌های خروج امن
    def signal_handler(signum, frame):
        print("\n🛑 دریافت سیگنال خروج، خاتمه امن...")
        if executor.is_running:
            asyncio.create_task(executor.shutdown_system())
            
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # ایجاد تنظیمات گروه
        config = SwarmConfig(num_drones=args.num_drones, base_port=args.base_port)
        
        # راه‌اندازی سیستم
        if not await executor.initialize_system(config, args.enable_visualization):
            print("❌ راه‌اندازی سیستم ناموفق بود.")
            return 1
            
        # اجرای مأموریت بر اساس نوع
        success = False
        if args.mission == 'formation':
            params = FormationParameters(FormationType(args.formation), args.size, args.altitude, Orientation(args.orientation))
            success = await executor.execute_formation_mission(params)
        elif args.mission == 'transform':
            params = TransformationParameters(move_x=args.move_x, move_y=args.move_y, rotate_z=args.rotate_z)
            success = await executor.execute_transformation_mission(params)
        elif args.mission == 'leader_follower':
            success = await executor.execute_leader_follower_mission(args.formation, args.size, args.path_file)
        elif args.mission == 'leader_removal':
            success = await executor.execute_leader_removal_mission()
        elif args.mission == 'complete':
            success = await executor.run_complete_mission_sequence()
            
        # خاتمه سیستم
        await executor.shutdown_system()
        
        if success:
            print("\n🎉 ماموریت با موفقیت به پایان رسید!")
            return 0
        else:
            print("\n❌ ماموریت ناموفق بود.")
            return 1
            
    except Exception as e:
        print(f"\n❌ خطای غیرمنتظره و بحرانی: {e}")
        if executor.is_running:
            await executor.shutdown_system()
        return 1

if __name__ == "__main__":
    if sys.platform.startswith('win'):
        asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
    sys.exit(asyncio.run(main()))