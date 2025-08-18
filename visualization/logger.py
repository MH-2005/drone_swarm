"""
کلاس ثبت‌کننده وقایع ماموریت (Mission Logger)
تمام رویدادهای مهم، آمارها و موقعیت‌ها را در فایل‌های JSON ذخیره می‌کند.
"""

import asyncio
import json
import time
import os
from datetime import datetime
from typing import Dict, Any

class MissionLogger:
    """ثبت‌کننده رویدادها و داده‌های ماموریت."""
    
    def __init__(self, log_dir: str = "mission_logs"):
        self.log_dir = log_dir
        self.session_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_path = os.path.join(log_dir, self.session_id)
        self.event_log_file = os.path.join(self.session_path, "events.jsonl")
        
        self._setup_logging()
        self.log_queue = asyncio.Queue()
        self.worker_task = asyncio.create_task(self._log_worker())
        
        print(f"✅ ثبت‌کننده وقایع آماده شد. لاگ‌ها در مسیر '{self.session_path}' ذخیره می‌شوند.")

    def _setup_logging(self):
        """ایجاد پوشه‌های مورد نیاز برای ذخیره لاگ‌ها."""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        if not os.path.exists(self.session_path):
            os.makedirs(self.session_path)

    async def log_event(self, event_type: str, data: Dict[str, Any]):
        """
        یک رویداد را به صورت غیرهمزمان به صف لاگ اضافه می‌کند.
        این کار از بلاک شدن برنامه اصلی برای نوشتن در فایل جلوگیری می‌کند.
        """
        log_entry = {
            "timestamp": time.time(),
            "event_type": event_type,
            "data": data
        }
        await self.log_queue.put(log_entry)

    async def _log_worker(self):
        """
        یک وظیفه پس‌زمینه که به طور مداوم لاگ‌ها را از صف برداشته و در فایل می‌نویسد.
        """
        try:
            while True:
                log_entry = await self.log_queue.get()
                try:
                    with open(self.event_log_file, "a", encoding="utf-8") as f:
                        f.write(json.dumps(log_entry) + "\n")
                except IOError as e:
                    print(f"❌ خطا در نوشتن لاگ: {e}")
                finally:
                    self.log_queue.task_done()
        except asyncio.CancelledError:
            print("worker لاگ متوقف شد.")

    async def shutdown(self):
        """خاتمه امن ثبت‌کننده وقایع."""
        print("در حال ذخیره لاگ‌های باقی‌مانده...")
        await self.log_queue.join()  # منتظر می‌ماند تا صف خالی شود
        self.worker_task.cancel()
        try:
            await self.worker_task
        except asyncio.CancelledError:
            pass
        print("ثبت وقایع با موفقیت خاتمه یافت.")