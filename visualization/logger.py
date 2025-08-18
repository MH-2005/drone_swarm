"""
سیستم لاگ‌گیری حرفه‌ای برای مأموریت‌های پرواز گروهی
"""

import asyncio
import json
import csv
import time
import os
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
import logging


@dataclass
class LogEntry:
    """ورودی لاگ"""
    timestamp: float
    event_type: str
    drone_id: Optional[int]
    data: Dict[str, Any]
    severity: str = "INFO"


class MissionLogger:
    """کلاس اصلی لاگ‌گیری مأموریت"""
    
    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # ایجاد پوشه لاگ
        os.makedirs(log_dir, exist_ok=True)
        
        # لیست‌های لاگ
        self.events: List[LogEntry] = []
        self.position_logs: List[Dict] = []
        self.formation_logs: List[Dict] = []
        self.leader_changes: List[Dict] = []
        self.errors: List[Dict] = []
        
        # تنظیم لاگر سیستم
        self._setup_system_logger()
        
        self.logger.info(f"سیستم لاگ‌گیری راه‌اندازی شد - Session: {self.session_id}")
        
    def _setup_system_logger(self):
        """تنظیم لاگر سیستم Python"""
        log_file = os.path.join(self.log_dir, f"system_{self.session_id}.log")
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file, encoding='utf-8'),
                logging.StreamHandler()
            ]
        )
        
        self.logger = logging.getLogger('SwarmMission')
        
    async def log_event(self, event_type: str, data: Dict[str, Any], 
                       drone_id: Optional[int] = None, severity: str = "INFO"):
        """ثبت یک رویداد"""
        entry = LogEntry(
            timestamp=time.time(),
            event_type=event_type,
            drone_id=drone_id,
            data=data.copy(),
            severity=severity
        )
        
        self.events.append(entry)
        
        # لاگ در سیستم
        log_msg = f"[{event_type}]"
        if drone_id is not None:
            log_msg += f"[Drone {drone_id}]"
        log_msg += f" {data}"
        
        if severity == "ERROR":
            self.logger.error(log_msg)
        elif severity == "WARNING":
            self.logger.warning(log_msg)
        else:
            self.logger.info(log_msg)
            
    async def log_positions(self, positions: Dict[int, Dict], leader_id: Optional[int] = None):
        """لاگ موقعیت‌های پهپادها"""
        log_entry = {
            'timestamp': time.time(),
            'positions': positions.copy(),
            'leader_id': leader_id
        }
        
        self.position_logs.append(log_entry)
        
    async def log_formation_change(self, formation_type: str, formation_size: float, 
                                  success: bool, details: Dict = None):
        """لاگ تغییر آرایش"""
        log_entry = {
            'timestamp': time.time(),
            'formation_type': formation_type,
            'formation_size': formation_size,
            'success': success,
            'details': details or {}
        }
        
        self.formation_logs.append(log_entry)
        
        await self.log_event(
            "formation_change",
            log_entry,
            severity="INFO" if success else "WARNING"
        )
        
    async def log_leader_change(self, old_leader: Optional[int], new_leader: int, reason: str = ""):
        """لاگ تغییر رهبر"""
        log_entry = {
            'timestamp': time.time(),
            'old_leader': old_leader,
            'new_leader': new_leader,
            'reason': reason
        }
        
        self.leader_changes.append(log_entry)
        
        await self.log_event(
            "leader_change",
            log_entry,
            drone_id=new_leader,
            severity="INFO"
        )
        
    async def log_error(self, error_type: str, message: str, drone_id: Optional[int] = None, 
                       details: Dict = None):
        """لاگ خطا"""
        error_entry = {
            'timestamp': time.time(),
            'error_type': error_type,
            'message': message,
            'drone_id': drone_id,
            'details': details or {}
        }
        
        self.errors.append(error_entry)
        
        await self.log_event(
            "error",
            error_entry,
            drone_id=drone_id,
            severity="ERROR"
        )
        
    async def save_logs(self, prefix: str = "mission"):
        """ذخیره همه لاگ‌ها در فایل‌های مختلف"""
        try:
            base_name = f"{prefix}_{self.session_id}"
            
            # ذخیره رویدادها در JSON
            events_file = os.path.join(self.log_dir, f"{base_name}_events.json")
            events_data = [asdict(event) for event in self.events]
            
            with open(events_file, 'w', encoding='utf-8') as f:
                json.dump(events_data, f, indent=2, ensure_ascii=False)
                
            # ذخیره موقعیت‌ها در CSV
            positions_file = os.path.join(self.log_dir, f"{base_name}_positions.csv")
            await self._save_positions_csv(positions_file)
            
            # ذخیره تغییرات آرایش
            formation_file = os.path.join(self.log_dir, f"{base_name}_formations.json")
            with open(formation_file, 'w', encoding='utf-8') as f:
                json.dump(self.formation_logs, f, indent=2, ensure_ascii=False)
                
            # ذخیره تغییرات رهبر
            leaders_file = os.path.join(self.log_dir, f"{base_name}_leaders.json")
            with open(leaders_file, 'w', encoding='utf-8') as f:
                json.dump(self.leader_changes, f, indent=2, ensure_ascii=False)
                
            # ذخیره خطاها
            errors_file = os.path.join(self.log_dir, f"{base_name}_errors.json")
            with open(errors_file, 'w', encoding='utf-8') as f:
                json.dump(self.errors, f, indent=2, ensure_ascii=False)
                
            # تولید گزارش خلاصه
            summary_file = os.path.join(self.log_dir, f"{base_name}_summary.json")
            await self._generate_summary_report(summary_file)
            
            self.logger.info(f"همه لاگ‌ها در {self.log_dir} ذخیره شد")
            
        except Exception as e:
            self.logger.error(f"خطا در ذخیره لاگ‌ها: {str(e)}")
            
    async def _save_positions_csv(self, filename: str):
        """ذخیره موقعیت‌ها در فرمت CSV"""
        if not self.position_logs:
            return
            
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            fieldnames = ['timestamp', 'drone_id', 'latitude', 'longitude', 'altitude', 'is_leader']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            for log_entry in self.position_logs:
                timestamp = log_entry['timestamp']
                leader_id = log_entry.get('leader_id')
                
                for drone_id, position in log_entry['positions'].items():
                    writer.writerow({
                        'timestamp': timestamp,
                        'drone_id': drone_id,
                        'latitude': position.get('latitude', 0),
                        'longitude': position.get('longitude', 0),
                        'altitude': position.get('altitude', 0),
                        'is_leader': drone_id == leader_id
                    })
                    
    async def _generate_summary_report(self, filename: str):
        """تولید گزارش خلاصه"""
        try:
            # محاسبه آمار کلی
            total_events = len(self.events)
            total_errors = len(self.errors)
            total_formations = len(self.formation_logs)
            total_leader_changes = len(self.leader_changes)
            
            # زمان‌بندی
            if self.events:
                start_time = min(event.timestamp for event in self.events)
                end_time = max(event.timestamp for event in self.events)
                duration = end_time - start_time
            else:
                start_time = end_time = duration = 0
                
            # آمار موفقیت
            successful_formations = len([f for f in self.formation_logs if f.get('success', False)])
            formation_success_rate = (successful_formations / total_formations * 100) if total_formations > 0 else 0
            
            # آمار خطاها
            error_types = {}
            for error in self.errors:
                error_type = error.get('error_type', 'unknown')
                error_types[error_type] = error_types.get(error_type, 0) + 1
                
            summary = {
                'session_id': self.session_id,
                'mission_duration': duration,
                'start_time': start_time,
                'end_time': end_time,
                'statistics': {
                    'total_events': total_events,
                    'total_errors': total_errors,
                    'total_formations': total_formations,
                    'successful_formations': successful_formations,
                    'formation_success_rate': round(formation_success_rate, 2),
                    'total_leader_changes': total_leader_changes
                },
                'error_breakdown': error_types,
                'timeline': {
                    'first_event': datetime.fromtimestamp(start_time).isoformat() if start_time > 0 else None,
                    'last_event': datetime.fromtimestamp(end_time).isoformat() if end_time > 0 else None
                }
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
                
        except Exception as e:
            self.logger.error(f"خطا در تولید گزارش خلاصه: {str(e)}")
            
    def get_statistics(self) -> Dict[str, Any]:
        """دریافت آمار فعلی"""
        return {
            'session_id': self.session_id,
            'total_events': len(self.events),
            'total_errors': len(self.errors),
            'total_formations': len(self.formation_logs),
            'total_leader_changes': len(self.leader_changes),
            'total_position_logs': len(self.position_logs)
        }
        
    async def export_for_analysis(self, format_type: str = "csv") -> str:
        """صادرات داده‌ها برای تجزیه و تحلیل"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            if format_type == "csv":
                filename = os.path.join(self.log_dir, f"analysis_data_{timestamp}.csv")
                await self._export_csv_for_analysis(filename)
            elif format_type == "json":
                filename = os.path.join(self.log_dir, f"analysis_data_{timestamp}.json")
                await self._export_json_for_analysis(filename)
            else:
                raise ValueError(f"فرمت نامعتبر: {format_type}")
                
            self.logger.info(f"داده‌ها برای تجزیه و تحلیل در {filename} صادر شد")
            return filename
            
        except Exception as e:
            self.logger.error(f"خطا در صادرات داده‌ها: {str(e)}")
            raise
            
    async def _export_csv_for_analysis(self, filename: str):
        """صادرات CSV برای تجزیه و تحلیل"""
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            fieldnames = [
                'timestamp', 'event_type', 'drone_id', 'latitude', 'longitude', 
                'altitude', 'formation_type', 'leader_id', 'success', 'error_type'
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            # ترکیب همه داده‌ها
            for event in self.events:
                row = {
                    'timestamp': event.timestamp,
                    'event_type': event.event_type,
                    'drone_id': event.drone_id,
                    'success': event.data.get('success'),
                    'error_type': event.data.get('error_type'),
                    'formation_type': event.data.get('formation_type'),
                    'leader_id': event.data.get('leader_id')
                }
                
                # اضافه کردن موقعیت اگر موجود باشد
                if 'positions' in event.data and event.drone_id:
                    pos = event.data['positions'].get(str(event.drone_id), {})
                    row.update({
                        'latitude': pos.get('latitude'),
                        'longitude': pos.get('longitude'),
                        'altitude': pos.get('altitude')
                    })
                    
                writer.writerow(row)
                
    async def _export_json_for_analysis(self, filename: str):
        """صادرات JSON برای تجزیه و تحلیل"""
        analysis_data = {
            'metadata': {
                'session_id': self.session_id,
                'export_time': time.time(),
                'total_events': len(self.events)
            },
            'events': [asdict(event) for event in self.events],
            'positions': self.position_logs,
            'formations': self.formation_logs,
            'leader_changes': self.leader_changes,
            'errors': self.errors
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(analysis_data, f, indent=2, ensure_ascii=False)