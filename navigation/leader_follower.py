"""
سیستم رهبر-پیرو برای مرحله سوم و چهارم شیوه‌نامه
پیاده‌سازی الگوریتم پیروی هوشمند با حفظ آرایش
"""

import asyncio
import math
import time
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, asdict
from enum import Enum

from ..core.drone import Drone, Position, DroneState
from ..core.swarm_manager import SwarmManager
from .path_manager import PathManager, PathPoint


class FollowMode(Enum):
    """حالت‌های پیروی"""
    FORMATION_KEEP = "formation_keep"  # حفظ آرایش
    ADAPTIVE = "adaptive"              # تطبیق با شرایط
    CLOSE_FOLLOW = "close_follow"      # پیروی نزدیک


@dataclass
class FollowerConfig:
    """تنظیمات پیروها"""
    follow_distance: float = 5.0      # فاصله پیروی از رهبر
    formation_type: str = "triangle"   # نوع آرایش
    formation_size: float = 8.0        # اندازه آرایش
    max_speed: float = 8.0             # حداکثر سرعت
    update_rate: float = 2.0           # نرخ به‌روزرسانی (Hz)
    position_tolerance: float = 2.0    # تلرانس موقعیت
    leader_timeout: float = 10.0       # زمان انتظار برای رهبر


@dataclass
class FollowerState:
    """وضعیت هر پیرو"""
    drone_id: int
    offset_x: float = 0.0      # آفست نسبت به رهبر
    offset_y: float = 0.0
    offset_z: float = 0.0
    target_position: Optional[Position] = None
    last_update: float = 0.0
    is_active: bool = True


class LeaderFollowerController:
    """کنترلر اصلی سیستم رهبر-پیرو"""
    
    def __init__(self, swarm_manager: SwarmManager, path_manager: PathManager):
        self.swarm = swarm_manager
        self.path_manager = path_manager
        
        # تنظیمات
        self.config = FollowerConfig()
        self.follow_mode = FollowMode.FORMATION_KEEP
        
        # وضعیت پیروها
        self.followers_state: Dict[int, FollowerState] = {}
        
        # کنترل اجرا
        self.is_following = False
        self.follow_task: Optional[asyncio.Task] = None
        
        # آمار
        self.formation_errors = []
        self.leader_changes_log = []
        
        print("کنترلر رهبر-پیرو آماده شد")
        
    def setup_formation(self, formation_type: str, size: float, follow_distance: float = 5.0):
        """تنظیم آرایش پیروها"""
        self.config.formation_type = formation_type
        self.config.formation_size = size
        self.config.follow_distance = follow_distance
        
        # محاسبه آفست‌های پیروها
        self._calculate_follower_offsets()
        
        print(f"آرایش {formation_type} با اندازه {size}m و فاصله {follow_distance}m تنظیم شد")
        
    def _calculate_follower_offsets(self):
        """محاسبه آفست‌های پیروها نسبت به رهبر"""
        followers = self.swarm.get_followers()
        if not followers:
            return
            
        formation_positions = self._generate_formation_offsets(
            len(followers), 
            self.config.formation_type,
            self.config.formation_size
        )
        
        # تخصیص آفست به هر پیرو
        for i, follower in enumerate(followers):
            if i < len(formation_positions):
                offset_x, offset_y = formation_positions[i]
                
                self.followers_state[follower.id] = FollowerState(
                    drone_id=follower.id,
                    offset_x=offset_x,
                    offset_y=offset_y - self.config.follow_distance,  # فاصله اضافی از رهبر
                    offset_z=0.0
                )
                
        print(f"آفست‌های {len(self.followers_state)} پیرو محاسبه شد")
        
    def _generate_formation_offsets(self, num_followers: int, formation_type: str, size: float) -> List[Tuple[float, float]]:
        """تولید آفست‌های آرایش"""
        positions = []
        
        if formation_type == "line":
            # آرایش خطی
            spacing = size / max(1, num_followers - 1) if num_followers > 1 else 0
            start_x = -size / 2
            
            for i in range(num_followers):
                x = start_x + i * spacing
                positions.append((x, 0.0))
                
        elif formation_type == "triangle":
            # آرایش مثلثی
            if num_followers >= 1:
                positions.append((0.0, -size/2))  # پشت رهبر
            if num_followers >= 2:
                positions.append((-size/3, -size))  # چپ
            if num_followers >= 3:
                positions.append((size/3, -size))   # راست
                
            # پیروهای اضافی در ردیف دوم
            for i in range(3, num_followers):
                angle = (2 * math.pi * (i-3)) / max(1, num_followers - 3)
                radius = size * 0.7
                x = radius * math.cos(angle)
                y = -size * 1.3 + radius * math.sin(angle)
                positions.append((x, y))
                
        elif formation_type == "square":
            # آرایش مربعی
            half_size = size / 2
            square_positions = [
                (-half_size, -half_size),  # پشت چپ
                (half_size, -half_size),   # پشت راست
                (-half_size, -size),       # ردیف دوم چپ
                (half_size, -size)         # ردیف دوم راست
            ]
            
            for i in range(min(num_followers, len(square_positions))):
                positions.append(square_positions[i])
                
        elif formation_type == "v_shape":
            # آرایش V شکل
            for i in range(num_followers):
                side = i % 2  # چپ یا راست
                row = i // 2 + 1
                
                x = (side * 2 - 1) * row * size * 0.3  # -1 یا +1
                y = -row * size * 0.5
                positions.append((x, y))
                
        else:
            # پیش‌فرض: دایره‌ای
            if num_followers == 1:
                positions.append((0.0, -size/2))
            else:
                radius = size / 2
                for i in range(num_followers):
                    angle = (2 * math.pi * i) / num_followers + math.pi/2  # شروع از پشت
                    x = radius * math.cos(angle)
                    y = radius * math.sin(angle) - size/2
                    positions.append((x, y))
                    
        return positions
        
    async def start_following(self, leader_manual_control: bool = False) -> bool:
        """شروع حالت پیروی"""
        try:
            if self.is_following:
                print("پیروی قبلاً شروع شده است")
                return True
                
            leader = self.swarm.get_leader()
            if not leader:
                print("هیچ رهبری برای پیروی وجود ندارد")
                return False
                
            followers = self.swarm.get_followers()
            if not followers:
                print("هیچ پیروی وجود ندارد")
                return False
                
            print(f"شروع پیروی از رهبر {leader.id} با {len(followers)} پیرو")
            
            # تنظیم آرایش اولیه
            if not self.followers_state:
                self._calculate_follower_offsets()
                
            # شروع حلقه پیروی
            self.is_following = True
            
            if leader_manual_control:
                # حالت کنترل دستی رهبر
                self.follow_task = asyncio.create_task(self._follow_manual_leader())
            else:
                # حالت پیروی از مسیر
                self.follow_task = asyncio.create_task(self._follow_path_leader())
                
            return True
            
        except Exception as e:
            print(f"خطا در شروع پیروی: {str(e)}")
            return False
            
    async def stop_following(self):
        """توقف پیروی"""
        print("توقف پیروی...")
        self.is_following = False
        
        if self.follow_task:
            self.follow_task.cancel()
            try:
                await self.follow_task
            except asyncio.CancelledError:
                pass
            self.follow_task = None
            
        print("پیروی متوقف شد")
        
    async def _follow_manual_leader(self):
        """پیروی از رهبر در حالت کنترل دستی"""
        try:
            update_interval = 1.0 / self.config.update_rate
            
            while self.is_following:
                leader = self.swarm.get_leader()
                if not leader or not leader.is_alive():
                    print("رهبر از دست رفت، انتظار برای رهبر جدید...")
                    await asyncio.sleep(2.0)
                    continue
                    
                # دریافت موقعیت رهبر
                leader_pos = await leader.get_current_position()
                if not leader_pos:
                    await asyncio.sleep(update_interval)
                    continue
                    
                # محاسبه موقعیت‌های هدف پیروها
                await self._update_followers_targets(leader_pos)
                
                # حرکت پیروها
                await self._move_followers_to_targets()
                
                await asyncio.sleep(update_interval)
                
        except asyncio.CancelledError:
            print("پیروی دستی لغو شد")
        except Exception as e:
            print(f"خطا در پیروی دستی: {str(e)}")
            
    async def _follow_path_leader(self):
        """پیروی از رهبر در حالت مسیر از پیش تعریف شده"""
        try:
            update_interval = 1.0 / self.config.update_rate
            
            # شروع حرکت رهبر روی مسیر
            path_task = asyncio.create_task(self.path_manager.execute_path())
            
            while self.is_following:
                leader = self.swarm.get_leader()
                if not leader or not leader.is_alive():
                    print("رهبر از دست رفت، تلاش برای انتخاب رهبر جدید...")
                    self.swarm._select_new_leader()
                    
                    new_leader = self.swarm.get_leader()
                    if new_leader:
                        print(f"رهبر جدید: پهپاد {new_leader.id}")
                        # ادامه مسیر با رهبر جدید
                        self.path_manager.set_current_drone(new_leader)
                        self._log_leader_change(leader.id if leader else -1, new_leader.id)
                    
                    await asyncio.sleep(2.0)
                    continue
                    
                # دریافت موقعیت رهبر
                leader_pos = await leader.get_current_position()
                if leader_pos:
                    # به‌روزرسانی پیروها
                    await self._update_followers_targets(leader_pos)
                    await self._move_followers_to_targets()
                    
                await asyncio.sleep(update_interval)
                
        except asyncio.CancelledError:
            print("پیروی مسیری لغو شد")
        except Exception as e:
            print(f"خطا در پیروی مسیری: {str(e)}")
            
    async def _update_followers_targets(self, leader_position: Position):
        """محاسبه موقعیت‌های هدف پیروها"""
        try:
            # دریافت جهت حرکت رهبر (برای تطبیق آرایش)
            leader_heading = await self._get_leader_heading()
            
            for follower_id, state in self.followers_state.items():
                if not state.is_active:
                    continue
                    
                # محاسبه موقعیت هدف با در نظر گیری جهت رهبر
                target_pos = self._calculate_follower_target(
                    leader_position, 
                    state.offset_x, 
                    state.offset_y,
                    state.offset_z,
                    leader_heading
                )
                
                state.target_position = target_pos
                state.last_update = time.time()
                
        except Exception as e:
            print(f"خطا در محاسبه اهداف پیروها: {str(e)}")
            
    async def _get_leader_heading(self) -> float:
        """دریافت جهت حرکت رهبر"""
        leader = self.swarm.get_leader()
        if not leader:
            return 0.0
            
        try:
            # سعی در دریافت جهت از تله‌متری
            async for attitude in leader.system.telemetry.attitude_euler():
                return attitude.yaw_deg
        except:
            # در صورت عدم دسترسی، جهت پیش‌فرض
            return 0.0
            
    def _calculate_follower_target(self, 
                                 leader_pos: Position,
                                 offset_x: float, 
                                 offset_y: float, 
                                 offset_z: float,
                                 leader_heading: float = 0.0) -> Position:
        """محاسبه موقعیت هدف یک پیرو"""
        
        # چرخش آفست بر اساس جهت رهبر
        heading_rad = math.radians(leader_heading)
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        
        # آفست چرخیده
        rotated_x = offset_x * cos_h - offset_y * sin_h
        rotated_y = offset_x * sin_h + offset_y * cos_h
        
        # تبدیل به موقعیت جغرافیایی
        lat_offset = rotated_y / 111319.9  # متر به درجه
        lon_offset = rotated_x / (111319.9 * math.cos(math.radians(leader_pos.latitude)))
        
        target_lat = leader_pos.latitude + lat_offset
        target_lon = leader_pos.longitude + lon_offset
        target_alt = leader_pos.altitude + offset_z
        
        return Position(target_lat, target_lon, target_alt)
        
    async def _move_followers_to_targets(self):
        """حرکت پیروها به موقعیت‌های هدف"""
        movement_tasks = []
        
        for follower in self.swarm.get_followers():
            if follower.id in self.followers_state:
                state = self.followers_state[follower.id]
                if state.is_active and state.target_position:
                    task = self._move_single_follower(follower, state.target_position)
                    movement_tasks.append(task)
                    
        # اجرای همزمان حرکات (بدون انتظار برای اتمام)
        if movement_tasks:
            asyncio.gather(*movement_tasks, return_exceptions=True)
            
    async def _move_single_follower(self, follower: Drone, target: Position) -> bool:
        """حرکت یک پیرو به موقعیت هدف"""
        try:
            current_pos = await follower.get_current_position()
            if not current_pos:
                return False
                
            # بررسی نیاز به حرکت
            distance = current_pos.distance_to(target)
            if distance < self.config.position_tolerance:
                return True  # قبلاً در موقعیت مطلوب است
                
            # حرکت به هدف (non-blocking)
            await follower.goto_position(target, speed=self.config.max_speed)
            return True
            
        except Exception as e:
            print(f"خطا در حرکت پیرو {follower.id}: {str(e)}")
            return False
            
    def _log_leader_change(self, old_leader: int, new_leader: int):
        """ثبت تغییر رهبر"""
        self.leader_changes_log.append({
            'timestamp': time.time(),
            'old_leader': old_leader,
            'new_leader': new_leader
        })
        
    async def handle_leader_loss(self) -> bool:
        """مدیریت از دست رفتن رهبر"""
        print("🔄 مدیریت از دست رفتن رهبر...")
        
        # انتخاب رهبر جدید
        if not self.swarm._select_new_leader():
            print("نمی‌توان رهبر جدید انتخاب کرد")
            return False
            
        new_leader = self.swarm.get_leader()
        if not new_leader:
            return False
            
        print(f"رهبر جدید انتخاب شد: پهپاد {new_leader.id}")
        
        # به‌روزرسانی آفست‌ها برای رهبر جدید
        await self._recalculate_offsets_for_new_leader(new_leader)
        
        # ادامه مسیر
        if hasattr(self.path_manager, 'set_current_drone'):
            self.path_manager.set_current_drone(new_leader)
            
        return True
        
    async def _recalculate_offsets_for_new_leader(self, new_leader: Drone):
        """محاسبه مجدد آفست‌ها برای رهبر جدید"""
        try:
            # اگر رهبر جدید قبلاً پیرو بود، آفستش را حذف کن
            if new_leader.id in self.followers_state:
                del self.followers_state[new_leader.id]
                
            # آفست‌های باقی‌مانده را مجدداً محاسبه کن
            self._calculate_follower_offsets()
            
            print(f"آفست‌ها برای رهبر جدید {new_leader.id} محاسبه شد")
            
        except Exception as e:
            print(f"خطا در محاسبه مجدد آفست‌ها: {str(e)}")
            
    async def change_formation_during_flight(self, new_formation: str, new_size: float) -> bool:
        """تغییر آرایش در حین پرواز"""
        try:
            print(f"تغییر آرایش به {new_formation} با اندازه {new_size}")
            
            # ذخیره آرایش قبلی
            old_formation = self.config.formation_type
            old_size = self.config.formation_size
            
            # تنظیم آرایش جدید
            self.config.formation_type = new_formation
            self.config.formation_size = new_size
            
            # محاسبه آفست‌های جدید
            self._calculate_follower_offsets()
            
            # اعمال تدریجی آرایش جدید
            await self._transition_to_new_formation()
            
            self.swarm.log_formation_change()
            print("تغییر آرایش با موفقیت انجام شد")
            return True
            
        except Exception as e:
            print(f"خطا در تغییر آرایش: {str(e)}")
            # بازگشت به آرایش قبلی
            self.config.formation_type = old_formation
            self.config.formation_size = old_size
            self._calculate_follower_offsets()
            return False
            
    async def _transition_to_new_formation(self):
        """انتقال تدریجی به آرایش جدید"""
        leader = self.swarm.get_leader()
        if not leader:
            return
            
        leader_pos = await leader.get_current_position()
        if not leader_pos:
            return
            
        # محاسبه موقعیت‌های جدید
        await self._update_followers_targets(leader_pos)
        
        # حرکت آرام به موقعیت‌های جدید
        transition_tasks = []
        for follower in self.swarm.get_followers():
            if follower.id in self.followers_state:
                state = self.followers_state[follower.id]
                if state.target_position:
                    task = follower.goto_position(
                        state.target_position, 
                        speed=self.config.max_speed * 0.7  # سرعت کمتر برای انتقال نرم
                    )
                    transition_tasks.append(task)
                    
        # انتظار برای اتمام انتقال
        if transition_tasks:
            await asyncio.gather(*transition_tasks, return_exceptions=True)
            
    def get_formation_status(self) -> Dict:
        """دریافت وضعیت آرایش"""
        return {
            'formation_type': self.config.formation_type,
            'formation_size': self.config.formation_size,
            'follow_distance': self.config.follow_distance,
            'active_followers': len([s for s in self.followers_state.values() if s.is_active]),
            'total_followers': len(self.followers_state),
            'is_following': self.is_following,
            'leader_changes': len(self.leader_changes_log),
            'formation_errors': len(self.formation_errors)
        }
        
    async def monitor_formation_quality(self):
        """نظارت بر کیفیت آرایش"""
        while self.is_following:
            try:
                await self._check_formation_integrity()
                await asyncio.sleep(5.0)  # بررسی هر 5 ثانیه
            except Exception as e:
                print(f"خطا در نظارت آرایش: {str(e)}")
                await asyncio.sleep(5.0)
                
    async def _check_formation_integrity(self):
        """بررسی یکپارچگی آرایش"""
        leader = self.swarm.get_leader()
        followers = self.swarm.get_followers()
        
        if not leader or not followers:
            return
            
        leader_pos = await leader.get_current_position()
        if not leader_pos:
            return
            
        formation_errors = []
        
        for follower in followers:
            if follower.id not in self.followers_state:
                continue
                
            follower_pos = await follower.get_current_position()
            if not follower_pos:
                continue
                
            # محاسبه خطای موقعیت
            expected_pos = self.followers_state[follower.id].target_position
            if expected_pos:
                error_distance = follower_pos.distance_to(expected_pos)
                
                if error_distance > self.config.position_tolerance * 2:
                    formation_errors.append({
                        'drone_id': follower.id,
                        'error_distance': error_distance,
                        'timestamp': time.time()
                    })
                    
        if formation_errors:
            self.formation_errors.extend(formation_errors)
            print(f"⚠️ خطاهای آرایش: {len(formation_errors)} پهپاد خارج از موقعیت")
            
    def enable_adaptive_following(self, enable: bool = True):
        """فعال‌سازی پیروی تطبیقی"""
        if enable:
            self.follow_mode = FollowMode.ADAPTIVE
            print("حالت پیروی تطبیقی فعال شد")
        else:
            self.follow_mode = FollowMode.FORMATION_KEEP
            print("حالت پیروی ساده فعال شد")
            
    async def emergency_formation_recovery(self):
        """بازیابی اضطراری آرایش"""
        print("🚨 شروع بازیابی اضطراری آرایش")
        
        try:
            leader = self.swarm.get_leader()
            if not leader:
                print("هیچ رهبری برای بازیابی وجود ندارد")
                return False
                
            # توقف حرکات فعلی
            for follower in self.swarm.get_followers():
                try:
                    await follower.system.action.hold()
                except:
                    pass
                    
            await asyncio.sleep(2.0)
            
            # محاسبه مجدد آرایش
            self._calculate_follower_offsets()
            
            # حرکت آرام به آرایش صحیح
            leader_pos = await leader.get_current_position()
            if leader_pos:
                await self._update_followers_targets(leader_pos)
                await self._move_followers_to_targets()
                
            print("بازیابی آرایش انجام شد")
            return True
            
        except Exception as e:
            print(f"خطا در بازیابی آرایش: {str(e)}")
            return False
            
    def get_statistics(self) -> Dict:
        """آمار عملکرد سیستم رهبر-پیرو"""
        total_errors = len(self.formation_errors)
        avg_error = 0.0
        
        if self.formation_errors:
            avg_error = sum(e['error_distance'] for e in self.formation_errors) / total_errors
            
        return {
            'total_formation_errors': total_errors,
            'average_formation_error': round(avg_error, 2),
            'leader_changes': len(self.leader_changes_log),
            'following_duration': time.time() - (self.leader_changes_log[0]['timestamp'] if self.leader_changes_log else time.time()),
            'active_followers': len([s for s in self.followers_state.values() if s.is_active]),
            'formation_config': {
                'type': self.config.formation_type,
                'size': self.config.formation_size,
                'follow_distance': self.config.follow_distance
            }
        }
        
    async def cleanup(self):
        """پاکسازی منابع"""
        await self.stop_following()
        self.followers_state.clear()
        self.formation_errors.clear()
        self.leader_changes_log.clear()
        print("منابع کنترلر رهبر-پیرو پاک شد")