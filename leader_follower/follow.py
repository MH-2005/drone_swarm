#!/usr/bin/env python3

import asyncio
import json
import math
import socket
import logging
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from enum import Enum

class FollowerState(Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
    FOLLOWING = "FOLLOWING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"

class FollowerSim:
    def __init__(self):
        self.setup_logging()
        
        # شبکه
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.settimeout(1.0)
        
        # MAVSDK
        self.drone = System()
        
        # وضعیت
        self.state = FollowerState.IDLE
        self.leader_data = {}
        self.last_leader_time = 0
        self.packet_count = 0
        
        # تنظیمات
        self.config = {
            'offset_north_m': -5.0,
            'offset_east_m': 0.0,
            'flight_speed': 2.0,
            'altitude_p_gain': 1.0,
            'connection_timeout': 3.0,
            'takeoff_trigger_alt': 2.0,
            'min_satellites': 8
        }
        
        self.is_running = True
        self.log.info("🎯 Follower Sim Initialized")

    def setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.log = logging.getLogger('FollowerSim')

    async def connect_to_sim(self):
        """اتصال به شبیه‌ساز PX4"""
        self.log.info("Connecting to PX4 SITL...")
        await self.drone.connect(system_address="udp://:14541")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("✅ Connected to PX4 SITL!")
                break

    async def listen_for_leader(self):
        """گوش دادن به داده‌های لیدر"""
        self.log.info("👂 Listening for leader on port 5005...")
        
        while self.is_running:
            try:
                data, addr = self.sock.recvfrom(4096)
                self.leader_data = json.loads(data.decode())
                self.last_leader_time = time.time()
                self.packet_count += 1
                
                if self.packet_count % 20 == 0:
                    alt = self.leader_data.get('relative_altitude_m', 0)
                    sats = self.leader_data.get('num_satellites', 0)
                    self.log.info(f"📨 Leader: Alt={alt:.1f}m, Packets={self.packet_count}")
                    
            except socket.timeout:
                continue
            except Exception as e:
                self.log.error(f"Listen error: {e}")

    def safety_checks(self):
        """بررسی‌های ایمنی"""
        if time.time() - self.last_leader_time > self.config['connection_timeout']:
            self.log.error("🚨 Lost connection to leader!")
            return False
            
        if self.leader_data.get('num_satellites', 0) < self.config['min_satellites']:
            self.log.warning("⚠️ Poor leader GPS")
            return False
            
        return True

    async def follow_leader(self):
        """منطق دنبال‌کنی لیدر"""
        self.log.info("🔄 Starting follow logic...")
        
        while self.is_running:
            try:
                # بررسی ایمنی
                if not self.safety_checks():
                    self.state = FollowerState.EMERGENCY
                    await asyncio.sleep(1)
                    continue
                
                # دریافت وضعیت فعلی
                current_pos = await self.drone.telemetry.position().__anext__()
                leader_alt = self.leader_data.get('relative_altitude_m', 0)
                leader_in_air = self.leader_data.get('is_in_air', False)
                am_i_in_air = await self.drone.telemetry.in_air().__anext__()
                
                # مدیریت حالت‌ها
                if self.state == FollowerState.IDLE:
                    if leader_in_air and leader_alt > self.config['takeoff_trigger_alt'] and not am_i_in_air:
                        self.log.info("🚀 Leader airborne - Taking off!")
                        await self.takeoff(leader_alt)
                        self.state = FollowerState.FOLLOWING
                        
                elif self.state == FollowerState.FOLLOWING:
                    if not leader_in_air and am_i_in_air:
                        self.log.info("🛬 Leader landing - Following to land")
                        await self.land()
                    else:
                        await self.maintain_formation()
                        
                elif self.state == FollowerState.EMERGENCY:
                    await self.emergency_procedure()
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.log.error(f"Follow error: {e}")
                await asyncio.sleep(0.1)

    async def takeoff(self, target_altitude):
        """برخاستن به ارتفاع مشخص"""
        self.log.info(f"🛫 Taking off to {target_altitude}m...")
        
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        
        # منتظر رسیدن به ارتفاع بمان
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= target_altitude - 0.5:
                self.log.info("✅ Reached target altitude")
                break
            await asyncio.sleep(0.1)

    async def land(self):
        """فرود"""
        self.log.info("🛬 Landing...")
        await self.drone.action.land()
        
        # منتظر فرود کامل بمان
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                self.log.info("✅ Landed successfully")
                await self.drone.action.disarm()
                self.state = FollowerState.IDLE
                break
            await asyncio.sleep(0.1)

    async def maintain_formation(self):
        """حفظ فاصله در تشکیل"""
        try:
            # شروع Offboard mode اگر لازم است
            flight_mode = str(await self.drone.telemetry.flight_mode().__anext__())
            if flight_mode != "OFFBOARD":
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                await self.drone.offboard.start()
            
            # محاسبه موقعیت هدف
            leader_lat = self.leader_data['latitude_deg']
            leader_lon = self.leader_data['longitude_deg']
            leader_alt = self.leader_data['relative_altitude_m']
            
            lat_offset = self.config['offset_north_m'] / 111320.0
            lon_offset = self.config['offset_east_m'] / (111320.0 * math.cos(math.radians(leader_lat)))
            
            target_lat = leader_lat + lat_offset
            target_lon = leader_lon + lon_offset
            
            # دریافت موقعیت فعلی
            current_pos = await self.drone.telemetry.position().__anext__()
            
            # محاسبه سرعت‌ها
            remaining_north = (target_lat - current_pos.latitude_deg) * 111320.0
            remaining_east = (target_lon - current_pos.longitude_deg) * 111320.0 * math.cos(math.radians(current_pos.latitude_deg))
            altitude_error = leader_alt - current_pos.relative_altitude_m
            
            velocity_n = remaining_north * self.config['flight_speed']
            velocity_e = remaining_east * self.config['flight_speed']
            velocity_d = -altitude_error * self.config['altitude_p_gain']  # منفی برای NED
            
            # محدود کردن سرعت
            max_speed = self.config['flight_speed']
            velocity_n = max(-max_speed, min(max_speed, velocity_n))
            velocity_e = max(-max_speed, min(max_speed, velocity_e))
            velocity_d = max(-max_speed, min(max_speed, velocity_d))
            
            # ارسال دستور سرعت
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(velocity_n, velocity_e, velocity_d, 0.0)
            )
            
        except OffboardError as e:
            self.log.error(f"Offboard error: {e}")
        except Exception as e:
            self.log.error(f"Formation error: {e}")

    async def emergency_procedure(self):
        """روال اضطراری"""
        self.log.error("🚨 Executing emergency procedure!")
        try:
            await self.drone.action.hold()
            await asyncio.sleep(1)
        except Exception as e:
            self.log.error(f"Emergency procedure error: {e}")

    async def run(self):
        """اجرای اصلی"""
        await self.connect_to_sim()
        
        # شروع tasks
        listener_task = asyncio.create_task(self.listen_for_leader())
        follow_task = asyncio.create_task(self.follow_leader())
        
        self.log.info("✅ Follower SIM is fully operational!")
        
        try:
            await asyncio.gather(listener_task, follow_task)
        except KeyboardInterrupt:
            self.log.info("Shutting down...")
        finally:
            self.is_running = False
            self.sock.close()

if __name__ == "__main__":
    follower = FollowerSim()
    asyncio.run(follower.run())