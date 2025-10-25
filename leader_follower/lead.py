#!/usr/bin/env python3

import asyncio
import json
import socket
import logging
import time
from mavsdk import System
from mavsdk.telemetry import Position, GpsInfo

class LeaderSim:
    def __init__(self):
        self.setup_logging()
        
        # شبکه
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.follower_addr = ('127.0.0.1', 5005)
        
        # MAVSDK
        self.drone = System()
        
        # داده‌ها
        self.telemetry = {
            'latitude_deg': 0.0,
            'longitude_deg': 0.0, 
            'relative_altitude_m': 0.0,
            'absolute_altitude_m': 0.0,
            'is_in_air': False,
            'num_satellites': 0,
            'timestamp': 0.0,
            'packet_id': 0
        }
        
        self.is_running = True
        self.log.info("🚀 Leader Sim Initialized")

    def setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.log = logging.getLogger('LeaderSim')

    async def connect_to_sim(self):
        """اتصال به شبیه‌ساز PX4"""
        self.log.info("Connecting to PX4 SITL...")
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("✅ Connected to PX4 SITL!")
                break

    async def update_telemetry(self):
        """آپدیت داده‌های تلهمتری"""
        async for position in self.drone.telemetry.position():
            self.telemetry.update({
                'latitude_deg': position.latitude_deg,
                'longitude_deg': position.longitude_deg,
                'relative_altitude_m': position.relative_altitude_m,
                'absolute_altitude_m': position.absolute_altitude_m,
                'timestamp': time.time()
            })

        async for in_air in self.drone.telemetry.in_air():
            self.telemetry['is_in_air'] = in_air
            if in_air:
                self.log.info("✈️ Leader is AIRBORNE")
            else:
                self.log.info("🛬 Leader is on GROUND")

        async for gps_info in self.drone.telemetry.gps_info():
            self.telemetry['num_satellites'] = gps_info.num_satellites

    async def broadcast_to_follower(self):
        """ارسال داده به فالوور"""
        self.log.info("Starting broadcast to follower...")
        
        while self.is_running:
            try:
                self.telemetry['packet_id'] += 1
                message = json.dumps(self.telemetry).encode('utf-8')
                self.sock.sendto(message, self.follower_addr)
                
                if self.telemetry['packet_id'] % 20 == 0:
                    self.log.info(
                        f"📤 Packet {self.telemetry['packet_id']}: "
                        f"Alt={self.telemetry['relative_altitude_m']:.1f}m, "
                        f"SAT={self.telemetry['num_satellites']}"
                    )
                
                await asyncio.sleep(0.05)  # 20 Hz
                
            except Exception as e:
                self.log.error(f"Broadcast error: {e}")
                await asyncio.sleep(0.1)

    async def run(self):
        """اجرای اصلی"""
        await self.connect_to_sim()
        
        # شروع tasks
        telemetry_task = asyncio.create_task(self.update_telemetry())
        broadcast_task = asyncio.create_task(self.broadcast_to_follower())
        
        self.log.info("✅ Leader SIM is fully operational!")
        
        try:
            await asyncio.gather(telemetry_task, broadcast_task)
        except KeyboardInterrupt:
            self.log.info("Shutting down...")
        finally:
            self.is_running = False
            self.sock.close()

if __name__ == "__main__":
    leader = LeaderSim()
    asyncio.run(leader.run())