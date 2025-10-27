#!/usr/bin/env python3
"""
Leader: دریافت تله‌متری از شبیه‌ساز و پخش JSON با UDP
ارسال با فرکانس BROADCAST_RATE_HZ و افزودن timestamp و seq برای ردیابی
"""
import asyncio
import logging
import socket
import json
import time
from mavsdk import System

# ==== پیکربندی ====
CONNECTION_STRING = "udp://:14541"  # اتصال به شبیه‌ساز
FOLLOWER_IP = "127.0.0.3"       # آدرس فالوور (قابل تغییر)
BROADCAST_PORT = 5005
BROADCAST_RATE_HZ = 20

# ==== لاگینگ (تنها یک‌بار) ====
logging.basicConfig(level=logging.INFO, format="%(asctime)s [LEADER] %(message)s")
log = logging.getLogger("leader")

class Leader:
    def __init__(self):
        self.drone = System()
        self.telemetry_data = {}
        self._stop = False

        # UDP socket (برای ارسال ساده و قابل فهم)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # اگر بخواهی broadcast کنی می‌تونی setsockopt بگذاری؛ الان مستقیم به follower می‌فرستیم
        self._target = (FOLLOWER_IP, BROADCAST_PORT)
        self._seq = 0

    async def run(self):
        log.info("🚀 Leader starting...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        # منتظر برقراری ارتباط با flight controller
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log.info("✅ Flight controller connected")
                break

        # start tasks to collect telemetry
        tasks = [
            asyncio.create_task(self._update_position()),
            asyncio.create_task(self._update_in_air_status()),
            asyncio.create_task(self._update_gps_info()),
        ]

        # wait until we have at least one position
        log.info("🛰️ Waiting for first position...")
        while 'latitude_deg' not in self.telemetry_data and not self._stop:
            await asyncio.sleep(0.1)

        log.info("📡 Position acquired - starting broadcast")
        try:
            await self._broadcast_loop()
        finally:
            self._stop = True
            for t in tasks:
                t.cancel()
            self._sock.close()
            log.info("🛑 Leader stopped and socket closed")

    async def _update_position(self):
        try:
            async for pos in self.drone.telemetry.position():
                self.telemetry_data.update({
                    "latitude_deg": pos.latitude_deg,
                    "longitude_deg": pos.longitude_deg,
                    "relative_altitude_m": pos.relative_altitude_m
                })
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"❌ Position task error: {e}")

    async def _update_in_air_status(self):
        try:
            async for in_air in self.drone.telemetry.in_air():
                self.telemetry_data['is_in_air'] = bool(in_air)
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"❌ In-air task error: {e}")

    async def _update_gps_info(self):
        try:
            async for gps in self.drone.telemetry.gps_info():
                self.telemetry_data['num_satellites'] = gps.num_satellites
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"❌ GPS task error: {e}")

    async def _broadcast_loop(self):
        interval = 1.0 / BROADCAST_RATE_HZ
        while not self._stop:
            self._seq += 1
            payload = {
                "seq": self._seq,
                "timestamp": time.time(),
                # make a shallow copy of telemetry data to avoid race conditions
                "telemetry": dict(self.telemetry_data)
            }
            try:
                msg = json.dumps(payload).encode("utf-8")
                # sendto is blocking but small; if desired can be made async
                self._sock.sendto(msg, self._target)
            except Exception as e:
                log.error(f"❌ UDP send error: {e}")
            await asyncio.sleep(interval)


if __name__ == "__main__":
    try:
        leader = Leader()
        asyncio.run(leader.run())
    except KeyboardInterrupt:
        log.info("🛑 Leader interrupted by user")
