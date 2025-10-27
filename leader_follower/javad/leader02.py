#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
leader01.py
Leader ساده که telemetry را از MAVSDK می‌خواند و به صورت UDP (broadcast یا unicast) منتشر می‌کند.

تنظیمات:
- CONNECTION_STRING: آدرس اتصال MAVSDK (udpin://:14541 یا udp://:14541)
- BROADCAST_ADDR: آدرس broadcast یا هدف (مثلاً "255.255.255.255" یا "127.0.0.1")
- BROADCAST_PORT: پورت که followerها گوش می‌دهند (مثلاً 5005)
"""
import asyncio
import json
import logging
import socket
import time
from mavsdk import System

# -------- CONFIG ----------
CONNECTION_STRING = "udp://:14540"   # تغییر بده مطابق instance PX4 (leader)
BROADCAST_ADDR = "10.37.247.166"     # یا "127.0.0.1" برای لوپ‌بک محلی
BROADCAST_PORT = 5005
BROADCAST_RATE_HZ = 20
LEADER_ID = "leader-1"
# --------------------------

logging.basicConfig(level=logging.INFO, format="%(asctime)s [LEADER] %(message)s")
log = logging.getLogger("leader01")


class Leader:
    def __init__(self):
        self.drone = System()
        self.telemetry_data = {}
        self._seq = 0
        self._stop = False

        # UDP socket for broadcast/unicast
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # allow broadcast
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            log.debug("Couldn't set SO_BROADCAST (platform/permission?)")
        self.target = (BROADCAST_ADDR, BROADCAST_PORT)

    async def run(self):
        log.info("🚀 Leader Starting...")
        await self.drone.connect(system_address=CONNECTION_STRING)

        # wait until connected
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log.info("✅ Flight controller connected")
                break

        # start telemetry tasks
        tasks = [
            asyncio.create_task(self._update_position()),
            asyncio.create_task(self._update_in_air_status()),
            asyncio.create_task(self._update_gps_info()),
        ]

        # wait for first position
        log.info("🛰️ Waiting for first position...")
        while 'latitude_deg' not in self.telemetry_data and not self._stop:
            await asyncio.sleep(0.05)

        log.info("📡 Position acquired - starting broadcast")

        try:
            await self._broadcast_loop()
        finally:
            self._stop = True
            for t in tasks:
                t.cancel()
            try:
                self.sock.close()
            except Exception:
                pass
            log.info("🛑 Leader stopped")

    async def _update_position(self):
        try:
            async for p in self.drone.telemetry.position():
                self.telemetry_data.update({
                    "latitude_deg": p.latitude_deg,
                    "longitude_deg": p.longitude_deg,
                    "relative_altitude_m": p.relative_altitude_m
                })
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"Position task error: {e}")

    async def _update_in_air_status(self):
        try:
            async for v in self.drone.telemetry.in_air():
                self.telemetry_data['is_in_air'] = bool(v)
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"In-air task error: {e}")

    async def _update_gps_info(self):
        try:
            async for g in self.drone.telemetry.gps_info():
                # num_satellites may be used by followers for quality
                self.telemetry_data['num_satellites'] = g.num_satellites
        except asyncio.CancelledError:
            return
        except Exception as e:
            log.error(f"GPS task error: {e}")

    async def _broadcast_loop(self):
        interval = 1.0 / BROADCAST_RATE_HZ
        while not self._stop:
            self._seq += 1
            payload = {
                "sender_type": "leader",
                "id": LEADER_ID,
                "seq": self._seq,
                "timestamp": time.time(),
                "telemetry": dict(self.telemetry_data)
            }
            try:
                b = json.dumps(payload).encode("utf-8")
                self.sock.sendto(b, self.target)
            except Exception as e:
                log.error(f"UDP send error: {e}")

            # debug logging
            lat = self.telemetry_data.get("latitude_deg")
            lon = self.telemetry_data.get("longitude_deg")
            alt = self.telemetry_data.get("relative_altitude_m")
            sats = self.telemetry_data.get("num_satellites")
            in_air = self.telemetry_data.get("is_in_air")
            log.debug(f"TX seq={self._seq} lat={lat} lon={lon} alt={alt} in_air={in_air} sats={sats}")
            if self._seq % (BROADCAST_RATE_HZ * 5) == 0:
                log.info(f"Broadcast seq={self._seq} lat={lat:.6f} lon={lon:.6f} alt={alt:.2f} sats={sats}")

            await asyncio.sleep(interval)


if __name__ == "__main__":
    try:
        leader = Leader()
        asyncio.run(leader.run())
    except KeyboardInterrupt:
        log.info("Interrupted by user")
