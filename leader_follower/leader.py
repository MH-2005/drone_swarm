#!/usr/bin/env python3

#
# leader_final.py
#
# Description:
# Runs on the manually piloted Leader drone. Its sole responsibility is to
# broadcast a "vitals" packet (position, altitude, in-air status, and GPS
# quality) via UDP for the Follower drone to receive and act upon.
#

import asyncio
import logging
import socket
import json
from mavsdk import System

# --- Configuration ---
LOG_FILE = "leader.log"
CONNECTION_STRING = "udp://:14541"
# Ensure this is the static IP of your Follower drone's Orange Pi
FOLLOWER_IP = "192.168.1.2"
BROADCAST_PORT = 5005
BROADCAST_RATE_HZ = 20  # 20 Hz is a good rate for smooth following

class Leader:
    """The flying beacon that broadcasts its vitals."""
    def __init__(self):
        self.drone = System()
        self.log = logging.getLogger(__name__)
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = (FOLLOWER_IP, BROADCAST_PORT)
        self.telemetry_data = {}

    async def run(self):
        """Main loop to connect and broadcast telemetry."""
        self.log.info("Connecting to flight controller...")
        await self.drone.connect(system_address=CONNECTION_STRING)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("Flight controller connected.")
                break

        self.log.info("Starting telemetry broadcast tasks...")
        
        asyncio.create_task(self._update_position())
        asyncio.create_task(self._update_in_air_status())
        asyncio.create_task(self._update_gps_info())

        while True:
            self.send_data(self.telemetry_data)
            await asyncio.sleep(1.0 / BROADCAST_RATE_HZ)

    async def _update_position(self):
        async for position in self.drone.telemetry.position():
            self.telemetry_data.update({
                "latitude_deg": position.latitude_deg,
                "longitude_deg": position.longitude_deg,
                "relative_altitude_m": position.relative_altitude_m
            })

    async def _update_in_air_status(self):
        async for is_in_air in self.drone.telemetry.in_air():
            self.telemetry_data['is_in_air'] = is_in_air

    async def _update_gps_info(self):
        async for gps_info in self.drone.telemetry.gps_info():
            self.telemetry_data['num_satellites'] = gps_info.num_satellites

    def send_data(self, data_dict):
        if not data_dict:
            return
        try:
            message = json.dumps(data_dict).encode('utf-8')
            self.sock.sendto(message, self.target_address)
        except Exception as e:
            self.log.error(f"Error sending data: {e}")

if __name__ == "__main__":
    leader = Leader()
    asyncio.run(leader.run())