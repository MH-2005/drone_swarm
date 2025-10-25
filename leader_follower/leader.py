#!/usr/bin/env python3
# leader_wifi_complete.py
#
# Description:
# The complete and final Leader script for robust Wi-Fi communication.
# It meticulously logs every step, from connection to broadcasting. Each
# packet is enriched with a timestamp and sequence number for flawless
# diagnostics before being sent to the Follower's static IP.
#

import asyncio
import logging
import socket
import json
import time
from mavsdk import System

# --- Configuration ---
LOG_FILE = "leader.log"
# For real hardware, use your serial port. For USB connection to Pixhawk, it's often ttyACM0.
CONNECTION_STRING = "udp://:14540"

# --- Wi-Fi Network Configuration ---
# The STATIC IP address of the Follower drone's computer (Orange Pi / RPi)
FOLLOWER_IP = "192.168.1.111"  # <-- IP ثابت پهپاد دنبال‌کننده را اینجا وارد کنید
BROADCAST_PORT = 5005
BROADCAST_RATE_HZ = 20  # 20 Hz is a good rate for smooth following

class Leader:
    """The flying beacon that broadcasts its vitals over Wi-Fi with extensive logging."""
    def __init__(self):
        self.drone = System()
        self.log = logging.getLogger(__name__)
        # Configure logging to be extremely detailed
        logging.basicConfig(
            level=logging.INFO, # Change to logging.DEBUG for packet-by-packet logs
            format="%(asctime)s [%(levelname)s] [LEADER] %(message)s",
            handlers=[logging.FileHandler(LOG_FILE), logging.StreamHandler()]
        )
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = (FOLLOWER_IP, BROADCAST_PORT)
        self.telemetry_data = {}
        self.sequence_counter = 0

    async def run(self):
        """Main loop to connect, wait for data, and then broadcast."""
        self.log.info("--- Leader Drone Initializing ---")
        self.log.info(f"Attempting to connect to flight controller at {CONNECTION_STRING}...")
        await self.drone.connect(system_address=CONNECTION_STRING)
        
        self.log.info("Waiting for connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.log.info("✅ Flight controller connected successfully.")
                break

        self.log.info("Starting background tasks to gather telemetry...")
        asyncio.create_task(self._update_position())
        asyncio.create_task(self._update_in_air_status())
        asyncio.create_task(self._update_gps_info())
        
        self.log.info("Waiting for a valid GPS position fix before broadcasting...")
        while 'latitude_deg' not in self.telemetry_data:
            self.log.info("Still waiting for GPS fix...")
            await asyncio.sleep(1)
        self.log.info(f"🛰️ GPS fix acquired! Latitude: {self.telemetry_data.get('latitude_deg')}")
        
        self.log.info("--- Starting Main Broadcast Loop ---")
        self.log.info(f"📡 Broadcasting vitals to Wi-Fi address {self.target_address[0]}:{self.target_address[1]}")

        while True:
            self.send_vitals()
            await asyncio.sleep(1.0 / BROADCAST_RATE_HZ)

    async def _update_position(self):
        """Background task to update position data."""
        async for position in self.drone.telemetry.position():
            self.telemetry_data.update({
                "latitude_deg": position.latitude_deg,
                "longitude_deg": position.longitude_deg,
                "relative_altitude_m": position.relative_altitude_m
            })

    async def _update_in_air_status(self):
        """Background task to update in-air status."""
        async for is_in_air in self.drone.telemetry.in_air():
            self.telemetry_data['is_in_air'] = is_in_air

    async def _update_gps_info(self):
        """Background task to update GPS satellite count."""
        async for gps_info in self.drone.telemetry.gps_info():
            self.telemetry_data['num_satellites'] = gps_info.num_satellites

    def send_vitals(self):
        """Packages and sends the latest vitals data with timestamp and sequence."""
        if 'latitude_deg' not in self.telemetry_data:
            self.log.warning("Skipping send, latitude not available yet.")
            return
        
        # Add crucial metadata for robust communication
        self.telemetry_data['timestamp'] = time.time()
        self.telemetry_data['seq'] = self.sequence_counter

        try:
            message_bytes = json.dumps(self.telemetry_data).encode('utf-8')
            self.sock.sendto(message_bytes, self.target_address)
            self.log.debug(f"-> SENT packet #{self.sequence_counter} ({len(message_bytes)} bytes)")
            self.sequence_counter += 1
        except Exception as e:
            self.log.error(f"NETWORK ERROR: Could not send data over Wi-Fi: {e}")

if __name__ == "__main__":
    leader = Leader()
    try:
        asyncio.run(leader.run())
    except KeyboardInterrupt:
        logging.info("🛑 Leader script terminated by user.")