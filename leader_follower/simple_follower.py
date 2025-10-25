#!/usr/bin/env python3

#
# follower_flight_receiver.py
#
# Description:
# An autonomous agent for basic flight tests. It listens for commands
# from the Leader and executes a full takeoff-and-land sequence. It provides
# detailed feedback and prioritizes safety.
#

import asyncio
import logging
import socket
import json
from mavsdk import System
from mavsdk.action import ActionError

# --- Configuration ---
CONNECTION_STRING = "serial:///dev/ttyACM0:921600" # اتصال به Pixhawk محلی
LISTENING_PORT = 5005
TAKEOFF_ALTITUDE = 2.5 # ارتفاع تیک‌آف به متر

# لاگ‌گیری واضح در ترمینال
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [FOLLOWER] %(message)s",
)

class Follower:
    def __init__(self):
        self.drone = System()
        self.is_flying = False

    async def run(self):
        """به پهپاد متصل شده و شنونده دستورات را راه‌اندازی می‌کند."""
        logging.info("Connecting to flight controller...")
        await self.drone.connect(system_address=CONNECTION_STRING)
        
        logging.info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logging.info("✅ --> Drone connected!")
                break
        
        asyncio.create_task(self._command_listener())
        logging.info(f"👂 Listening for commands on port {LISTENING_PORT}...")
        
        while True:
            await asyncio.sleep(1)

    async def execute_takeoff_sequence(self):
        """سکانس کامل تیک‌آف را اجرا می‌کند."""
        if self.is_flying:
            logging.warning("Already in the air. Ignoring TAKEOFF command.")
            return

        logging.info("--- Starting Takeoff Sequence ---")
        
        # ۱. Arm کردن
        logging.info("1. Arming...")
        try:
            await self.drone.action.arm()
            logging.info("   --> ARM successful.")
        except ActionError as e:
            logging.error(f"   ❌ ARM FAILED: {e}")
            return # ادامه نده اگر Arm نشد

        # ۲. تنظیم ارتفاع تیک‌آف
        logging.info(f"2. Setting takeoff altitude to {TAKEOFF_ALTITUDE}m...")
        await self.drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)

        # ۳. تیک‌آف
        logging.info("3. Taking off...")
        try:
            await self.drone.action.takeoff()
            logging.info("   --> TAKEOFF successful. Now hovering.")
            self.is_flying = True
        except ActionError as e:
            logging.error(f"   ❌ TAKEOFF FAILED: {e}")
            # اگر تیک‌آف ناموفق بود، برای ایمنی Disarm کن
            await self.drone.action.disarm()

    async def execute_land_sequence(self):
        """سکانس کامل فرود را اجرا می‌کند."""
        if not self.is_flying:
            logging.warning("Already on the ground. Ignoring LAND command.")
            return

        logging.info("--- Starting Landing Sequence ---")
        
        # ۱. فرود
        logging.info("1. Landing...")
        try:
            await self.drone.action.land()
            logging.info("   --> LAND successful. On the ground.")
            self.is_flying = False
        except ActionError as e:
            logging.error(f"   ❌ LAND FAILED: {e}")
            return # اگر فرود ناموفق بود، ادامه نده

        # ۲. Disarm کردن
        logging.info("2. Disarming...")
        try:
            await self.drone.action.disarm()
            logging.info("   --> DISARM successful. Motors are off.")
        except ActionError as e:
            logging.error(f"   ❌ DISARM FAILED: {e}")

    async def _command_listener(self):
        """به بسته‌های UDP گوش می‌دهد و دستورات را پردازش می‌کند."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setblocking(False)
            sock.bind(('0.0.0.0', LISTENING_PORT))
            
            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    message_str = data.decode('utf-8')
                    logging.info(f"📨 Received message: '{message_str}' from {addr[0]}")
                    
                    command_data = json.loads(message_str)
                    command = command_data.get("command")

                    if command == "TAKEOFF":
                        await self.execute_takeoff_sequence()
                    
                    elif command == "LAND":
                        await self.execute_land_sequence()

                except BlockingIOError:
                    pass
                except Exception as e:
                    logging.error(f"Error in listener: {e}")
                
                await asyncio.sleep(0.1)

if __name__ == "__main__":
    follower = Follower()
    asyncio.run(follower.run())