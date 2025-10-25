#!/usr/bin/env python3

#
# leader_flight_command.py
#
# Description:
# A simple remote control for initiating flight tests. It sends "TAKEOFF"
# and "LAND" commands via UDP to the Follower drone.
#

import socket
import json
import logging

# --- Configuration ---
FOLLOWER_IP = "192.168.1.2"  # <-- IP دقیق Orange Pi دنبال‌کننده را اینجا وارد کنید
COMMAND_PORT = 5005

# لاگ‌گیری واضح در ترمینال
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LEADER] %(message)s",
)

def run_leader():
    """منتظر ورودی کاربر می‌ماند و دستورات پروازی را ارسال می‌کند."""
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    logging.info(f"🚀 Leader ready to send flight commands to {FOLLOWER_IP}:{COMMAND_PORT}")
    logging.info("Press Enter to send TAKEOFF, or type 'land' and press Enter to send LAND.")

    while True:
        try:
            user_input = input("Flight Command > ")

            command = "LAND" if user_input.lower() == 'land' else "TAKEOFF"

            message_dict = {"command": command}
            message_bytes = json.dumps(message_dict).encode('utf-8')

            sock.sendto(message_bytes, (FOLLOWER_IP, COMMAND_PORT))
            logging.info(f"📤 Sent command: {command}")

        except (KeyboardInterrupt, EOFError):
            logging.info("\n🛑 Leader stopped by user.")
            break
        except Exception as e:
            logging.error(f"An error occurred: {e}")
            break
            
    sock.close()

if __name__ == "__main__":
    run_leader()