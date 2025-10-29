#!/usr/bin/env python3
"""
FollowerRect - اصلاح‌شده
نسخهٔ اصلاح‌شدهٔ کنترل کنندهٔ رندر برای گروهی از پهپادها با استفاده از MAVSDK
تغییرات مهم:
 - رفع مشکل الگوریتم انتخابات (Bully-like)
 - پیاده‌سازی `listen_udp` و `announce_presence` با bind
 - تصحیح نام و مفهوم UPDATE_INTERVAL_S (بازه بر حسب ثانیه)
 - اصلاح start_mission تا از ایندکس فعلی ادامه دهد (برای resume)
 - اطمینان از await کردن cancel روی mission_task برای cleanup
 - اضافه کردن محدودیت/timeout در loopهای حساس (takeoff/landing/move)
 - چک exclusion zone قبل از حرکت
 - کنترل خطا و validation پیام‌های UDP

نکته‌ها دربارهٔ Z/NED:
 - در این کد "Down" مثبت است (مطابق MAVSDK): مقدار بزرگتر = پایین‌تر
 - برای هدف‌گیری ارتفاعی از مقدار منفی برای بالا رفتن نسبت به home استفاده شده است

نکات اجرایی:
 - قبل از اجرا مقدار NODE_ID و MAV_CONN را برای هر پهپاد تنظیم کنید
 - برای تست محلی از SITL (مثل PX4 SITL) استفاده کنید یا آدرس MAV_CONN مناسب
"""

import asyncio
import json
import socket
import time
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Tuple
import logging

# ======================= پیکربندی =======================
NODE_ID = 3  # Change for each drone (1..SWARM_SIZE)
MAV_CONN = "udp://:14540"  # e.g., SITL
UDP_BROADCAST_IP = "255.255.255.255"  # Broadcast address - می‌توانید آن را به شبکه محلی محدود کنید
UDP_PORT = 5005
SWARM_SIZE = 3
UPDATE_INTERVAL_S = 0.1  # seconds between offboard velocity commands (0.1 -> 10 Hz)

# سرعت‌ها
MAX_CRUISE_VELOCITY_M_S = 0.8
TAKEOFF_VELOCITY_M_S = 0.8
LAND_VELOCITY_M_S = 0.4

# Safety / timeouts
POSITION_TIMEOUT_S = 40.0  # حداکثر زمان منتظر ماندن برای رسیدن به هدف
ELECTION_TIMEOUT_S = 10.0
HEALTH_WAIT_TIMEOUT_S = 20.0
PROXIMITY_THRESHOLD_M = 0

# Logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(f"Drone_{NODE_ID}")

# ======================= انواع و داده‌ها =======================
class MessageType(Enum):
    PRESENCE = "PRESENCE"
    TELEMETRY = "TELEMETRY"
    ELECTION = "ELECTION"
    COORDINATOR = "COORDINATOR"
    MISSION = "MISSION"
    MISSION_START = "MISSION_START"
    HOLD = "HOLD"
    RESUME = "RESUME"

class DroneState(Enum):
    INIT = "INIT"
    ARMING = "ARMING"
    ARMED = "ARMED"
    OFFBOARD = "OFFBOARD"
    MISSION_RUNNING = "MISSION_RUNNING"
    HOLDING = "HOLDING"
    LANDING = "LANDING"
    DISCONNECTED = "DISCONNECTED"

class MissionStep(Enum):
    TAKEOFF = "TAKEOFF"
    LINE = "LINE"
    ALIGN_ALTITUDE = "ALIGN_ALTITUDE"
    TRI_H = "TRI_H"
    MOVE_FORWARD = "MOVE_FORWARD"
    TRI_V = "TRI_V"
    MOVE_BACK = "MOVE_BACK"
    RETURN = "RETURN"
    LAND = "LAND"

@dataclass
class Position:
    x: float = 0.0  # North (m)
    y: float = 0.0  # East (m)
    z: float = 0.0  # Down (m) - Positive downwards

@dataclass
class DroneInfo:
    node_id: int
    position_ned: Position
    state: DroneState
    last_update: float
    is_leader: bool = False

@dataclass
class MissionCommand:
    step: MissionStep
    parameters: Dict = None

# ======================= کلاس کنترل کننده =======================
class FollowerRect:
    def __init__(self, node_id: int, mav_conn: str):
        # Validate node id
        if node_id < 1 or node_id > SWARM_SIZE:
            raise ValueError(f"NODE_ID must be between 1 and SWARM_SIZE (1..{SWARM_SIZE})")

        self.node_id = node_id
        self.mav_conn = mav_conn
        self.drone = System()

        # UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # For receive, we will bind later in listen_udp

        # Swarm bookkeeping
        self.swarm_members: Dict[int, DroneInfo] = {}
        self.swarm_ids = sorted(list(range(1, SWARM_SIZE + 1)))
        self.drone_index = self.swarm_ids.index(self.node_id)

        # State
        self.current_state = DroneState.INIT
        self.is_leader = False
        self.leader_id = None
        self.election_in_progress = False

        # Mission
        self.mission_plan: List[MissionCommand] = []
        self.current_mission_step_index = 0
        self.hold_position = False
        self.mission_task: asyncio.Task = None

        # Safety
        self.exclusion_zones: List[Tuple[Position, float]] = []
        self.safety_radius = 2.0

        # Position (NED)
        self.current_position_ned = Position(0.0, 0.0, 0.0)
        self.target_position_ned = Position(0.0, 0.0, 0.0)

        # Mission params
        self.takeoff_down = -8.0  # target down (negative -> above home)
        self.line_spacing = 4.0
        self.triangle_spacing = 4.0
        self.move_distance = 5.0

        self.initial_altitudes = {
            1: -6.0,   # پهپاد ۱ در ارتفاع ۶ متری
            2: -8.0,   # پهپاد ۲ در ارتفاع ۸ متری
            3: -10.0   # پهپاد ۳ در ارتفاع ۱۰ متری
        }

        # Internal tasks references for cleanup
        self._pos_task = None
        self._background_tasks = []

        # PID vertical controller params (in __init__)
        self.z_kp = 0.8
        self.z_ki = 0.1
        self.z_kd = 0.05
        self._z_integral = 0.0
        self._z_prev_error = 0.0
        self._z_last_time = None
        self.z_integral_limit = 5.0  # جلوگیری از wind-up


        self.pause_between_steps_s = 5.0


    # ----------------------- اتصال و آماده‌سازی -----------------------
    async def connect_drone(self):
        logger.info(f"Connecting to drone at {self.mav_conn}")
        await self.drone.connect(system_address=self.mav_conn)

        # Wait with timeout
        start_t = time.time()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("Connected to drone!")
                return
            if time.time() - start_t > 10.0:
                raise TimeoutError("Timeout connecting to drone")

    async def setup_drone(self):
        # Wait for good health
        logger.info("Waiting for drone health (global pos, home pos, armable)...")
        start_t = time.time()
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok and health.is_armable:
                logger.info("Drone health OK")
                break
            if time.time() - start_t > HEALTH_WAIT_TIMEOUT_S:
                raise TimeoutError("Timeout waiting for drone health checks")
            await asyncio.sleep(0.5)

        # Start position update task
        self._pos_task = asyncio.create_task(self.update_position_ned())
        self._background_tasks.append(self._pos_task)

        # Provide an initial offboard velocity (some firmwares require an initial set)
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        except Exception as e:
            logger.warning(f"Initial offboard set failed: {e}")

    async def update_position_ned(self):
        """Continuously update current position in NED coordinates"""
        try:
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                try:
                    pos = pos_vel.position
                    self.current_position_ned = Position(
                        x=pos.north_m,
                        y=pos.east_m,
                        z=pos.down_m
                    )
                except Exception:
                    # If structure differs, try other attribute names or ignore
                    pass
        except asyncio.CancelledError:
            logger.info("Position update task cancelled")
        except Exception as e:
            logger.error(f"update_position_ned failed: {e}")

    

    # ----------------------- ارتباط UDP -----------------------
    def send_udp_message(self, message_type: MessageType, data: Dict):
        """ارسال پیام UDP به صورت broadcast
        پیام شامل node_id، type، timestamp و data می‌باشد.
        """
        message = {
            "node_id": self.node_id,
            "type": message_type.value,
            "timestamp": time.time(),
            "data": data or {}
        }
        try:
            self.udp_socket.sendto(json.dumps(message).encode(), (UDP_BROADCAST_IP, UDP_PORT))
        except Exception as e:
            logger.error(f"Failed to send UDP message: {e}")

    async def listen_udp(self):
        """دریافت پیام‌های UDP (bind و پردازش غیرمسدود)
        این تابع از run_in_executor برای recvfrom استفاده می‌کند تا با event loop تداخل نداشته باشد.
        """
        try:
            self.udp_socket.bind(("", UDP_PORT))
            self.udp_socket.setblocking(True)
        except Exception as e:
            logger.error(f"Failed to bind UDP socket: {e}")
            return

        loop = asyncio.get_event_loop()
        logger.info(f"Listening for UDP messages on port {UDP_PORT}")
        while True:
            try:
                # recvfrom runs in threadpool to avoid blocking the event loop
                data, addr = await loop.run_in_executor(None, self.udp_socket.recvfrom, 4096)
                if not data:
                    await asyncio.sleep(0.01)
                    continue
                try:
                    message = json.loads(data.decode())
                except Exception:
                    logger.warning("Received invalid JSON over UDP")
                    continue
                # Dispatch handler
                asyncio.create_task(self.safe_handle_message(message))
            except asyncio.CancelledError:
                logger.info("listen_udp cancelled")
                break
            except Exception as e:
                # transient errors
                logger.warning(f"listen_udp error: {e}")
                await asyncio.sleep(0.1)

    async def safe_handle_message(self, message: Dict):
        try:
            await self.handle_message(message)
        except Exception as e:
            logger.error(f"Error handling message: {e}")

    async def announce_presence(self):
        """هر ثانیه حضور و وضعیت را broadcast می‌کند."""
        while True:
            payload = {
                "position": {"x": self.current_position_ned.x, "y": self.current_position_ned.y, "z": self.current_position_ned.z},
                "state": self.current_state.value if isinstance(self.current_state, DroneState) else str(self.current_state)
            }
            self.send_udp_message(MessageType.PRESENCE, payload)
            await asyncio.sleep(1.0)

    async def handle_message(self, message: Dict):
        # Basic validation
        try:
            msg_type = MessageType(message.get("type"))
        except Exception:
            logger.warning(f"Unknown message type: {message.get('type')}")
            return

        sender_id = message.get("node_id")
        if sender_id is None:
            logger.warning("UDP message missing node_id")
            return

        # Update swarm members if message from others
        if sender_id != self.node_id:
            pos_data = message.get("data", {}).get("position", {})
            if sender_id not in self.swarm_members:
                self.swarm_members[sender_id] = DroneInfo(node_id=sender_id, position_ned=Position(), state=DroneState.INIT, last_update=time.time())

            di = self.swarm_members[sender_id]
            di.last_update = time.time()
            # Update state safely
            state_str = message.get("data", {}).get("state")
            if state_str:
                try:
                    di.state = DroneState(state_str)
                except Exception:
                    pass
            # Update position safely
            try:
                di.position_ned = Position(x=float(pos_data.get("x", 0.0)), y=float(pos_data.get("y", 0.0)), z=float(pos_data.get("z", 0.0)))
            except Exception:
                pass

        # Handle types
        if msg_type == MessageType.PRESENCE:
            # If no leader known, start election (but don't spam)
            if not self.leader_id and not self.election_in_progress:
                await self.start_election()
        elif msg_type == MessageType.ELECTION:
            await self.handle_election(message)
        elif msg_type == MessageType.COORDINATOR:
            await self.handle_coordinator(message)
        elif msg_type == MessageType.MISSION:
            await self.handle_mission(message)
        elif msg_type == MessageType.MISSION_START:
            await self.handle_mission_start(message)
        elif msg_type == MessageType.HOLD:
            await self.handle_hold(message)
        elif msg_type == MessageType.RESUME:
            await self.handle_resume(message)

    # ----------------------- انتخابات -----------------------
    async def start_election(self):
        """شروع انتخابات (Bully-like): پیام ELECTION حاوی candidate_id ارسال می‌شود.
        اگر هیچ پاسخ از نودهای با id بزرگ‌تر نیاید، کاندیدا رهبر می‌شود.
        """
        if self.election_in_progress:
            return
        self.election_in_progress = True
        logger.info("Starting leader election")
        self.send_udp_message(MessageType.ELECTION, {"candidate_id": self.node_id})

        # Wait برای پاسخ از nodes بالاتر
        await asyncio.sleep(ELECTION_TIMEOUT_S)

        # اگر کسی بالاتر به ما پاسخ نداده باشد و هنوز election_in_progress است => رهبر شو
        if self.election_in_progress:
            await self.become_leader()

    async def handle_election(self, message: Dict):
        data = message.get("data", {})
        candidate = data.get("candidate_id")
        responder = data.get("responder")

        # اگر پیام ELECTION شامل responder بود، یعنی یک نود با id پایین‌تر دارد به ما می‌گوید بالاتر وجود دارد
        # اما این شکل ساده‌سازی شده است: اگر من بالاتر از candidate هستم، به او پاسخ می‌دهم.
        if candidate is None:
            # ممکن است این پیام پاسخ به candidate باشد
            return

        if self.node_id > candidate:
            # اعلام به کاندیدا که گرهٔ بالاتری وجود دارد
            self.send_udp_message(MessageType.ELECTION, {"response_to": candidate, "responder": self.node_id})
            # و خودم انتخابات را شروع می‌کنم تا خودم رهبر شوم
            # (تا زمانی که گره‌ای با id بزرگ‌تر نباشد)
            # اما مراقب باشیم recursion ایجاد نشود: اگر election_in_progress باشد دیگر start نکن
            if not self.election_in_progress:
                await self.start_election()
        else:
            # candidate >= self.node_id => قبول می‌کنیم یا منتظر می‌مانیم که coordinator اعلام شود
            pass

    async def become_leader(self):
        self.is_leader = True
        self.leader_id = self.node_id
        self.election_in_progress = False
        logger.info("I am now the leader!")
        self.send_udp_message(MessageType.COORDINATOR, {})
        await self.create_mission_plan()

    async def handle_coordinator(self, message: Dict):
        self.leader_id = message.get("node_id")
        self.is_leader = (self.leader_id == self.node_id)
        self.election_in_progress = False
        if not self.is_leader:
            logger.info(f"Leader is node {self.leader_id}")

    # ----------------------- برنامه‌ریزی ماموریت -----------------------
    async def create_mission_plan(self):
        if not self.is_leader:
            return
        self.mission_plan = [
            MissionCommand(MissionStep.TAKEOFF),      # ۱. هرکس به ارتفاع اولیه خودش بلند می‌شود
            MissionCommand(MissionStep.LINE),         # ۲. در همان ارتفاع، در یک خط قرار می‌گیرند
            MissionCommand(MissionStep.ALIGN_ALTITUDE), # ۳. حالا همه به ارتفاع نهایی و یکسان می‌روند
            MissionCommand(MissionStep.TRI_H),        # ۴. اکنون که هم‌ارتفاع هستند، فرمیشن مثلث را تشکیل می‌دهند
            MissionCommand(MissionStep.MOVE_FORWARD, {"distance": self.move_distance}),
            MissionCommand(MissionStep.TRI_V),
            MissionCommand(MissionStep.MOVE_BACK, {"distance": -self.move_distance}),
            MissionCommand(MissionStep.RETURN),
            MissionCommand(MissionStep.LAND)
        ]
        mission_data = {"mission_steps": [{"step": cmd.step.value, "parameters": cmd.parameters} for cmd in self.mission_plan]}
        self.send_udp_message(MessageType.MISSION, mission_data)
        logger.info("Mission plan created and broadcasted")

    async def handle_mission(self, message: Dict):
        # دریافت برنامهٔ مأموریت
        data = message.get("data", {})
        steps = data.get("mission_steps", [])
        parsed = []
        for s in steps:
            try:
                step_enum = MissionStep(s.get("step"))
                parsed.append(MissionCommand(step_enum, s.get("parameters")))
            except Exception:
                logger.warning(f"Unknown mission step received: {s}")
        if parsed:
            self.mission_plan = parsed
            logger.info("Mission plan updated from leader message")

    async def handle_mission_start(self, message: Dict):
        # رهبر پیغام شروع را فرستاده است
        await self.start_mission()

    # ----------------------- اجرای ماموریت -----------------------
    async def start_mission(self):
        # Cancel existing mission task (با await برای cleanup)
        if self.mission_task and not self.mission_task.done():
            self.mission_task.cancel()
            try:
                await self.mission_task
            except asyncio.CancelledError:
                pass

        self.current_state = DroneState.MISSION_RUNNING
        logger.info("Starting mission execution")

        async def mission_sequence():
            try:
                for step_index in range(self.current_mission_step_index, len(self.mission_plan)):
                    if self.hold_position:
                        await self.wait_for_resume()
                    self.current_mission_step_index = step_index
                    mission_cmd = self.mission_plan[step_index]
                    await self.execute_mission_step(mission_cmd)

                    if step_index < len(self.mission_plan) - 1:
                        logger.info(f"Pausing for {self.pause_between_steps_s} seconds before next step...")
                        await asyncio.sleep(self.pause_between_steps_s)

            except asyncio.CancelledError:
                logger.info("Mission execution cancelled")
            except Exception as e:
                logger.error(f"Mission execution failed: {e}")
                self.hold_position = True
                await self.hold_current_position()

        self.mission_task = asyncio.create_task(mission_sequence())


    async def execute_mission_step(self, mission_cmd: MissionCommand):
        logger.info(f"Executing mission step: {mission_cmd.step.value}")
        
        initial_altitude = self.initial_altitudes.get(self.node_id, self.takeoff_down)

        if mission_cmd.step == MissionStep.TAKEOFF:
            original_takeoff_alt = self.takeoff_down
            self.takeoff_down = initial_altitude
            await self.takeoff()
            self.takeoff_down = original_takeoff_alt 
            
        elif mission_cmd.step == MissionStep.LINE:
            target_pos = self.calculate_line_position_ned()
            target_pos.z = initial_altitude
            await self.move_to_formation(target_pos)

        elif mission_cmd.step == MissionStep.ALIGN_ALTITUDE:
            logger.info(f"Aligning altitude to {-self.takeoff_down} m")
            target_pos = Position(
                x=self.current_position_ned.x,
                y=self.current_position_ned.y,
                z=self.takeoff_down 
            )
            await self.move_to_formation(target_pos)

        elif mission_cmd.step == MissionStep.TRI_H:
            positions = self.calculate_triangle_positions_horizontal()
            target_pos = positions.get(self.node_id, self.calculate_line_position_ned())
            target_pos.z = self.takeoff_down
            await self.move_to_formation(target_pos)

        elif mission_cmd.step in (MissionStep.MOVE_FORWARD, MissionStep.MOVE_BACK):
            distance = mission_cmd.parameters.get("distance", 0.0) if mission_cmd.parameters else 0.0
            await self.move_formation(distance_north=distance, distance_east=0.0)
        
        elif mission_cmd.step == MissionStep.TRI_V:
            positions = self.calculate_triangle_positions_vertical()
            target_pos = positions.get(self.node_id, self.calculate_line_position_ned())
            await self.move_to_formation(target_pos)
        
        elif mission_cmd.step == MissionStep.RETURN:
            return_pos = self.calculate_line_position_ned()
            return_pos.z = self.takeoff_down
            await self.move_to_formation(return_pos)
        
        elif mission_cmd.step == MissionStep.LAND:
            await self.land()

    async def wait_for_resume(self):
        self.current_state = DroneState.HOLDING
        logger.info("Mission held, maintaining position...")
        await self.hold_current_position()
        while self.hold_position:
            await asyncio.sleep(0.1)
        self.current_state = DroneState.MISSION_RUNNING
        logger.info("Mission resumed")

    # ----------------------- حرکت و کنترل -----------------------
    async def takeoff(self):
        logger.info(f"Taking off to {-self.takeoff_down} m (Down = {self.takeoff_down})")
        self.target_position_ned.z = self.takeoff_down

        # هدف: کاهش مقدار z (زیرا Down مثبت است) تا به مقدار target برسیم
        target = self.target_position_ned.z
        start_t = time.time()

        while True:
            # safety: timeout
            if time.time() - start_t > POSITION_TIMEOUT_S:
                logger.warning("Takeoff timeout reached")
                break

            if self.hold_position:
                logger.info("Takeoff interrupted by HOLD")
                return

            error = target - self.current_position_ned.z
            if abs(error) < 0.5:
                logger.info("Reached takeoff altitude (within tolerance)")
                break

            # Ramp-based vertical speed (negative value to go up since down positive)
            ramp_dist = 2.0
            ramp = min(1.0, abs(error) / ramp_dist)
            speed = -TAKEOFF_VELOCITY_M_S * ramp  # negative -> move up

            # ensure minimum effective speed unless very close
            if abs(speed) < 0.2:
                speed = math.copysign(0.2, speed)

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, speed, 0.0))
            await asyncio.sleep(UPDATE_INTERVAL_S)

        # hold at altitude after takeoff
        await self.hold_current_position()

    async def land(self):
        self.current_state = DroneState.LANDING
        logger.info("Landing...")

        start_t = time.time()
        while True:
            # check disarmed -> landed
            armed_gen = self.drone.telemetry.armed()
            try:
                armed = await armed_gen.__anext__()
                if not armed:
                    logger.info("Disarmed -> assumed landed")
                    return
            except Exception:
                # ignore telemetry read failures
                pass

            # send gentle positive down velocity
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, LAND_VELOCITY_M_S, 0.0))
            await asyncio.sleep(UPDATE_INTERVAL_S)

            if time.time() - start_t > POSITION_TIMEOUT_S * 2:
                logger.warning("Landing taking too long; attempting action.land()")
                try:
                    await self.drone.action.land()
                except Exception as e:
                    logger.error(f"action.land failed: {e}")
                return

    async def move_to_formation(self, target_pos_ned: Position):
        logger.info(f"Moving to formation target NED: x={target_pos_ned.x:.2f}, y={target_pos_ned.y:.2f}, z={target_pos_ned.z:.2f}")

        # safety: check exclusion zone
        if self.is_in_exclusion(target_pos_ned):
            logger.warning("Target inside exclusion zone -> aborting move")
            return

        position_tolerance = 0.3
        start_t = time.time()
        while True:
            if self.hold_position:
                logger.info("Movement interrupted by HOLD")
                return

            # timeout
            if time.time() - start_t > POSITION_TIMEOUT_S:
                logger.warning("move_to_formation timeout")
                break

            delta_n = target_pos_ned.x - self.current_position_ned.x
            delta_e = target_pos_ned.y - self.current_position_ned.y
            delta_d = target_pos_ned.z - self.current_position_ned.z
            error_mag = math.sqrt(delta_n**2 + delta_e**2 + delta_d**2)

            if error_mag < position_tolerance:
                logger.info("Reached formation position (within tolerance)")
                break

            # P controller
            k_p = 0.5
            vel_n = delta_n * k_p
            vel_e = delta_e * k_p
            vel_d = delta_d * k_p

            # limit horizontal
            horiz = math.sqrt(vel_n**2 + vel_e**2)
            if horiz > MAX_CRUISE_VELOCITY_M_S:
                scale = MAX_CRUISE_VELOCITY_M_S / horiz
                vel_n *= scale
                vel_e *= scale

            # limit vertical
            if abs(vel_d) > TAKEOFF_VELOCITY_M_S:
                vel_d = math.copysign(TAKEOFF_VELOCITY_M_S, vel_d)

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_d, 0.0))
            await asyncio.sleep(UPDATE_INTERVAL_S)

        await self.hold_current_position()

    async def move_formation(self, distance_north: float, distance_east: float):
        target_n = self.current_position_ned.x + distance_north
        target_e = self.current_position_ned.y + distance_east
        target = Position(target_n, target_e, self.current_position_ned.z)
        await self.move_to_formation(target)

    async def hold_current_position(self):
        """ارسال مکرر سرعت صفر تا زمان آزادسازی HOLD"""
        self.target_position_ned = Position(self.current_position_ned.x, self.current_position_ned.y, self.current_position_ned.z)
        self.current_state = DroneState.HOLDING
        logger.info("Holding current position")
        while self.hold_position:
            try:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            except Exception as e:
                logger.warning(f"Failed to send hold velocity: {e}")
            await asyncio.sleep(UPDATE_INTERVAL_S)
        self.current_state = DroneState.OFFBOARD


    async def monitor_proximity(self):
        while True:
            if self.hold_position:
                await asyncio.sleep(1.0)
                continue

            my_pos = self.current_position_ned
            for drone_id, other_drone in self.swarm_members.items():
                other_pos = other_drone.position_ned
                distance = math.sqrt(
                    (my_pos.x - other_pos.x)**2 +
                    (my_pos.y - other_pos.y)**2 +
                    (my_pos.z - other_pos.z)**2
                )

                if distance < PROXIMITY_THRESHOLD_M:
                    logger.critical(
                        f"!!! خطر برخورد !!! فاصله با پهپاد {drone_id} بسیار کم است ({distance:.2f} متر). ورود به حالت HOLD."
                    )
                    
                    self.hold_position = True

                    if self.mission_task and not self.mission_task.done():
                        self.mission_task.cancel()
                        try:
                            await self.mission_task
                        except asyncio.CancelledError:
                            pass 

                    asyncio.create_task(self.hold_current_position())

                    break
            
            await asyncio.sleep(0.2)

    # ----------------------- محاسبات فرمِیشن -----------------------
    def calculate_line_position_ned(self) -> Position:
        center_offset = (SWARM_SIZE - 1) / 2.0
        y_pos = (self.drone_index - center_offset) * self.line_spacing
        return Position(x=0.0, y=y_pos, z=self.takeoff_down)

    def calculate_triangle_positions_horizontal(self) -> Dict[int, Position]:
        positions = {}
        s = self.triangle_spacing
        z = self.takeoff_down
        if SWARM_SIZE >= 3:
            positions[self.swarm_ids[0]] = Position(x=s * math.sqrt(3)/2, y=0.0, z=z)
            positions[self.swarm_ids[1]] = Position(x=0.0, y=-s/2, z=z)
            positions[self.swarm_ids[2]] = Position(x=0.0, y=s/2, z=z)
        else:
            positions[self.node_id] = self.calculate_line_position_ned()
        return positions

    def calculate_triangle_positions_vertical(self) -> Dict[int, Position]:
        positions = {}
        s = self.triangle_spacing
        if SWARM_SIZE >= 3:
            positions[self.swarm_ids[0]] = Position(x=0.0, y=0.0, z=self.takeoff_down + s * math.sqrt(3)/2)
            positions[self.swarm_ids[1]] = Position(x=0.0, y=-s/2, z=self.takeoff_down)
            positions[self.swarm_ids[2]] = Position(x=0.0, y=s/2, z=self.takeoff_down)
        else:
            positions[self.node_id] = self.calculate_line_position_ned()
        return positions

    # ----------------------- کمکی‌ها و ایمنی -----------------------
    def is_in_exclusion(self, pos: Position) -> bool:
        for center, r in self.exclusion_zones:
            dx = pos.x - center.x
            dy = pos.y - center.y
            dz = pos.z - center.z
            if math.sqrt(dx*dx + dy*dy + dz*dz) < r:
                return True
        return False

    async def arm_and_wait_offboard(self):
       """
        Arm the drone and then WAIT for the pilot to switch flight mode to OFFBOARD.
        This function WILL NOT call offboard.start().
       """
       logger.info("Arming drone (waiting for pilot to enable OFFBOARD)...")
       try:
            await self.drone.action.arm()
       except Exception as e:
            logger.error(f"Failed to arm: {e}")
            return

       self.current_state = DroneState.ARMED
       logger.info("Armed. Now waiting for flight mode to become OFFBOARD (pilot must switch mode).")

       overall_timeout = 60.0
       start_t = time.time()

       while True:
            if overall_timeout is not None and (time.time() - start_t) > overall_timeout:
                logger.warning("Timeout while waiting for OFFBOARD mode. Still ARMED and waiting.")
                start_t = time.time()
                await asyncio.sleep(1.0)

            try: 
                async for fm in self.drone.telemetry.flight_mode():
                # FIX: FlightMode is an object, not a raw string. Convert safely.
                    try:
                        current_fm = getattr(fm, "name", None)
                        if current_fm is None:
                            current_fm = str(fm)
                        current_fm = current_fm.upper()
                    except Exception:
                        current_fm = str(fm).upper()

                    if current_fm == "OFFBOARD":
                        logger.info("Flight mode is OFFBOARD (pilot switched). Proceeding.")
                        self.current_state = DroneState.OFFBOARD

                        if self.is_leader:
                            start_wait = time.time()
                            while not await self.all_drones_ready():
                                if time.time() - start_wait > 20.0:
                                    logger.warning("Timeout waiting for other drones to be OFFBOARD; proceeding anyway.")
                                    break
                                await asyncio.sleep(1.0)
                            self.send_udp_message(MessageType.MISSION_START, {})
                            await self.start_mission()
                        return
                    else:
                    # not OFFBOARD yet -> short sleep then re-check
                        await asyncio.sleep(0.5)
                        break
            except asyncio.CancelledError:
                logger.info("arm_and_wait_offboard cancelled")
                return
            except Exception as e:
                logger.warning(f"Error while reading flight mode telemetry: {e}")
                await asyncio.sleep(0.5)


    async def all_drones_ready(self) -> bool:
        # need SWARM_SIZE-1 others
        if len(self.swarm_members) < SWARM_SIZE - 1:
            return False
        for drone_info in self.swarm_members.values():
            if drone_info.state != DroneState.OFFBOARD:
                return False
        return True

    # async def monitor_swarm_health(self):
    #     while True:
    #         now = time.time()
    #         disconnected = []
    #         for drone_id, di in list(self.swarm_members.items()):
    #             if now - di.last_update > 5.0:
    #                 disconnected.append(drone_id)
    #                 self.exclusion_zones.append((di.position_ned, self.safety_radius))
    #         for d in disconnected:
    #             del self.swarm_members[d]
    #             logger.warning(f"Drone {d} disconnected -> exclusion zone created")

    #         # leader lost?
    #         if self.leader_id and self.leader_id != self.node_id and self.leader_id not in self.swarm_members and not self.election_in_progress:
    #             logger.warning("Leader disconnected, starting new election")
    #             self.leader_id = None
    #             await self.start_election()

    #         await asyncio.sleep(1)
    async def monitor_swarm_health(self):
     while True:
        now = time.time()
        disconnected = []

        for drone_id, di in list(self.swarm_members.items()):
            if now - di.last_update > 5.0:
                disconnected.append(drone_id)
                self.exclusion_zones.append((di.position_ned, self.safety_radius))

        for d in disconnected:
            del self.swarm_members[d]
            logger.warning(f"Drone {d} disconnected -> exclusion zone created")

        # اگر leader ناپدید شد، شروع انتخابات
        if self.leader_id and self.leader_id != self.node_id and self.leader_id not in self.swarm_members and not self.election_in_progress:
            logger.warning("Leader disconnected, starting new election")
            self.leader_id = None
            await self.start_election()

        # اگر قطع اتصال‌های متوالی زیاد شد: به جای RTL/LAND فقط HOLD می‌کنیم
        # (پارامترها را بر حسب نیاز خود تنظیم کن)
        MAX_DISCONNECT_FOR_HOLD = 2
        if len(disconnected) >= MAX_DISCONNECT_FOR_HOLD:
            logger.critical("Multiple drones disconnected -> entering HOLD mode (no RTL/land will be executed).")
            # فعال‌سازی حالت HOLD و توقف ماموریت
            self.hold_position = True
            if self.mission_task and not self.mission_task.done():
                self.mission_task.cancel()
                try:
                    await self.mission_task
                except asyncio.CancelledError:
                    pass
            # اجرا کردن نگهداری موقعیت به صورت background (تا حلقه ادامه پیدا کند)
            # اگر می‌خواهی synchronous باشه از await استفاده کن؛ اینجا background است.
            asyncio.create_task(self.hold_current_position())

        await asyncio.sleep(1)


    # async def monitor_flight_mode(self):
    #     while True:
    #         try:
    #             async for flight_mode in self.drone.telemetry.flight_mode():
    #                 if self.current_state in [DroneState.OFFBOARD, DroneState.MISSION_RUNNING, DroneState.HOLDING]:
    #                     if (flight_mode or "").upper() != "OFFBOARD":
    #                         if not self.hold_position:
    #                             logger.warning(f"Offboard lost! switched to {flight_mode} -> activating HOLD")
    #                             self.hold_position = True
    #                             if self.mission_task and not self.mission_task.done():
    #                                 self.mission_task.cancel()
    #                                 try:
    #                                     await self.mission_task
    #                                 except asyncio.CancelledError:
    #                                     pass
    #                             # run hold as background task so monitor can continue
    #                             asyncio.create_task(self.hold_current_position())
    #                 break
    #         except Exception as e:
    #             logger.error(f"Error in flight mode monitor: {e}")
    #         await asyncio.sleep(0.5)
    async def monitor_flight_mode(self):
     while True:
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                if self.current_state in [DroneState.OFFBOARD, DroneState.MISSION_RUNNING, DroneState.HOLDING]:
                    mode_str = None
                    try:
                        mode_str = getattr(flight_mode, "name", None) or str(flight_mode)
                    except Exception:
                        mode_str = str(flight_mode)
                    if (mode_str or "").upper() != "OFFBOARD":
                        if not self.hold_position:
                            logger.warning(f"Offboard mode lost! switched to {mode_str}. Activating internal HOLD (no RTL/land).")
                            self.hold_position = True
                            # cancel mission safely
                            if self.mission_task and not self.mission_task.done():
                                self.mission_task.cancel()
                                try:
                                    await self.mission_task
                                except asyncio.CancelledError:
                                    pass
                            # hold as background task
                            asyncio.create_task(self.hold_current_position())
                break
        except Exception as e:
            logger.error(f"Error in flight mode monitor: {e}")
        await asyncio.sleep(0.5)


    # ----------------------- دستورات HOLD/RESUME -----------------------
    async def handle_hold(self, message: Dict):
        if self.hold_position:
            return
        self.hold_position = True
        if self.mission_task and not self.mission_task.done():
            self.mission_task.cancel()
            try:
                await self.mission_task
            except asyncio.CancelledError:
                pass

    async def handle_resume(self, message: Dict):
        if not self.hold_position:
            return
        self.hold_position = False
        # resume mission from current index
        if self.current_state == DroneState.HOLDING:
            await self.start_mission()

    # ----------------------- حلقهٔ اصلی -----------------------
    async def run(self):
        try:
            await self.connect_drone()
            await self.setup_drone()

            # background tasks
            self._background_tasks.append(asyncio.create_task(self.listen_udp()))
            self._background_tasks.append(asyncio.create_task(self.announce_presence()))
            self._background_tasks.append(asyncio.create_task(self.monitor_swarm_health()))
            self._background_tasks.append(asyncio.create_task(self.monitor_flight_mode()))
            self._background_tasks.append(asyncio.create_task(self.monitor_proximity()))

            # give time for discovery
            await asyncio.sleep(3)

            if not self.leader_id:
                await self.start_election()

            await self.arm_and_wait_offboard()

            # main keep-alive
            while True:
                await asyncio.sleep(1)

        except asyncio.CancelledError:
            logger.info("run cancelled")
        # except Exception as e:
        #     logger.error(f"Fatal error in main run loop: {e}")
        #     try:
        #         await self.drone.action.land()
        #     except Exception as le:
        #         logger.critical(f"Emergency Land failed: {le}")
        # finally:
        #     # cleanup background tasks
        #     for t in self._background_tasks:
        #         if t and not t.done():
        #             t.cancel()
        #             try:
        #                 await t
        #             except Exception:
        #                 pass
        #     if self._pos_task and not self._pos_task.done():
        #         self._pos_task.cancel()
        #         try:
        #             await self._pos_task
        #         except Exception:
        #             pass
        except Exception as e:
            logger.error(f"Fatal error in main run loop: {e}")
            # به جای تلاش برای فرود یا RTL، فقط وارد حالت HOLD می‌شویم و سعی می‌کنیم موقعیت را حفظ کنیم.
            try:
                logger.critical("Entering HOLD due to fatal error (no automatic landing).")
                self.hold_position = True
                if self.mission_task and not self.mission_task.done():
                    self.mission_task.cancel()
                    try:
                        await self.mission_task
                    except asyncio.CancelledError:
                        pass
                # نگهداری موقعیت تا کاربر/خلبان تصمیم بگیرد
                await self.hold_current_position()
            except Exception as le:
                logger.critical(f"HOLD fallback failed: {le}")



# ======================= اجرا =======================
async def main():
    follower = FollowerRect(NODE_ID, MAV_CONN)
    await follower.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting program.")
    except RuntimeError as e:
        # fallback for environments where asyncio.run() cannot be used
        if "cannot run" in str(e):
            loop = asyncio.get_event_loop()
            loop.run_until_complete(main())
        else:
            raise
