import math
import sys
import os
import threading
import time
import asyncio
from queue import Queue
import numpy as np
import yaml
from pid_controller import PID
import csv
from controller import Robot, Supervisor

# ==================== CONFIGURATION ====================

ROBOT_DEF_NAME = "POLULU_ROBOT"        # DEF name in the .wbt world file
LEFT_MOTOR_NAME = "LeftMotor"
RIGHT_MOTOR_NAME = "RightMotor"
GPS_NAME = "gps"
IMU_NAME = "imu"
DISPLAY_NAME = "display"
FILENAME = "experiments/path1.yaml"
LOGFILE = "sim_data/SIM_log1_ground_with_maxvel0_3.csv"

# Navigation control parameters
HEADING_THRESHOLD = 5.0  # degrees - target reached if within this angle
TRANSLATIONAL_THRESHOLD = 0.05 # meters - waypoint reached if within this
TURN_SPEED = 0.35  # Motor speed for turning (0.0 to 1.0)
MIN_TURN_SPEED = 0.25  # Minimum speed to ensure movement
CONTROL_RATE = 50  # Hz - control loop update rate
MOTION_CAPTURE_RATE = 40

MAX_VEL = 0.3
MAX_ROT = 0.5
CRUISING_SPEED = 0.3
DIST_KP = 0.9
DIST_KI = 0.0 
DIST_KD = 0.01
HEAD_KP = 0.045
HEAD_KI = 0.0
HEAD_KD = 0.0085

#Display parameter for smaller screens
DISPLAY_SCALE = 0.6

# Navigation states
STATE_IDLE = 0
STATE_DRIVING = 1
STATE_ARRIVED = 2
STATE_FINISHED = 3
BOX_SIZE  = 30

MIN_LIN_SPEED = 0.15
MIN_ROT_SPEED = 0.02
 #720 rpm is the polulu max
 #12 rps
 #75.4 rad/s

MAX_WHEEL_SPEED = 75.4
BLE_LATENCY_STEPS = 2
# =======================================================
class NavigationController:
    def __init__(self, 
                 shared_state, 
                 nav_shared_state, 
                 command_queue,
                 max_vel=0.6,
                 max_rot=0.6,
                 v_cruise=0.6,
                 dist_kp=2.8, dist_ki=0.0, dist_kd=0.1,
                 head_kp=2.0, head_ki=0.0, head_kd=0.1
                 ):
        """
        Initialize navigation controller.
        
        Args:
            shared_state: Shared robot state
            command_queue: Queue for sending motor commands
        """
        self.shared_state = shared_state
        self.nav_shared_state = nav_shared_state
        self.command_queue = command_queue
        self.state = STATE_IDLE
        self.max_vel = max_vel   # max motor power (e.g. 0.6)
        self.max_rot = max_rot
        self.v_cruise = v_cruise     # forward power for intermediate waypoints

        self.dist_pid = PID(dist_kp, dist_ki, dist_kd, integral_limit=2.0)
        self.head_pid = PID(head_kp, head_ki, head_kd, integral_limit=2.0)
        self.is_final = False

    def reset(self):
        self.dist_pid.reset()
        self.head_pid.reset()
    
    def set_is_final(self, is_final: bool):
        self.is_final = is_final
    
    def update(self, dt=0.02, is_stop=True):
        """
        Update control loop (call at CONTROL_RATE Hz).
        """
        #with self.nav_shared_state['lock']:
        self.state = self.nav_shared_state['state']
        if self.state == STATE_IDLE:
            return
        
        # Get current robot state (thread-safe)
        #with self.shared_state['lock']:
        rel_x = self.shared_state['x']
        rel_y = self.shared_state['y']
        rel_yaw = self.shared_state['yaw']

        # navigation to final waypoint
        self.waypoint_navigate(rel_x, rel_y, rel_yaw, dt=dt, is_stop=is_stop)

    def waypoint_navigate(self, dx, dy, d_dir_deg, dt, is_stop=True):
        """
        Docstring for waypoint_navigate
        
        :param dx: x displacement to target in meters
        :param dy: y displacement to target in meters
        :param d_dir_deg: yaw displacement to target in degrees
        :param is_stop: whether or not this is a stopping waypoint.
                        If True, stops at waypoint. 
                        If False, follows through (keeping speed)
        """
        dist = math.hypot(dx, dy)
        d_dir = math.radians(d_dir_deg)
        if self.is_final and dist < TRANSLATIONAL_THRESHOLD:
            self.stop_motors()
            self.state = STATE_FINISHED
            #with self.nav_shared_state['lock']:
            self.nav_shared_state['state'] = STATE_FINISHED
            return
        if dist < TRANSLATIONAL_THRESHOLD:
            if is_stop:
                self.stop_motors()
            self.state = STATE_ARRIVED
            #with self.nav_shared_state['lock']:
            self.nav_shared_state['state'] = STATE_ARRIVED
            return
        # tune heading pid
        #at_final = is_final and dist < TRANSLATIONAL_THRESHOLD  # meters threshold (tune)
        w_cmd = self.head_pid.step(d_dir, dt)
        w_cmd_max = 1.0
        w_cmd = max(-w_cmd_max, min(w_cmd, w_cmd_max))

        if w_cmd > 0:
            w_cmd = min(w_cmd, self.max_rot)
        else:
            w_cmd = max(w_cmd, -self.max_rot)

        # tune linear pid
        # Final waypoint: distance PID, slows down as dist → 0
        e_dist = dist
        
        v_cmd = self.dist_pid.step(e_dist, dt)
        # Never go backwards for final approach (optional):
        v_cmd = max(0.0, v_cmd)

        # Limit linear command
        v_cmd_max = 1.0
        v_cmd = max(-v_cmd_max, min(v_cmd, v_cmd_max))
        angle_mag = abs(d_dir)
        if angle_mag < math.radians(15):
            v_cmd = min(self.v_cruise, v_cmd)
        v_cmd = min(v_cmd, self.max_vel)

        #multiwaypoint
        if not is_stop and abs(d_dir_deg) < HEADING_THRESHOLD*2:
            v_cmd = max(v_cmd, self.v_cruise)
        
        #send motor commands
        if abs(v_cmd) < MIN_LIN_SPEED and abs(v_cmd) > 0:
            v_cmd = MIN_LIN_SPEED * (1 if v_cmd > 0 else -1)
        if abs(w_cmd) < MIN_ROT_SPEED and abs(w_cmd) > 0:
            w_cmd = MIN_ROT_SPEED * (1 if w_cmd > 0 else -1)
        left  = v_cmd - w_cmd
        right = v_cmd + w_cmd
        m = max(1.0, abs(left), abs(right))
        left  /= m
        right /= m
        #print(f"left: {left}, right: {right} ")
        left_power  = left
        right_power = right
        
        self.command_queue.put((left_power, right_power))
    
    def stop_motors(self):
        """Stop all motors."""
        self.command_queue.put((0.0, 0.0))


def main(filename=None, logfile=None):
    robot = Supervisor()
    #get devices for simulating motion capture
    gps = robot.getDevice(GPS_NAME)
    mouse = robot.getMouse()
    MOTION_CAPTURE_RATE = CONTROL_RATE

    mouse.enable(int(1000//MOTION_CAPTURE_RATE))
    mouse.enable3dPosition()

    marker = robot.getFromDef("WAYPOINT_MARKER")
    marker_translation = marker.getField("translation")

    gps.enable(int(1000//MOTION_CAPTURE_RATE))

    imu = robot.getDevice(IMU_NAME)
    imu.enable(int(1000//MOTION_CAPTURE_RATE))

    try:
        display = robot.getDevice(DISPLAY_NAME)
    except Exception:
        display = None

    left_motor = robot.getDevice(LEFT_MOTOR_NAME)
    right_motor = robot.getDevice(RIGHT_MOTOR_NAME)

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    robot_node = robot.getFromDef(ROBOT_DEF_NAME)
    if robot_node is None:
        print(f"[WARN] Could not find robot with DEF {ROBOT_DEF_NAME}; teleport will not work.")
    else:
        translation_field = robot_node.getField('translation')
        rotation_field = robot_node.getField('rotation')

    # ========== Shared state & controller ==========
    shared_state = {
        'x': 0.0,
        'y': 0.0,
        'yaw': 0.0,   # relative yaw to target
        'tracking': False,
        'status': 'Initializing',
    }

    nav_shared_state = {
        'state': STATE_IDLE
    }

    command_queue = Queue()
    navigation = NavigationController(shared_state, nav_shared_state, command_queue,
                 max_vel=MAX_VEL,
                 max_rot=MAX_ROT,
                 v_cruise=CRUISING_SPEED,
                 dist_kp=DIST_KP, dist_ki=DIST_KI, dist_kd=DIST_KD,
                 head_kp=HEAD_KP, head_ki=HEAD_KI, head_kd=HEAD_KD)
    

    nav_shared_state["state"] = STATE_IDLE
    dt = 1.0 / CONTROL_RATE
    if filename is None:
        sp_xm = None
        sp_ym = None
    else:
        with open(filename, "r") as f:
            cfg = yaml.safe_load(f)
        waypoints = cfg.get("waypoints", [])
        if not waypoints:
            print("No waypoints found in waypoints.yaml")
            return
        else:
            print(f"Loaded {len(waypoints)} waypoints from file.")
        wp_idx = 0
        sp_xm = float(waypoints[wp_idx]["x"])
        sp_ym = float(waypoints[wp_idx]["y"])

        def set_waypoint(i):
            nonlocal wp_idx, sp_xm, sp_ym
            wp_idx = i
            wp = waypoints[wp_idx]
            sp_xm = float(wp["x"])
            sp_ym = float(wp["y"])
            print(f"Waypoint {wp_idx} (from file):", sp_xm, sp_ym)
            marker_translation.setSFVec3f([sp_xm, sp_ym, 0.002])
            nav_shared_state['state'] = STATE_DRIVING
            navigation.set_is_final(wp_idx + 1 == len(waypoints))
            navigation.reset()
        
        if robot_node is not None:
            current_translation = translation_field.getSFVec3f()
            base_height = current_translation[1]
            first_wp = waypoints[0]
            start_x = float(first_wp["x"])
            start_y = float(first_wp["y"])

            # In Webots, ground plane is usually X-Z, Y is up.
            # We'll map waypoint (x, y) -> (x, base_height, y)
            translation_field.setSFVec3f([start_x, start_y, base_height])

            
            rotation_field.setSFRotation([0.0, 0.0, 1.0, math.pi/2])
            robot.step(int(1000 * dt))  # apply teleport
            print(f"Teleported robot to starting waypoint: ({start_x}, {start_y})")

        
        set_waypoint(1 if len(waypoints) > 1 else 0)
    latency_buffer = []
    start_time = time.perf_counter()
    log = []
    last_time = robot.getTime()
    steps = CONTROL_RATE * 2
    for _ in range(steps):
        if robot.step(int(1000 * dt)) == -1:
            break
    while robot.step(int(1000*dt)) != -1:
        now = robot.getTime()
        dt_meas = now - last_time
        last_time = now
        dt_meas = max(0.005, min(dt_meas, 0.1))
        # ========== "Motion capture": get robot pose from Webots ==========
        gps_vals = gps.getValues()        # [x, y, z] with y up
        imu_rpy = imu.getRollPitchYaw()   # [roll, pitch, yaw]

        # Map plane coordinates: world x,z as planar x,y
        x_world = gps_vals[0]
        y_world = gps_vals[1]
        z_world = gps_vals[2]

        roll = imu_rpy[0]
        pitch = imu_rpy[1]
        yaw = imu_rpy[2]

        yaw_deg = math.degrees(yaw- math.pi/2) #offset due to modeling things
        yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0
        cur_time = time.perf_counter() - start_time

        if filename is None:
            state = mouse.getState()
            if state.left:  # left mouse button pressed this step
                wx = state.x
                wy = state.y
                wz = state.z

                # Ignore invalid picks (e.g., if mouse is not over the 3D scene)
                if not (math.isnan(wx) or math.isnan(wy) or math.isnan(wz)):
                    # For a flat world, you usually navigate in the ground plane (x, y)
                    sp_xm = wx
                    sp_ym = wy
                    nav_shared_state["state"] = STATE_DRIVING
                print(f"New waypoint from mouse: x={wx:.3f}, y={wy:.3f}, z={wz:.3f}")
                marker_translation.setSFVec3f([wx, wy, 0.002])

            if sp_xm is not None:
                # Compute relative quantities to current waypoint
                rel_x = sp_xm - x_world
                rel_y = sp_ym - y_world

                target_yaw = math.degrees(math.atan2(rel_y, rel_x))

                rel_yaw = target_yaw - yaw_deg #FLIPPED FROM OROGINAL SCRIPT

                rel_yaw = (rel_yaw + 180.0) % 360.0 - 180.0  # normalize to [-180, 180]

                shared_state['x'] = rel_x
                shared_state['y'] = rel_y
                shared_state['yaw'] = rel_yaw
                navigation.update(dt=dt, is_stop=True)
        else:
            # Compute relative quantities to current waypoint
            rel_x = sp_xm - x_world
            rel_y = sp_ym - y_world

            target_yaw = math.degrees(math.atan2(rel_y, rel_x))

            rel_yaw = target_yaw - yaw_deg #FLIPPED FROM ORIGINAL_SCRIPT 

            rel_yaw = (rel_yaw + 180.0) % 360.0 - 180.0  # normalize to [-180, 180]

            shared_state['x'] = rel_x
            shared_state['y'] = rel_y
            shared_state['yaw'] = rel_yaw
            
            current_wp = waypoints[wp_idx]
            # navigation.update(dt=dt, is_stop=current_wp.get("stop", True))
            navigation.update(dt=dt_meas, is_stop=current_wp.get("stop", True))

        current = {
            "t": cur_time,
            "x": x_world,
            "y": y_world,
            "z": z_world,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "desired_x": sp_xm if sp_xm is not None else math.nan,
            "desired_y": sp_ym if sp_ym is not None else math.nan
        }

        log.append(current)
        #send bluetooth command
        latest_cmd = None
        while not command_queue.empty():
            latest_cmd = command_queue.get()
        if latest_cmd is not None:
            latency_buffer.append(latest_cmd)

        # Apply command after BLE_LATENCY_STEPS steps
        if len(latency_buffer) > BLE_LATENCY_STEPS:
            left_power, right_power = latency_buffer.pop(len(latency_buffer) - 1)
        else:
            left_power, right_power = 0.0, 0.0

        # Convert normalized [-1, 1] power to wheel velocities
        left_motor.setVelocity(left_power * MAX_WHEEL_SPEED)
        right_motor.setVelocity(right_power * MAX_WHEEL_SPEED)

        if display is not None:
            w = display.getWidth()
            h = display.getHeight()
            display.setColor(0xFFFFFF)
            display.fillRectangle(0, 0, w, h)
            display.setColor(0x000000)
            text1 = f"x={x_world:.3f}, y={y_world:.3f}, z={z_world:.3f}"
            text2 = f"roll={math.degrees(roll):.1f}, pitch={math.degrees(pitch):.1f}, yaw={yaw_deg:.1f}"
            text3 = f"left_motor={(left_power * MAX_WHEEL_SPEED):.3f}, right_motor={(right_power * MAX_WHEEL_SPEED):.3f}"
            display.drawText(text1, 2, 2)
            display.drawText(text2, 2, 18)
            display.drawText(text3, 2, 34)
            if sp_xm:
                text4 = f"rel_x={rel_x:.3f}, rel_y={rel_y:.3f}, rel_yaw={rel_yaw:.3f}, "
                display.drawText(text4, 2, 50)

        state = nav_shared_state['state']
        if filename is None:
            if state == STATE_ARRIVED:
                nav_shared_state['state'] = STATE_IDLE
                navigation.stop_motors()
                sp_xm = None
                sp_ym = None
                print("Waypoint reached")
        else:
            if state == STATE_ARRIVED:
                if wp_idx + 1 < len(waypoints):
                    next_idx = wp_idx + 1
                    print(f"Advancing to waypoint {next_idx}: {waypoints[next_idx]}")
                    set_waypoint(next_idx)
                    nav_shared_state['state'] = STATE_DRIVING
                else:
                    nav_shared_state['state'] = STATE_IDLE
                    navigation.stop_motors()
                    print("All waypoints completed.")
                    break
            elif state == STATE_FINISHED:
                navigation.stop_motors()
                print("All waypoints completed (final threshold reached).")
                if logfile is not None and log:
                    logs = log
                    fieldnames = list(logs[0].keys())
                    with open(logfile, "w", newline="") as f:
                        writer = csv.DictWriter(f, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerows(logs)
                break


if __name__ == '__main__':
    main(FILENAME, LOGFILE)
