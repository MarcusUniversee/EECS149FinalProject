#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Crazyflie Controller (UDP Receiver + PID)
-----------------------------------------
Runs in **Python 2.7**
Receives (x, y, z) from the AprilTag tracker through UDP.
Uses the same PID controllers as the original Kinect project.

Required process layout:
    - Python 3.11 runs:  position.py   (AprilTag tracker & UDP sender)
    - Python 2.7   runs: this script   (Crazyflie control)

Author: Jinhong's Project Version
"""

import sys
import socket
import logging
import signal
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie

# Import PID controllers from your PID file
from pid import PID, PID_RP

# ----------------- CONFIGURATION -----------------
UDP_IP = "127.0.0.1"     # receiving from same PC
UDP_PORT = 5005

ROLL_CAP = 15.0          # degrees
PITCH_CAP = 15.0         # degrees
THRUST_CAP = 55000       # raw thrust
BASE_THRUST = 38000      # hover offset
# --------------------------------------------------


class UDPReceiver(threading.Thread):
    """
    Thread that listens to UDP packets and stores the latest:
        self.x, self.y, self.z
    """

    def __init__(self, ip, port):
        threading.Thread.__init__(self)
        self.daemon = True

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))

        self.x = None
        self.y = None
        self.z = None

    def run(self):
        print("UDP Receiver started on {}:{}".format(UDP_IP, UDP_PORT))
        while True:
            data, addr = self.sock.recvfrom(1024)
            try:
                parts = data.decode().split(",")
                if len(parts) == 3:
                    self.x = float(parts[0])
                    self.y = float(parts[1])
                    self.z = float(parts[2])
            except:
                continue


class AprilTagPilot(object):

    def __init__(self):

        print("Initializing Crazyflie link...")
        cflib.crtp.init_drivers()
        self._cf = Crazyflie()

        # ---- Create PID controllers ----
        # roll PID controls left-right movement
        self.roll_pid = PID_RP(P=0.05, I=0.00025, D=1.0, set_point=0.0)

        # pitch PID controls forward-back movement
        self.pitch_pid = PID_RP(P=0.10, I=0.00025, D=1.0, set_point=0.0)

        # thrust PID controls altitude (z)
        self.thrust_pid = PID(P=30.0, I=40.0, D=500.0, set_point=0.0)

        # ---- Desired AprilTag pixel location (center of frame) ----
        self.sp_x = 320
        self.sp_y = 240
        self.sp_z = 0.4      # desired height in meters (adjust)

        # ---- Start UDP receiver ----
        self.udp = UDPReceiver(UDP_IP, UDP_PORT)
        self.udp.start()

        signal.signal(signal.SIGINT, signal.SIG_DFL)

    def connect(self, uri):
        self._cf.connectionFailed.add_callback(self._connection_failed)
        self._cf.open_link(uri)

    def _connection_failed(self, link, msg):
        print("Connection failed: {}".format(msg))
        sys.exit(-1)

    def control_loop(self):
        """
        Main flight control loop.
        Uses UDP (x,y,z) as measurement, PID to generate commands.
        """

        print("Starting control loop...")
        safety_count = 15

        while True:

            x = self.udp.x
            y = self.udp.y
            z = self.udp.z

            if x is not None and y is not None and z is not None:

                # reset failsafe counter
                safety_count = 15

                # ----- Compute pixel errors -----
                ex = self.sp_x - x      # left-right
                ey = self.sp_y - y      # forward-back
                ez = self.sp_z - z      # altitude

                # ----- PID outputs -----
                roll_cmd  = self.roll_pid.update(ex)
                pitch_cmd = self.pitch_pid.update(ey)
                thrust_cmd = self.thrust_pid.update(ez)

                # ----- Convert PID to setpoints -----
                roll_sp = max(-ROLL_CAP, min(ROLL_CAP, -roll_cmd))
                pitch_sp = max(-PITCH_CAP, min(PITCH_CAP, -pitch_cmd))
                thrust_sp = BASE_THRUST + thrust_cmd

                if thrust_sp > THRUST_CAP:
                    thrust_sp = THRUST_CAP
                if thrust_sp < 0:
                    thrust_sp = 0

                print("Roll={:.2f} Pitch={:.2f} Thrust={:.0f}".format(
                    roll_sp, pitch_sp, thrust_sp))

                # ----- Send command to Crazyflie -----
                self._cf.commander.send_setpoint(roll_sp, pitch_sp, 0, int(thrust_sp))

            else:
                # No UDP → reduce safety counter
                safety_count -= 1
                print("No vision data... safety countdown:", safety_count)

            if safety_count <= 0:
                print("Failsafe triggered! Stopping motors.")
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                break


def main():

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="radio://0/10/250K")
    args = parser.parse_args()

    pilot = AprilTagPilot()
    pilot.connect(args.uri)

    pilot.control_loop()


if __name__ == "__main__":
    main()
