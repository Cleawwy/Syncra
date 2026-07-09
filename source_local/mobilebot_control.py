#!/usr/bin/env python3
# cyl_robot_sender.py
# Sends wheel velocities to CylRobotExt via ZeroMQ PUSH.
# Legacy direct-visualization helper only.
# Syncra's main safe proof path is MQTT -> state service -> HTTP -> Omniverse.
# Do not use this file as a real robot control interface.
# pip install pyzmq
#
# Usage:
#   python cyl_robot_sender.py                          # spiral motion
#   python cyl_robot_sender.py --mode circle            # constant circle
#   python cyl_robot_sender.py --mode strafe            # lateral strafe
#   python cyl_robot_sender.py --mode spin              # spin in place
#   python cyl_robot_sender.py --mode figure8           # figure-8 path
#   python cyl_robot_sender.py --mode manual            # type wheel speeds
#   python cyl_robot_sender.py --mode stop              # send zero then exit
 
import argparse
import math
import time
import zmq
 
# =============================================================================
# WHEEL LAYOUT  (must match extension WHEEL_ANGLES_DEG)
# =============================================================================
WHEEL_ANGLES_DEG = [90.0, 210.0, 330.0]
WHEEL_OFFSET_R   = 0.18   # metres (18 cm)
 
# =============================================================================
# FORWARD KINEMATICS HELPER  (body velocity -> wheel velocities)
# Inverse of the extension's wheel_to_body_velocity()
#
# Given desired body velocity (vx, vz [m/s], omega [rad/s]) in robot-local frame,
# compute individual wheel speeds (m/s).
#
# Each wheel i contributes:
#   w_i = -vx*sin(A_i) + vz*cos(A_i) + omega*r
# =============================================================================
 
def body_to_wheels(vx: float, vz: float, omega: float) -> tuple:
    r = WHEEL_OFFSET_R
    wheels = []
    for deg in WHEEL_ANGLES_DEG:
        a = math.radians(deg)
        w = -vx * math.sin(a) + vz * math.cos(a) + omega * r
        wheels.append(w)
    return tuple(wheels)
 
 
# =============================================================================
# MOTION GENERATORS
# All return (w0, w1, w2) in m/s at time t (seconds)
# =============================================================================
 
def motion_spiral(t: float) -> tuple:
    # forward speed ramps up, while slowly spinning
    fwd   = 0.3 + 0.3 * math.sin(t * 0.2)   # 0..0.6 m/s
    omega = 0.2 * math.sin(t * 0.15)          # gentle yaw
    return body_to_wheels(0.0, fwd, omega)
 
 
def motion_circle(t: float, radius: float = 0.5) -> tuple:
    # constant forward + constant rotation for a circular path
    # radius (m): larger = bigger circle
    speed = 0.3
    omega = speed / radius
    return body_to_wheels(0.0, speed, omega)
 
 
def motion_strafe(t: float) -> tuple:
    # oscillate sideways (pure lateral motion)
    vx = 0.3 * math.sin(t * 0.8)
    return body_to_wheels(vx, 0.0, 0.0)
 
 
def motion_spin(t: float) -> tuple:
    # spin in place, oscillating direction
    omega = 0.8 * math.sin(t * 0.5)
    return body_to_wheels(0.0, 0.0, omega)
 
 
def motion_figure8(t: float) -> tuple:
    # figure-8: sinusoidal forward + sinusoidal yaw offset by 90 deg
    fwd   =  0.35 * math.cos(t * 0.4)
    omega =  0.6  * math.sin(t * 0.4)
    return body_to_wheels(0.0, fwd, omega)
 
 
MOTION_MAP = {
    "spiral":  motion_spiral,
    "circle":  motion_circle,
    "strafe":  motion_strafe,
    "spin":    motion_spin,
    "figure8": motion_figure8,
}
 
# =============================================================================
# PINO PLACEHOLDER
# Replace body of pino_wheels() with your model output.
# Input: t (float) in seconds
# Output: (w0, w1, w2) wheel speeds in m/s
# =============================================================================
 
def pino_wheels(t: float) -> tuple:
    # TODO: load your PINO model and call here
    # e.g.:
    #   pred = model.predict(t)   # returns [w0, w1, w2]
    #   return tuple(pred)
    return motion_spiral(t)   # fallback to spiral while testing
 
 
# =============================================================================
 
def send(sock, w0, w1, w2) -> None:
    sock.send_json({"w0": round(w0, 5), "w1": round(w1, 5), "w2": round(w2, 5)})
 
 
def _print_state(t, w0, w1, w2):
    vx, vz, omega = _fwd(w0, w1, w2)
    speed = math.sqrt(vx**2 + vz**2)
    print(f"  t={t:6.2f}s  w=({w0:.3f},{w1:.3f},{w2:.3f}) m/s"
          f"  body_speed={speed:.3f} m/s  omega={math.degrees(omega):.1f} deg/s")
 
 
def _fwd(w0, w1, w2):
    # simplified forward kin for display only
    A = [math.radians(d) for d in WHEEL_ANGLES_DEG]
    r = WHEEL_OFFSET_R
    vx = (1/3)*(-math.sin(A[0])*w0 - math.sin(A[1])*w1 - math.sin(A[2])*w2)
    vz = (1/3)*( math.cos(A[0])*w0 + math.cos(A[1])*w1 + math.cos(A[2])*w2)
    om = (w0 + w1 + w2) / (3 * r)
    return vx, vz, om
 
 
def mode_motion(sock, fn, hz, duration, label):
    interval = 1.0 / hz
    t0    = time.time()
    t_end = t0 + duration if duration > 0 else float("inf")
    print(f"[sender] mode={label}  hz={hz}  dur={'inf' if duration<=0 else duration}s")
    while time.time() < t_end:
        t = time.time() - t0
        w0, w1, w2 = fn(t)
        send(sock, w0, w1, w2)
        _print_state(t, w0, w1, w2)
        time.sleep(interval)
 
 
def mode_manual(sock):
    print("[sender] manual mode")
    print("  enter: w0 w1 w2  (m/s each wheel)")
    print("  enter: vx vz omega  (m/s, m/s, rad/s) with prefix 'b '")
    print("  enter: reset   to send reset command")
    print("  Ctrl+C to quit")
    while True:
        try:
            raw = input("cmd> ").strip()
            if not raw:
                continue
            if raw == "reset":
                sock.send_json({"reset": True})
                print("  sent reset")
                continue
            parts = raw.split()
            if parts[0] == "b" and len(parts) == 4:
                vx, vz, om = float(parts[1]), float(parts[2]), float(parts[3])
                w0, w1, w2 = body_to_wheels(vx, vz, om)
                send(sock, w0, w1, w2)
                print(f"  body ({vx},{vz},{om}) -> wheels ({w0:.3f},{w1:.3f},{w2:.3f})")
            elif len(parts) == 3:
                w0, w1, w2 = float(parts[0]), float(parts[1]), float(parts[2])
                send(sock, w0, w1, w2)
                _print_state(0, w0, w1, w2)
            else:
                print("  usage: w0 w1 w2   or   b vx vz omega")
        except KeyboardInterrupt:
            break
        except ValueError:
            print("  invalid input")
 
 
def main():
    parser = argparse.ArgumentParser(description="Cylindrical robot wheel velocity sender")
    parser.add_argument("--host",     default="127.0.0.1")
    parser.add_argument("--port",     type=int,   default=5557)
    parser.add_argument("--mode",     default="spiral",
                        choices=["spiral","circle","strafe","spin","figure8","manual","pino","stop"])
    parser.add_argument("--hz",       type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=0,
                        help="run time in seconds (0 = infinite)")
    parser.add_argument("--radius",   type=float, default=0.5,
                        help="circle radius in metres (circle mode only)")
    args = parser.parse_args()
 
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    addr = f"tcp://{args.host}:{args.port}"
    sock.connect(addr)
    time.sleep(0.1)
    print(f"[sender] connected to {addr}")
 
    try:
        if args.mode == "stop":
            send(sock, 0.0, 0.0, 0.0)
            print("[sender] sent stop (all wheels zero)")
 
        elif args.mode == "manual":
            mode_manual(sock)
 
        elif args.mode == "circle":
            mode_motion(sock,
                        lambda t: motion_circle(t, args.radius),
                        args.hz, args.duration, "circle")
 
        elif args.mode == "pino":
            mode_motion(sock, pino_wheels, args.hz, args.duration, "pino")
 
        elif args.mode in MOTION_MAP:
            mode_motion(sock, MOTION_MAP[args.mode],
                        args.hz, args.duration, args.mode)
 
    except KeyboardInterrupt:
        print("\n[sender] interrupted")
        send(sock, 0.0, 0.0, 0.0)   # safety stop
 
    print("[sender] done")
    sock.close()
    ctx.term()
 
 
if __name__ == "__main__":
    main()
