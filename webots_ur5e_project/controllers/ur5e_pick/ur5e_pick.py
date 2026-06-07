"""
UR5e Minimal Pick-and-Place Controller  (v2 - fixed motor sequencing)
=====================================================================
Uses GPS-calibrated joint angles + Connector magnetic grasping.
Moves joints in safe sequence to avoid wrist motor stall.
"""

import sys
import os
import math

try:
    from controller import Robot
except ImportError:
    sys.exit("Must be run from Webots.")

TIME_STEP = 16

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output.log")
_log_file = open(LOG_PATH, "w", encoding="utf-8")
_orig_print = print
def print(*args, **kwargs):
    _orig_print(*args, **kwargs)
    _log_file.write(" ".join(str(a) for a in args) + "\n")
    _log_file.flush()

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
SENSOR_NAMES = [n + "_sensor" for n in JOINT_NAMES]

HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

# Calibration result: these reach GPS (0.441, -0.037, 0.780), d=0.049m from target
PICK_GRASP = [0.79, -0.50, 1.90, 0.171, 0.0, 0.0]
PICK_ABOVE = [0.79, -0.80, 1.90, 0.471, 0.0, 0.0]

# Place = pick rotated 180 degrees around shoulder_pan
PLACE_GRASP = [0.79 - math.pi, -0.50, 1.90, 0.171, 0.0, 0.0]
PLACE_ABOVE = [0.79 - math.pi, -0.80, 1.90, 0.471, 0.0, 0.0]

TARGET_POS = [0.45, 0.0, 0.81]
MAX_VEL = 1.5


def dist3(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def generate_calibration_poses():
    poses = []
    for sp in [-1.57, -0.79, 0.0, 0.79, 1.57]:
        for sl in [-0.5, -0.8, -1.1, -1.4]:
            for el in [0.4, 0.9, 1.4, 1.9]:
                w1 = math.pi / 2.0 - sl - el
                if -3.14 < w1 < 3.14:
                    poses.append([sp, sl, el, w1, 0.0, 0.0])
    return poses


class UR5ePickController:

    def __init__(self):
        self.robot = Robot()
        self.ts = TIME_STEP
        self.motors = []
        self.sensors = []
        for jn, sn in zip(JOINT_NAMES, SENSOR_NAMES):
            m = self.robot.getDevice(jn)
            if m:
                m.setVelocity(MAX_VEL)
            self.motors.append(m)
            s = self.robot.getDevice(sn)
            if s:
                s.enable(self.ts)
            self.sensors.append(s)

        self.gps = self.robot.getDevice("tool_gps")
        if self.gps:
            self.gps.enable(self.ts)

        self.connector = self.robot.getDevice("connector")
        if self.connector:
            self.connector.enablePresence(self.ts)

        for _ in range(4):
            self.robot.step(self.ts)

    def get_joints(self):
        return [s.getValue() if s else 0.0 for s in self.sensors]

    def set_joints(self, pos):
        for m, p in zip(self.motors, pos):
            if m:
                m.setPosition(p)

    def gps_pos(self):
        return list(self.gps.getValues()) if self.gps else [0, 0, 0]

    def wait_ms(self, ms):
        for _ in range(max(1, ms // self.ts)):
            self.robot.step(self.ts)

    def wait_reach(self, target, timeout_ms=15000, threshold=0.08):
        elapsed = 0
        while elapsed < timeout_ms:
            self.robot.step(self.ts)
            elapsed += self.ts
            cur = self.get_joints()
            if all(abs(c - t) < threshold for c, t in zip(cur, target)):
                return True
        return False

    def move_to(self, target, label="", settle_ms=2000):
        """Move all joints at max velocity."""
        for m in self.motors:
            if m:
                m.setVelocity(MAX_VEL)
        self.set_joints(target)
        reached = self.wait_reach(target)
        self.wait_ms(settle_ms)
        gps = self.gps_pos()
        cur = self.get_joints()
        tag = "OK" if reached else "TIMEOUT"
        if label:
            print(f"  [{tag:7s}] {label}")
            print(f"           Joints: [{', '.join(f'{j:+.3f}' for j in cur)}]")
            print(f"           GPS:    ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")
        return gps, reached

    def move_sequenced(self, target, label="", settle_ms=2000):
        """Move to target in two phases: first wrist, then the rest.
        Prevents wrist motor stall from large simultaneous movements."""
        for m in self.motors:
            if m:
                m.setVelocity(MAX_VEL)

        cur = self.get_joints()
        wrist_diff = abs(target[3] - cur[3])

        if wrist_diff > 0.5:
            phase1 = list(cur)
            phase1[3] = target[3]
            phase1[4] = target[4]
            phase1[5] = target[5]
            self.set_joints(phase1)
            self.wait_reach(phase1, timeout_ms=8000)
            self.wait_ms(500)
            print(f"    (wrist pre-positioned, Δ={wrist_diff:.2f} rad)")

        self.set_joints(target)
        reached = self.wait_reach(target)
        self.wait_ms(settle_ms)
        gps = self.gps_pos()
        cur = self.get_joints()
        tag = "OK" if reached else "TIMEOUT"
        if label:
            print(f"  [{tag:7s}] {label}")
            print(f"           Joints: [{', '.join(f'{j:+.3f}' for j in cur)}]")
            print(f"           GPS:    ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")
        return gps, reached

    def calibrate(self):
        print("\n" + "=" * 60)
        print("  CALIBRATION PHASE")
        print(f"  Target: ({TARGET_POS[0]:.3f}, {TARGET_POS[1]:.3f}, {TARGET_POS[2]:.3f})")
        print("=" * 60)

        poses = generate_calibration_poses()
        print(f"  Testing {len(poses)} candidate poses ...\n")
        self.move_to(HOME, "HOME (start)", settle_ms=500)

        best_pose = None
        best_dist = float("inf")
        best_gps = None
        results = []

        for i, pose in enumerate(poses):
            self.set_joints(pose)
            self.wait_reach(pose, timeout_ms=5000, threshold=0.12)
            self.wait_ms(400)
            gps = self.gps_pos()
            d = dist3(gps, TARGET_POS)
            results.append((d, list(pose), list(gps)))
            if d < best_dist:
                best_dist = d
                best_pose = list(pose)
                best_gps = list(gps)
                print(f"  [{i+1:3d}/{len(poses)}] d={d:.3f}m "
                      f"GPS=({gps[0]:+.3f},{gps[1]:+.3f},{gps[2]:+.3f}) *** BEST")
            elif (i + 1) % 20 == 0:
                print(f"  [{i+1:3d}/{len(poses)}] d={d:.3f}m (best so far: {best_dist:.3f}m)")

        self.move_to(HOME, "HOME (end calibration)", settle_ms=500)

        results.sort(key=lambda r: r[0])
        print(f"\n  TOP 5:")
        for rank, (d, p, g) in enumerate(results[:5]):
            print(f"    #{rank+1}: d={d:.3f}m  J=[{', '.join(f'{v:+.2f}' for v in p)}]"
                  f"  GPS=({g[0]:+.3f},{g[1]:+.3f},{g[2]:+.3f})")

        print(f"\n  BEST: d={best_dist:.3f}m  "
              f"Pose=[{', '.join(f'{v:+.3f}' for v in best_pose)}]")

        return best_pose, best_dist, results

    def grab(self):
        if not self.connector:
            print("  [ERROR] No connector!")
            return False
        presence = self.connector.getPresence()
        gps = self.gps_pos()
        d = dist3(gps, TARGET_POS)
        print(f"\n  >> Connector presence: {presence}")
        print(f"     GPS: ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")
        print(f"     Distance to target: {d:.4f}m")
        self.connector.lock()
        self.wait_ms(1500)
        p2 = self.connector.getPresence()
        print(f"  >> LOCKED (presence after: {p2})")
        return True

    def release(self):
        if self.connector:
            self.connector.unlock()
            self.wait_ms(1000)
            print("  >> UNLOCKED (released)")

    def pick_and_place(self, grasp, above, place_grasp, place_above):
        print("\n" + "=" * 60)
        print("  PICK-AND-PLACE EXECUTION")
        print(f"  GRASP:       [{', '.join(f'{v:+.3f}' for v in grasp)}]")
        print(f"  ABOVE:       [{', '.join(f'{v:+.3f}' for v in above)}]")
        print(f"  PLACE_GRASP: [{', '.join(f'{v:+.3f}' for v in place_grasp)}]")
        print("=" * 60)

        print("\n--- 1. HOME ---")
        self.move_to(HOME, "HOME")

        print("\n--- 2. ABOVE PICK (sequenced) ---")
        self.move_sequenced(above, "ABOVE PICK")

        print("\n--- 3. DESCEND TO GRASP ---")
        self.move_to(grasp, "GRASP POSITION")

        print("\n--- 4. GRAB ---")
        self.grab()

        print("\n--- 5. LIFT ---")
        self.move_to(above, "LIFT")

        print("\n--- 6. HOME (transit) ---")
        self.move_sequenced(HOME, "HOME (transit)", settle_ms=1000)

        print("\n--- 7. ABOVE PLACE (sequenced) ---")
        self.move_sequenced(place_above, "ABOVE PLACE")

        print("\n--- 8. LOWER TO PLACE ---")
        self.move_to(place_grasp, "PLACE POSITION")

        print("\n--- 9. RELEASE ---")
        self.release()

        print("\n--- 10. RETREAT ---")
        self.move_to(place_above, "RETREAT")

        print("\n--- 11. HOME ---")
        self.move_sequenced(HOME, "HOME (done)")

        print("\n" + "=" * 60)
        print("  PICK-AND-PLACE COMPLETE!")
        print("=" * 60)

    def run(self):
        print()
        print("=" * 60)
        print("  UR5e Pick-and-Place Controller v2")
        print("  GPS calibration + Connector grab")
        print("  Sequenced wrist movement for reliability")
        print("=" * 60)

        grasp, calib_dist, results = self.calibrate()

        if calib_dist > 0.15:
            print(f"\n  [WARN] Best distance {calib_dist:.3f}m > 0.15m tolerance!")
            print(f"  Connector may not lock. Adjust target in .wbt file.")
        else:
            print(f"\n  [OK] Best distance {calib_dist:.3f}m within tolerance!")

        above_candidates = [(d, p, g) for d, p, g in results
                            if abs(p[0] - grasp[0]) < 0.1 and d < 0.3
                            and p[1] < grasp[1] - 0.1]
        if above_candidates:
            above_candidates.sort(key=lambda r: r[0])
            above = above_candidates[0][1]
            print(f"  Using calibrated ABOVE pose: [{', '.join(f'{v:+.2f}' for v in above)}]")
        else:
            above = list(grasp)
            above[1] -= 0.30
            above[3] = math.pi / 2.0 - above[1] - above[2]
            print(f"  Using derived ABOVE pose: [{', '.join(f'{v:+.2f}' for v in above)}]")

        place_grasp = list(grasp)
        place_grasp[0] -= math.pi
        place_above = list(above)
        place_above[0] -= math.pi

        self.pick_and_place(grasp, above, place_grasp, place_above)

        print(f"\n  Calibrated GRASP: [{', '.join(f'{v:.4f}' for v in grasp)}]")
        print(f"  Distance to target: {calib_dist:.3f}m")
        print(f"\n  Idling. Check Webots for visual result.")

        while self.robot.step(self.ts) != -1:
            pass


if __name__ == "__main__":
    UR5ePickController().run()
