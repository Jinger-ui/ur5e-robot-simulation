"""
UR5e Joint Angle Test / Calibration Controller
================================================
Sweeps through a fine grid of joint angles and prints GPS positions.
Helps find the exact angles needed to reach any target position.

Usage: Set this as the controller in the world file, run Webots,
       and read the console output to find the best angles.
"""

import sys
import math

try:
    from controller import Robot
except ImportError:
    sys.exit("Must be run from Webots.")

TIME_STEP = 16

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
SENSOR_NAMES = [n + "_sensor" for n in JOINT_NAMES]

HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]


def dist3(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


class UR5eTestController:

    def __init__(self):
        self.robot = Robot()
        self.ts = TIME_STEP

        self.motors = []
        self.sensors = []
        for jn, sn in zip(JOINT_NAMES, SENSOR_NAMES):
            m = self.robot.getDevice(jn)
            if m:
                m.setVelocity(1.0)
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

    def set_joints(self, pos):
        for m, p in zip(self.motors, pos):
            if m:
                m.setPosition(p)

    def get_joints(self):
        return [s.getValue() if s else 0.0 for s in self.sensors]

    def gps_pos(self):
        if self.gps:
            return list(self.gps.getValues())
        return [0, 0, 0]

    def wait_reach(self, target, timeout_ms=8000, threshold=0.08):
        elapsed = 0
        while elapsed < timeout_ms:
            self.robot.step(self.ts)
            elapsed += self.ts
            cur = self.get_joints()
            if all(abs(c - t) < threshold for c, t in zip(cur, target)):
                return True
        return False

    def move_and_report(self, pose, label=""):
        self.set_joints(pose)
        self.wait_reach(pose, timeout_ms=5000)
        for _ in range(max(1, 500 // self.ts)):
            self.robot.step(self.ts)
        gps = self.gps_pos()
        joints = self.get_joints()
        presence = self.connector.getPresence() if self.connector else -1
        return gps, joints, presence

    def run(self):
        print()
        print("=" * 70)
        print("  UR5e Joint Angle Test Controller")
        print("  Sweeps angles and prints GPS positions for calibration")
        print("=" * 70)

        self.set_joints(HOME)
        for _ in range(max(1, 3000 // self.ts)):
            self.robot.step(self.ts)
        gps = self.gps_pos()
        print(f"\n  HOME GPS: ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")

        target_pos = [0.45, 0.0, 0.81]
        print(f"  Target:   ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")

        print(f"\n{'='*70}")
        print(f"  PHASE 1: Coarse sweep (shoulder_pan x shoulder_lift x elbow)")
        print(f"{'='*70}")
        print(f"  {'#':>4s}  {'sp':>6s} {'sl':>6s} {'el':>6s} {'w1':>6s}  "
              f"{'GPS_X':>7s} {'GPS_Y':>7s} {'GPS_Z':>7s}  {'dist':>6s} {'P':>2s}")

        all_results = []
        idx = 0

        for sp in [-2.36, -1.57, -0.79, 0.0, 0.79, 1.57, 2.36]:
            for sl in [-0.4, -0.6, -0.8, -1.0, -1.2, -1.4]:
                for el in [0.3, 0.6, 0.9, 1.2, 1.5, 1.8]:
                    w1 = math.pi / 2.0 - sl - el
                    if w1 < -3.14 or w1 > 3.14:
                        continue
                    pose = [sp, sl, el, w1, 0.0, 0.0]
                    idx += 1

                    gps, joints, presence = self.move_and_report(pose)
                    d = dist3(gps, target_pos)
                    all_results.append((d, list(pose), list(gps), presence))

                    if d < 0.20 or idx <= 5 or idx % 20 == 0:
                        p_str = "Y" if presence > 0 else "."
                        print(f"  {idx:4d}  {sp:+6.2f} {sl:+6.2f} {el:+6.2f} "
                              f"{w1:+6.2f}  {gps[0]:+7.3f} {gps[1]:+7.3f} "
                              f"{gps[2]:+7.3f}  {d:6.3f} {p_str:>2s}"
                              + (" ***" if d < 0.10 else ""))

        all_results.sort(key=lambda r: r[0])

        print(f"\n{'='*70}")
        print(f"  RESULTS: Top 10 closest to target")
        print(f"{'='*70}")
        for rank, (d, p, g, pr) in enumerate(all_results[:10]):
            print(f"  #{rank+1:2d}  d={d:.4f}m  "
                  f"GPS=({g[0]:+.4f},{g[1]:+.4f},{g[2]:+.4f})  "
                  f"Joints=[{', '.join(f'{v:+.3f}' for v in p)}]  "
                  f"Presence={'Y' if pr > 0 else 'N'}")

        if all_results:
            best = all_results[0]
            print(f"\n  BEST PICK POSE: [{', '.join(f'{v:.4f}' for v in best[1])}]")
            print(f"  GPS position:   ({best[2][0]:.4f}, {best[2][1]:.4f}, {best[2][2]:.4f})")
            print(f"  Distance to target: {best[0]:.4f}m")

            if best[0] <= 0.15:
                print(f"  [OK] Within connector tolerance (0.15m)")
            else:
                print(f"  [WARN] Outside connector tolerance. Adjust target position in .wbt")
                print(f"         Suggested target translation: "
                      f"{best[2][0]:.3f} {best[2][1]:.3f} {best[2][2] - 0.035:.3f}")

            print(f"\n  Paste this into ur5e_pick.py as GRASP pose:")
            print(f"  GRASP = [{', '.join(f'{v:.4f}' for v in best[1])}]")

            approach = list(best[1])
            approach[1] += 0.25
            approach[2] -= 0.30
            approach[3] = math.pi / 2.0 - approach[1] - approach[2]
            print(f"  APPROACH = [{', '.join(f'{v:.4f}' for v in approach)}]")

        print(f"\n  Total poses tested: {idx}")
        print(f"  Done. Idling.")

        while self.robot.step(self.ts) != -1:
            pass


if __name__ == "__main__":
    UR5eTestController().run()
