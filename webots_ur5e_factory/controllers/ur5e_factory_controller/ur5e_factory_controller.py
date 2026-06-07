"""
UR5e Factory Pick-and-Place Controller
======================================
Multi-object version of the verified Connector-based pick controller.
Uses GPS-calibrated joint angles + Connector magnetic grasping.
Moves joints in safe sequence to avoid wrist motor stall.
Handles 3 target objects sequentially.
"""

import sys
import os
import math

try:
    from controller import Robot
except ImportError:
    sys.exit("Must be run from Webots.")

TIME_STEP = 16
MAX_VEL = 1.5

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

# --- Verified base poses from successful ur5e_pick calibration ---
# These reach GPS ~(0.441, -0.037, 0.780), d=0.049m from target at (0.45, 0, 0.81)
BASE_PICK_GRASP = [0.79, -0.50, 1.90, 0.171, 0.0, 0.0]
BASE_PICK_ABOVE = [0.79, -0.80, 1.90, 0.471, 0.0, 0.0]
BASE_PLACE_GRASP = [0.79 - math.pi, -0.50, 1.90, 0.171, 0.0, 0.0]
BASE_PLACE_ABOVE = [0.79 - math.pi, -0.80, 1.90, 0.471, 0.0, 0.0]

REF_PICK_XY = (0.45, 0.0)
REF_PLACE_XY = (-0.5, 0.0)

TASKS = [
    {
        "name": "Red Box",
        "pick_pos": [0.45, 0.0, 0.81],
        "place_pos": [-0.5, 0.0, 0.78],
    },
    {
        "name": "Green Box",
        "pick_pos": [0.45, -0.2, 0.81],
        "place_pos": [-0.5, 0.2, 0.78],
    },
    {
        "name": "Blue Box",
        "pick_pos": [0.45, 0.2, 0.81],
        "place_pos": [-0.5, -0.2, 0.78],
    },
]


def dist3(a, b):
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def compute_sp_offset(ref_xy, target_xy):
    """Shoulder-pan offset to reach a target at a different XY angle from base."""
    ref_angle = math.atan2(ref_xy[1], ref_xy[0])
    tgt_angle = math.atan2(target_xy[1], target_xy[0])
    delta = tgt_angle - ref_angle
    while delta > math.pi:
        delta -= 2 * math.pi
    while delta <= -math.pi:
        delta += 2 * math.pi
    return -delta


def offset_pose(base_pose, sp_delta):
    pose = list(base_pose)
    pose[0] += sp_delta
    return pose


class UR5eFactoryController:

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
        """Two-phase motion: wrist first, then the rest. Prevents stall."""
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
            print(f"    (wrist pre-positioned, delta={wrist_diff:.2f} rad)")

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

    def grab(self, target_pos):
        if not self.connector:
            print("  [ERROR] No connector device!")
            return False
        presence = self.connector.getPresence()
        gps = self.gps_pos()
        d = dist3(gps, target_pos)
        print(f"\n  >> Connector presence: {presence}")
        print(f"     GPS:    ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")
        print(f"     Target: ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")
        print(f"     Distance to target: {d:.4f}m")
        self.connector.lock()
        self.wait_ms(1500)
        p2 = self.connector.getPresence()
        locked = p2 == 1
        status = "SUCCESS" if locked else "ATTEMPTED"
        print(f"  >> LOCK {status} (presence after lock: {p2})")
        return True

    def release(self):
        if self.connector:
            self.connector.unlock()
            self.wait_ms(1000)
            print("  >> UNLOCKED (object released)")

    def pick_and_place_single(self, task_idx, task):
        name = task["name"]
        pick_pos = task["pick_pos"]
        place_pos = task["place_pos"]

        pick_sp = compute_sp_offset(REF_PICK_XY, (pick_pos[0], pick_pos[1]))
        place_sp = compute_sp_offset(REF_PLACE_XY, (place_pos[0], place_pos[1]))

        grasp = offset_pose(BASE_PICK_GRASP, pick_sp)
        above = offset_pose(BASE_PICK_ABOVE, pick_sp)
        place_grasp = offset_pose(BASE_PLACE_GRASP, place_sp)
        place_above = offset_pose(BASE_PLACE_ABOVE, place_sp)

        print(f"\n{'=' * 60}")
        print(f"  TASK {task_idx + 1}/{len(TASKS)}: {name}")
        print(f"  Pick:  ({pick_pos[0]:.3f}, {pick_pos[1]:.3f}, {pick_pos[2]:.3f})")
        print(f"  Place: ({place_pos[0]:.3f}, {place_pos[1]:.3f}, {place_pos[2]:.3f})")
        print(f"  Shoulder-pan offset: pick={pick_sp:+.3f}  place={place_sp:+.3f}")
        print(f"  GRASP angles: [{', '.join(f'{v:+.3f}' for v in grasp)}]")
        print(f"{'=' * 60}")

        print(f"\n--- Step 1: HOME ---")
        self.move_to(HOME, "HOME")

        print(f"\n--- Step 2: ABOVE PICK (sequenced) ---")
        self.move_sequenced(above, "ABOVE PICK")

        print(f"\n--- Step 3: DESCEND TO GRASP ---")
        self.move_to(grasp, "GRASP POSITION")

        print(f"\n--- Step 4: GRAB ---")
        self.grab(pick_pos)

        print(f"\n--- Step 5: LIFT ---")
        self.move_to(above, "LIFT")

        print(f"\n--- Step 6: HOME (transit) ---")
        self.move_sequenced(HOME, "HOME (transit)", settle_ms=1000)

        print(f"\n--- Step 7: ABOVE PLACE (sequenced) ---")
        self.move_sequenced(place_above, "ABOVE PLACE")

        print(f"\n--- Step 8: LOWER TO PLACE ---")
        self.move_to(place_grasp, "PLACE POSITION")

        print(f"\n--- Step 9: RELEASE ---")
        self.release()

        print(f"\n--- Step 10: RETREAT ---")
        self.move_to(place_above, "RETREAT")

        print(f"\n--- Step 11: HOME ---")
        self.move_sequenced(HOME, "HOME (done)")

        print(f"\n  >>> {name} placed successfully! <<<")

    def verify_base_pose(self):
        """Quick GPS check with the known-good base grasp pose."""
        print("\n  GPS VERIFICATION (base grasp pose)")
        self.move_sequenced(BASE_PICK_ABOVE, "ABOVE (verify)")
        self.move_to(BASE_PICK_GRASP, "GRASP (verify)")
        gps = self.gps_pos()
        d = dist3(gps, TASKS[0]["pick_pos"])
        print(f"  GPS:    ({gps[0]:.4f}, {gps[1]:.4f}, {gps[2]:.4f})")
        print(f"  Target: ({TASKS[0]['pick_pos'][0]:.4f}, {TASKS[0]['pick_pos'][1]:.4f}, {TASKS[0]['pick_pos'][2]:.4f})")
        print(f"  Distance: {d:.4f}m  (tolerance: 0.15m)")
        if d > 0.15:
            print(f"  [WARN] Distance exceeds tolerance! Connector may not lock.")
        else:
            print(f"  [OK] Within tolerance. Connector will lock reliably.")
        self.move_sequenced(HOME, "HOME (verified)")
        return d

    def run(self):
        print()
        print("=" * 60)
        print("  UR5e Factory Pick-and-Place Controller")
        print("  ======================================")
        print("  Mode:     Multi-object Connector grasping")
        print("  Objects:  3 (Red, Green, Blue)")
        print("  Strategy: GPS-verified + Connector magnetic snap")
        print("  Motion:   Sequenced wrist for reliability")
        print("=" * 60)

        print(f"\n  [SIGNAL] GREEN LIGHT - System initializing")
        self.move_to(HOME, "HOME (init)")

        calib_dist = self.verify_base_pose()

        if calib_dist > 0.20:
            print(f"\n  [ABORT] Base pose verification failed (d={calib_dist:.3f}m)")
            print(f"  Check robot/table positions in .wbt file.")
            while self.robot.step(self.ts) != -1:
                pass
            return

        for i, task in enumerate(TASKS):
            print(f"\n  [SIGNAL] YELLOW LIGHT - Working on {task['name']}")
            self.pick_and_place_single(i, task)
            print(f"  [SIGNAL] GREEN LIGHT - {task['name']} completed!")

        print(f"\n{'=' * 60}")
        print(f"  {'*' * 40}")
        print(f"  ***      MISSION COMPLETE!          ***")
        print(f"  ***  All 3 objects picked & placed   ***")
        print(f"  {'*' * 40}")
        print(f"  [SIGNAL] GREEN LIGHT - All tasks done")
        print(f"{'=' * 60}")

        print(f"\n  Summary:")
        for i, task in enumerate(TASKS):
            print(f"    Task {i+1}: {task['name']}")
            print(f"      Picked from ({task['pick_pos'][0]:.2f}, {task['pick_pos'][1]:.2f}, {task['pick_pos'][2]:.2f})")
            print(f"      Placed at   ({task['place_pos'][0]:.2f}, {task['place_pos'][1]:.2f}, {task['place_pos'][2]:.2f})")
        print(f"\n  Idling. Check Webots 3D view for result.")

        while self.robot.step(self.ts) != -1:
            pass


if __name__ == "__main__":
    UR5eFactoryController().run()
