#!/usr/bin/env python3
"""
Generate the robot arm clips shown by the motion viewer: a 6-axis arm on a workbench that
wipes the table, assembles a stack of parts and drives screws into a housing.

    python tools/make_robot_clips.py [--out data/robot]

Each clip is a BVH file (centimeters, Y up, 60 fps) and a .scene text file next to it with the
workcell (static boxes and cylinders), the shapes of the props, the dirt to clean and the
program steps. The props (parts, screws) are extra joints of the BVH, so the clips also open
in other BVH tools.

The motion is planned like a robot program: joint moves (MoveJ) between tasks and straight
tool moves (MoveL) near the parts, with minimum jerk timing, solved by analytic inverse
kinematics (spherical wrist, elbow up).
"""
import argparse
import math
import os
import sys

import numpy as np

FPS = 60
SURFACE = 75.0                         # workbench top
BASE = np.array([0.0, SURFACE, 0.0])   # robot mounted on the workbench
# link lengths (cm): base -> turret -> shoulder, upper arm, forearm, wrist 1 -> wrist 2, wrist 3 -> flange
H_TURRET, H_SHOULDER, L_UPPER, L_FORE, L_WRIST, L_FLANGE = 9.0, 8.0, 42.5, 39.2, 6.0, 8.0
ARM = [("Turret", "Y", (0, H_TURRET, 0)), ("Shoulder", "X", (0, H_SHOULDER, 0)), ("Elbow", "X", (0, L_UPPER, 0)),
       ("Wrist1", "Y", (0, L_FORE, 0)), ("Wrist2", "X", (0, L_WRIST, 0)), ("Wrist3", "Y", (0, 0, 0))]


# ---------------------------------------------------------------- math

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def tool_down(yaw):
    """tool frame whose +Y (the approach direction) points down, turned by yaw around the vertical"""
    return rot_y(yaw) @ rot_x(math.pi)


def closest_turn(a, ref):
    """a + 2 pi k, closest to ref"""
    return a + 2 * math.pi * round((ref - a) / (2 * math.pi))


def min_jerk(t):
    return t * t * t * (10 - 15 * t + 6 * t * t)


def euler_yxz(R):
    """R = Ry(a) Rx(b) Rz(c), in degrees (the order of the BVH channels written for the props)"""
    b = math.asin(max(-1.0, min(1.0, -R[1, 2])))
    return math.degrees(math.atan2(R[0, 2], R[2, 2])), math.degrees(b), math.degrees(math.atan2(R[1, 0], R[1, 1]))


# ---------------------------------------------------------------- robot

class Arm:
    def __init__(self, tool_length):
        self.tool = tool_length  # flange -> tool center point (TCP)

    def frames(self, q, spindle=0.0):
        """world position and rotation of every joint (q in radians)"""
        p, R = BASE.copy(), np.eye(3)
        out = {"Base": (p, R)}
        for (name, axis, offset), a in zip(ARM, q):
            p = p + R @ np.array(offset, dtype=float)
            R = R @ (rot_y(a) if axis == "Y" else rot_x(a))
            out[name] = (p, R)
        p = p + R @ np.array([0, L_FLANGE, 0])
        R = R @ rot_y(spindle)
        out["Tool"] = (p, R)
        out["TCP"] = (p + R @ np.array([0, self.tool, 0]), R)
        return out

    def ik(self, p, R, ref):
        """joint angles that put the TCP at p with the tool rotation R (spindle at 0), closest to ref"""
        c = p - R @ np.array([0, L_FLANGE + self.tool, 0]) - BASE   # wrist center, base space
        q0 = closest_turn(math.atan2(c[0], c[2]), ref[0])
        r, h = math.hypot(c[0], c[2]), c[1] - H_TURRET - H_SHOULDER
        l2 = L_FORE + L_WRIST
        cos2 = (r * r + h * h - L_UPPER ** 2 - l2 ** 2) / (2 * L_UPPER * l2)
        if abs(cos2) > 1.0:
            raise ValueError("target out of reach: %s" % np.round(p, 1))
        q2 = math.acos(cos2)  # elbow up
        q1 = math.atan2(r, h) - math.atan2(l2 * math.sin(q2), L_UPPER + l2 * math.cos(q2))
        M = (rot_y(q0) @ rot_x(q1 + q2)).T @ R  # what the wrist has to do: Ry(q3) Rx(q4) Ry(q5)
        q4 = math.acos(max(-1.0, min(1.0, M[1, 1])))
        best = None
        for s in (1.0, -1.0):
            if abs(math.sin(q4)) < 1e-6:  # wrist singularity: keep wrist 1 where it was
                a = ref[3]
                c5 = math.atan2(M[0, 2], M[0, 0]) - a
            else:
                a = math.atan2(s * M[0, 1], s * M[2, 1])
                c5 = math.atan2(s * M[1, 0], -s * M[1, 2])
            q = np.array([q0, q1, q2, closest_turn(a, ref[3]), s * q4, closest_turn(c5, ref[5])])
            cost = np.sum((q[3:] - ref[3:]) ** 2)
            if best is None or cost < best[0]:
                best = (cost, q)
        return best[1]


class Prop:
    def __init__(self, name, p, yaw=0.0):
        self.name = name
        self.p = np.array(p, dtype=float)
        self.R = rot_y(math.radians(yaw))
        self.grip = None  # (rotation, offset) in the tool frame while it is held
        self.track = []


class Program:
    """robot program that records one frame per 1/60 s"""

    def __init__(self, tool_length, home, home_yaw=0.0, fingers=None):
        self.arm = Arm(tool_length)
        self.q = self.arm.ik(np.array(home, dtype=float), tool_down(home_yaw), np.zeros(6))
        self.yaw = home_yaw
        self.spindle = 0.0
        self.fingers = fingers  # half opening of the gripper (cm), None: no gripper
        self.props = []
        self.track = []
        self.steps = []

    def tcp(self):
        return self.arm.frames(self.q, self.spindle)["TCP"]

    def emit(self):
        p, R = self.tcp()
        for pr in self.props:
            if pr.grip is not None:
                pr.R = R @ pr.grip[0]
                pr.p = p + R @ pr.grip[1]
            pr.track.append((pr.p.copy(), pr.R.copy()))
        self.track.append((self.q.copy(), self.spindle, self.fingers))

    def step(self, text):
        self.steps.append((len(self.track), text))

    def wait(self, seconds):
        for _ in range(int(round(seconds * FPS))):
            self.emit()

    def move_j(self, p, yaw, speed_deg=75.0):
        """joint space move to a TCP pose, the joint that moves most peaks at ~speed_deg / s"""
        q0, q1 = self.q.copy(), self.arm.ik(np.array(p, dtype=float), tool_down(yaw), self.q)
        seconds = max(0.6, 1.875 * math.degrees(np.abs(q1 - q0).max()) / speed_deg)
        n = int(round(seconds * FPS))
        for i in range(1, n + 1):
            self.q = q0 + (q1 - q0) * min_jerk(i / n)
            self.emit()
        self.yaw = yaw

    def move_l(self, p, yaw=None, speed=18.0, min_seconds=0.35):
        """straight TCP move at ~speed cm / s on average"""
        yaw = self.yaw if yaw is None else yaw
        p0, p1, y0 = self.tcp()[0], np.array(p, dtype=float), self.yaw
        seconds = max(min_seconds, np.linalg.norm(p1 - p0) / speed, abs(yaw - y0) / math.radians(90))
        self.path(lambda s: (p0 + (p1 - p0) * s, y0 + (yaw - y0) * s), seconds)

    def path(self, f, seconds, timing=min_jerk):
        """TCP along f(s) -> (position, yaw) for s in [0, 1]"""
        n = max(1, int(round(seconds * FPS)))
        for i in range(1, n + 1):
            p, self.yaw = f(timing(i / n))
            self.q = self.arm.ik(p, tool_down(self.yaw), self.q)
            self.emit()

    def gripper(self, half_opening, seconds=0.35):
        f0 = self.fingers
        n = int(round(seconds * FPS))
        for i in range(1, n + 1):
            self.fingers = f0 + (half_opening - f0) * min_jerk(i / n)
            self.emit()

    def attach(self, prop):
        p, R = self.tcp()
        prop.grip = (R.T @ prop.R, R.T @ (prop.p - p))

    def detach(self, prop):
        prop.grip = None


# ---------------------------------------------------------------- tasks

WORKBENCH = [
    "# workbench: top, frame and legs",
    "box 0 %.1f 25 170 4 100 0.72 0.73 0.74" % (SURFACE - 2),
    "box 0 %.1f 25 164 6 94 0.30 0.32 0.36" % (SURFACE - 7),
] + ["box %d %.1f %d 6 %.1f 6 0.30 0.32 0.36" % (x, (SURFACE - 4) / 2, z, SURFACE - 4)
     for x in (-79, 79) for z in (-19, 69)]


def wipe_table():
    """clean a coffee spill: zigzag strokes with a sponge, then scrub a stubborn stain"""
    rng = np.random.default_rng(7)
    prog = Program(9.0, home=(0, SURFACE + 38, 32), home_yaw=math.pi / 2)
    up = 10.0
    x0, x1, rows = -30.0, 30.0, [36.0, 43.0, 50.0, 57.0]
    stain = np.array([13.0, SURFACE, 46.0])
    prog.wait(0.8)
    prog.step("Approach the table")
    prog.move_j((x0, SURFACE + up, rows[0]), math.pi / 2)
    prog.move_l((x0, SURFACE, rows[0]), speed=12.0)
    for i, z in enumerate(rows):
        prog.step("Wipe stroke %d / %d" % (i + 1, len(rows)))
        xa, xb = (x0, x1) if i % 2 == 0 else (x1, x0)
        prog.move_l((xb, SURFACE, z), speed=24.0)
        if i + 1 < len(rows):
            prog.move_l((xb, SURFACE, rows[i + 1]), speed=12.0)
    prog.step("Lift and move to the stain")
    prog.move_l((x0, SURFACE + up, rows[-1]), speed=15.0)
    prog.move_l(stain + (4.5, up, 0), speed=25.0)
    prog.move_l(stain + (4.5, 0, 0), speed=12.0)
    prog.step("Scrub the stain")
    radius, turns = 4.5, 4
    prog.path(lambda s: (stain + radius * np.array([math.cos(2 * math.pi * turns * s), 0,
                                                    math.sin(2 * math.pi * turns * s)]), math.pi / 2), 5.0)
    prog.step("Lift and return home")
    prog.move_l(stain + (4.5, up, 0), speed=12.0)
    prog.move_j((0, SURFACE + 38, 32), math.pi / 2)
    prog.wait(1.0)

    scene = ["tool sponge 9", "tcp TCP", "surface %.1f" % SURFACE, "camera 0 80 36 0.62 0.55 175"] + WORKBENCH
    scene += ["# spray bottle and a towel",
              "cylinder -62 %.1f 8 3.2 16 0.15 0.55 0.85" % SURFACE,
              "cylinder -62 %.1f 8 1.2 6 0.92 0.92 0.92" % (SURFACE + 16),
              "box -46 %.1f 6 16 1 11 0.35 0.65 0.55" % (SURFACE + 0.5),
              "# dirt: x y z radius seconds-of-wiping"]
    spots = []
    for _ in range(90):  # coffee spill: a few overlapping blobs
        c = [(-6, 47), (3, 52), (-14, 41), (8, 39)][rng.integers(4)]
        p = np.array(c, dtype=float) + rng.normal(0, 4.0, 2)
        spots.append((p[0], p[1], rng.uniform(1.4, 3.2), 0.05))
    for _ in range(45):  # crumbs over the whole area
        spots.append((rng.uniform(x0, x1), rng.uniform(rows[0] - 2, rows[-1] + 2), rng.uniform(0.5, 1.0), 0.02))
    for _ in range(14):  # dried stain that needs scrubbing
        p = stain[[0, 2]] + rng.normal(0, 1.6, 2)
        spots.append((p[0], p[1], rng.uniform(1.2, 2.2), 1.2))
    scene += ["dirt %.1f %.2f %.1f %.2f %.2f" % (x, SURFACE + 0.04, z, r, s) for x, z, r, s in spots]
    return prog, scene


def pick_place():
    """assemble a stack of four parts: pick from the tray, turn, place on the fixture"""
    tray_y = SURFACE + 1.5
    parts = [  # name, size (x y z), color, pick position, place yaw (deg)
        ("Part1", (8.0, 2.5, 8.0), (0.85, 0.22, 0.2), (-52, 30), 0),
        ("Part2", (6.0, 3.0, 6.0), (0.95, 0.72, 0.15), (-38, 30), 45),
        ("Part3", (4.6, 3.0, 4.6), (0.25, 0.65, 0.35), (-52, 46), 0),
        ("Part4", (3.0, 4.0, 3.0), (0.2, 0.45, 0.85), (-38, 46), 45),
    ]
    fixture = np.array([42.0, tray_y, 40.0])
    prog = Program(14.0, home=(0, SURFACE + 40, 32), home_yaw=0.0, fingers=5.0)
    for name, size, _, (x, z), _ in parts:
        prog.props.append(Prop(name, (x, tray_y + size[1] / 2, z)))
    prog.wait(0.8)
    height = tray_y
    above = 12.0
    for i, (name, size, _, (x, z), yaw_deg) in enumerate(parts):
        prop = prog.props[i]
        pick = prop.p.copy()
        place = np.array([fixture[0], height + size[1] / 2, fixture[2]])
        yaw = math.radians(yaw_deg)
        prog.step("Pick part %d" % (i + 1))
        prog.move_j(pick + (0, above, 0), 0.0)
        prog.gripper(size[0] / 2 + 2.2, 0.25)
        prog.move_l(pick, speed=14.0)
        prog.gripper(size[0] / 2 + 0.6, 0.35)
        prog.attach(prop)
        prog.wait(0.15)
        prog.move_l(pick + (0, above, 0), speed=16.0)
        prog.step("Place part %d" % (i + 1))
        prog.move_j(place + (0, above, 0), yaw)
        prog.move_l(place + (0, 1.0, 0), speed=10.0)
        prog.move_l(place, speed=4.0)
        prog.gripper(size[0] / 2 + 2.2, 0.3)
        prog.detach(prop)
        prog.move_l(place + (0, above, 0), speed=16.0)
        height += size[1]
    prog.step("Return home")
    prog.move_j((0, SURFACE + 40, 32), 0.0)
    prog.gripper(5.0, 0.3)
    prog.wait(1.0)

    scene = ["tool gripper 14", "tcp TCP", "surface %.1f" % SURFACE, "camera 2 86 34 0.42 0.50 185"] + WORKBENCH
    scene += ["# parts tray and assembly fixture",
              "box -45 %.2f 38 26 1.5 34 0.22 0.27 0.36" % (SURFACE + 0.75),
              "box %.1f %.2f %.1f 16 1.5 16 0.55 0.57 0.60" % (fixture[0], SURFACE + 0.75, fixture[2]),
              "cylinder %.1f %.2f %.1f 0.9 1.2 0.35 0.36 0.4" % (fixture[0] - 6, tray_y, fixture[2] - 6),
              "cylinder %.1f %.2f %.1f 0.9 1.2 0.35 0.36 0.4" % (fixture[0] + 6, tray_y, fixture[2] + 6)]
    scene += ["prop %s box %.1f %.1f %.1f %.2f %.2f %.2f" % ((name,) + size + color) for name, size, color, _, _ in parts]
    return prog, scene


def screw_driving():
    """fasten the lid of a housing: drive four screws with an electric screwdriver"""
    lid = SURFACE + 8.6
    holes = [(6.0, 44.0), (30.0, 44.0), (30.0, 60.0), (6.0, 60.0)]
    loose = 1.6
    prog = Program(18.0, home=(10, SURFACE + 42, 30), home_yaw=0.0)
    for i, (x, z) in enumerate(holes):
        prog.props.append(Prop("Screw%d" % (i + 1), (x, lid + loose, z), yaw=37 * i))
    prog.wait(0.8)
    for i, (x, z) in enumerate(holes):
        screw = prog.props[i]
        head = screw.p + (0, 0.7, 0)
        prog.step("Move to screw %d" % (i + 1))
        prog.move_j(head + (0, 8, 0), 0.0)
        prog.move_l(head + (0, 0.6, 0), speed=14.0)
        prog.move_l(head, speed=3.0)
        prog.attach(screw)
        prog.step("Drive screw %d" % (i + 1))
        spindle0, p0 = prog.spindle, prog.tcp()[0]
        n = int(round(1.4 * FPS))
        for k in range(1, n + 1):
            s = k / n
            prog.spindle = spindle0 + 2 * math.pi * 6 * (s * s * (3 - 2 * s))
            prog.q = prog.arm.ik(p0 - (0, loose * min_jerk(s), 0), tool_down(prog.yaw), prog.q)
            prog.emit()
        prog.detach(screw)
        prog.wait(0.2)
        prog.move_l(head + (0, 8 - loose, 0), speed=16.0)
    prog.step("Return home")
    prog.move_j((10, SURFACE + 42, 30), 0.0)
    prog.wait(1.0)

    scene = ["tool screwdriver 18", "tcp TCP", "surface %.1f" % SURFACE, "camera 14 90 40 0.62 0.42 120"] + WORKBENCH
    scene += ["# housing with its lid",
              "box 18 %.1f 52 30 8 22 0.45 0.52 0.62" % (SURFACE + 4),
              "box 18 %.2f 52 28 0.6 20 0.25 0.27 0.30" % (lid - 0.3)]
    scene += ["cylinder %.1f %.2f %.1f 1.6 0.05 0.12 0.12 0.13" % (x, lid, z) for x, z in holes]
    scene += ["# screws feeder", "box -40 %.1f 40 14 6 10 0.42 0.44 0.48" % (SURFACE + 3)]
    for i in range(len(holes)):
        scene += ["prop Screw%d screw 1.3 0.7 2.4 0.86 0.70 0.32" % (i + 1)]
    return prog, scene


# ---------------------------------------------------------------- output

def write_bvh(path, prog):
    gripper = prog.fingers is not None
    lines = ["HIERARCHY", "ROOT Base", "{", "\tOFFSET 0 0 0",
             "\tCHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation"]
    depth = 1
    for name, axis, offset in ARM:
        t = "\t" * depth
        lines += [t + "JOINT " + name, t + "{", t + "\tOFFSET %g %g %g" % offset, t + "\tCHANNELS 1 %srotation" % axis]
        depth += 1
    t = "\t" * depth
    lines += [t + "JOINT Tool", t + "{", t + "\tOFFSET 0 %g 0" % L_FLANGE, t + "\tCHANNELS 1 Yrotation"]
    if gripper:
        for side, x in (("FingerL", -1), ("FingerR", 1)):
            lines += [t + "\tJOINT " + side, t + "\t{", t + "\t\tOFFSET %g %g 0" % (x * prog.fingers, prog.arm.tool - 5),
                      t + "\t\tCHANNELS 3 Xposition Yposition Zposition",
                      t + "\t\tEnd Site", t + "\t\t{", t + "\t\t\tOFFSET 0 5 0", t + "\t\t}", t + "\t}"]
    lines += [t + "\tJOINT TCP", t + "\t{", t + "\t\tOFFSET 0 %g 0" % prog.arm.tool, t + "\t\tCHANNELS 0",
              t + "\t\tEnd Site", t + "\t\t{", t + "\t\t\tOFFSET 0 2 0", t + "\t\t}", t + "\t}", t + "}"]
    for d in range(depth - 1, 0, -1):
        lines.append("\t" * d + "}")
    for pr in prog.props:
        p = pr.track[0][0] - BASE
        lines += ["\tJOINT " + pr.name, "\t{", "\t\tOFFSET %.2f %.2f %.2f" % tuple(p),
                  "\t\tCHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation",
                  "\t\tEnd Site", "\t\t{", "\t\t\tOFFSET 0 1 0", "\t\t}", "\t}"]
    lines += ["}", "MOTION", "Frames: %d" % len(prog.track), "Frame Time: %.6f" % (1.0 / FPS)]

    def num(v):
        s = ("%.3f" % v).rstrip("0").rstrip(".")
        return "0" if s in ("-0", "") else s

    for f, (q, spindle, fingers) in enumerate(prog.track):
        v = [BASE[0], BASE[1], BASE[2], 0, 0, 0] + [math.degrees(a) for a in q] + [math.degrees(spindle)]
        if gripper:
            v += [-fingers, prog.arm.tool - 5, 0, fingers, prog.arm.tool - 5, 0]
        for pr in prog.props:
            p, R = pr.track[f]
            v += list(p - BASE) + list(euler_yxz(R))
        lines.append(" ".join(num(x) for x in v))
    with open(path, "w", newline="\n") as fp:
        fp.write("\n".join(lines) + "\n")


def check(name, prog):
    """report reach / smoothness, the viewer would show any problem as a jump"""
    q = np.array([t[0] for t in prog.track])
    dq = np.degrees(np.abs(np.diff(q, axis=0))).max(axis=0) * FPS
    tcp = np.array([prog.arm.frames(t[0], t[1])["TCP"][0] for t in prog.track])
    elbow = np.array([prog.arm.frames(t[0])["Elbow"][0] for t in prog.track])
    print("%-14s %5d frames (%4.1f s)  max joint speed %s deg/s  lowest TCP %.2f  lowest elbow %.1f" % (
        name, len(q), len(q) / FPS, np.round(dq).astype(int), tcp[:, 1].min(), elbow[:, 1].min()))


def main():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(root, "data", "robot"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name, task in (("Robot_WipeTable", wipe_table), ("Robot_PickPlace", pick_place),
                       ("Robot_ScrewDriving", screw_driving)):
        prog, scene = task()
        write_bvh(os.path.join(args.out, name + ".bvh"), prog)
        with open(os.path.join(args.out, name + ".scene"), "w", newline="\n") as fp:
            fp.write("# %s: %s\n" % (name, task.__doc__.strip()))
            fp.write("\n".join(scene) + "\n")
            fp.write("".join("step %d %s\n" % s for s in prog.steps))
        check(name, prog)


if __name__ == "__main__":
    sys.exit(main())
