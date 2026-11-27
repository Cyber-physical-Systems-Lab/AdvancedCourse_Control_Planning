"""Generate description/shelf.sdf: a shelf with open-top plastic bins.

Run:  python3 make_shelf.py            (writes ../shelf.sdf)
All numbers are in the Gazebo world frame (robot base_link is 0.03 m above world).

How a bin behaves
-----------------
Each bin rests on its shelf plate and is only held by contact and friction (no hidden rail).
It is kept in the x-z plane by three joints (bin can slide along x, move up/down, and tilt
forward/back), so it cannot spin or slide sideways, but tipping is real physics:

  * pulled out until its centre of mass passes the plate's front edge, the front tips down;
  * its back rim then rises and hits the underside of the plate above (only GAP_ABOVE_BIN
    above the bin), so the bin stays wedged at a small angle and does not fall, even after the
    gripper lets go.

Joint ranges are only a safety net in case the wedge slips (bin can never drop out).
Joint states (slide m, tilt rad) are published on /shelf/joint_states.
"""
import math
import os

# ---------------- parameters ----------------
SHELF_FRONT_X = 0.70          # front face of the shelf plates
SHELF_DEPTH = 0.45
SHELF_CENTER_Y = -0.20
SHELF_WIDTH = 0.50
BOTTOM_PLATE_TOP = 0.10       # top surface of the lowest plate
N_LEVELS = 3                  # levels that carry a bin
PLATE_T = 0.02
POST = 0.03

BIN_DEPTH_X = 0.40            # outer size of a bin
BIN_WIDTH_Y = 0.34
BIN_HEIGHT = 0.15
WALL = 0.01                   # wall / bottom thickness
BIN_MASS = 0.8
GAP_ABOVE_BIN = 0.03          # bin rim -> underside of the plate above (small, so a tipping bin wedges)

# how far each bin's front sticks out past the shelf front at start (one value per level).
# Only the target bin (level 0) sticks out; a bin above it at the same x would sit exactly
# where the wrist and gripper must be for a top-down grasp.
OVERHANGS = [0.00, 0.10, 0.0]

MU_BIN = 0.5                  # bin <-> plate friction (plastic on metal)
SLIDE_RANGE = (-0.02, 0.32)   # safety limits: how far a bin can move along -x (m)
LIFT_RANGE = (-0.15, 0.10)    # safety limits: up/down (m)
TILT_RANGE = (-0.6, 0.6)      # safety limits: tilt (rad, about 34 deg)
# ---------------------------------------------

LEVEL_STEP = BIN_HEIGHT + GAP_ABOVE_BIN + PLATE_T
PLATE_TOPS = [BOTTOM_PLATE_TOP + i * LEVEL_STEP for i in range(N_LEVELS)]
ROOF_TOP = BOTTOM_PLATE_TOP + N_LEVELS * LEVEL_STEP

GREY = "0.55 0.57 0.6 1"
BLUE = "0.15 0.35 0.8 1"


def surface(mu):
    return f"""
        <surface>
          <friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
          <contact><ode><kp>100000</kp><kd>10</kd><max_vel>1.0</max_vel><min_depth>0.001</min_depth></ode></contact>
        </surface>"""


def box(name, xyz, size, color, surf=""):
    pose = " ".join(f"{v:.4f}" for v in xyz) + " 0 0 0"
    s = " ".join(f"{v:.4f}" for v in size)
    return f"""
      <collision name="{name}_c">
        <pose>{pose}</pose>
        <geometry><box><size>{s}</size></box></geometry>{surf}
      </collision>
      <visual name="{name}_v">
        <pose>{pose}</pose>
        <geometry><box><size>{s}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>"""


def inertial(mass, ixx, iyy, izz, pose="0 0 0 0 0 0"):
    return f"""
      <inertial><pose>{pose}</pose><mass>{mass}</mass>
        <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>"""


def frame_link():
    x0, x1 = SHELF_FRONT_X, SHELF_FRONT_X + SHELF_DEPTH
    y0, y1 = SHELF_CENTER_Y - SHELF_WIDTH / 2, SHELF_CENTER_Y + SHELF_WIDTH / 2
    cx = (x0 + x1) / 2
    parts = []
    h = ROOF_TOP
    for i, (px, py) in enumerate([(x0 + POST / 2, y0 + POST / 2), (x0 + POST / 2, y1 - POST / 2),
                                  (x1 - POST / 2, y0 + POST / 2), (x1 - POST / 2, y1 - POST / 2)]):
        parts.append(box(f"post{i}", (px, py, h / 2), (POST, POST, h), GREY))
    for i, top in enumerate(PLATE_TOPS + [ROOF_TOP]):
        parts.append(box(f"plate{i}", (cx, SHELF_CENTER_Y, top - PLATE_T / 2),
                         (SHELF_DEPTH, SHELF_WIDTH, PLATE_T), GREY, surface(MU_BIN)))
    return f"""
    <link name="frame">{inertial(20, 2, 2, 2, f"{cx:.3f} {SHELF_CENTER_Y:.3f} 0.4 0 0 0")}{''.join(parts)}
    </link>"""


def helper_link(name, pose):
    # tiny massless-ish link used only to chain the planar joints (no collision, no gravity)
    return f"""
    <link name="{name}">
      <pose>{pose}</pose>
      <gravity>false</gravity>{inertial(0.05, 1e-4, 1e-4, 1e-4)}
    </link>"""


def joint(name, jtype, parent, child, axis, lo, hi, damping=0.0):
    return f"""
    <joint name="{name}" type="{jtype}">
      <parent>{parent}</parent>
      <child>{child}</child>
      <axis>
        <xyz>{axis}</xyz>
        <limit><lower>{lo}</lower><upper>{hi}</upper><effort>1000</effort><velocity>2.0</velocity></limit>
        <dynamics><damping>{damping}</damping></dynamics>
      </axis>
    </joint>"""


def bin_links(i, plate_top):
    # bin link frame = front-bottom-centre of the bin at its start position
    fx = SHELF_FRONT_X - OVERHANGS[i]
    pose = f"{fx:.4f} {SHELF_CENTER_Y:.4f} {plate_top + 0.001:.4f} 0 0 0"
    D, W, H, t = BIN_DEPTH_X, BIN_WIDTH_Y, BIN_HEIGHT, WALL
    sf = surface(MU_BIN)
    parts = [
        box("bottom", (D / 2, 0, t / 2), (D, W, t), BLUE, sf),
        box("front", (t / 2, 0, H / 2), (t, W, H), BLUE, sf),
        box("back", (D - t / 2, 0, H / 2), (t, W, H), BLUE, sf),
        box("left", (D / 2, W / 2 - t / 2, H / 2), (D - 2 * t, t, H), BLUE, sf),
        box("right", (D / 2, -W / 2 + t / 2, H / 2), (D - 2 * t, t, H), BLUE, sf),
    ]
    m = BIN_MASS
    ixx, iyy, izz = m * (W ** 2 + H ** 2) / 12, m * (D ** 2 + H ** 2) / 12, m * (D ** 2 + W ** 2) / 12
    return (helper_link(f"bin_{i}_slider", pose) + helper_link(f"bin_{i}_lifter", pose) + f"""
    <link name="bin_{i}">
      <pose>{pose}</pose>{inertial(m, ixx, iyy, izz, f"{D / 2:.3f} 0 {H / 3:.3f} 0 0 0")}{''.join(parts)}
    </link>"""
            + joint(f"drawer_{i}", "prismatic", "frame", f"bin_{i}_slider", "-1 0 0", *SLIDE_RANGE)
            + joint(f"bin_{i}_lift", "prismatic", f"bin_{i}_slider", f"bin_{i}_lifter", "0 0 1", *LIFT_RANGE)
            + joint(f"bin_{i}_tilt", "revolute", f"bin_{i}_lifter", f"bin_{i}", "0 1 0", *TILT_RANGE, damping=0.05))


def main():
    bins = "".join(bin_links(i, top) for i, top in enumerate(PLATE_TOPS))
    names = "".join(f"\n        <joint_name>{n}_{i}</joint_name>" if n == "drawer" else f"\n        <joint_name>bin_{i}_{n}</joint_name>"
                    for i in range(N_LEVELS) for n in ("drawer", "tilt"))
    sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/make_shelf.py: shelf + open-top bins that slide, tip and wedge (see the script). -->
<sdf version="1.6">
  <model name="shelf">
    <self_collide>true</self_collide>{frame_link()}
    <joint name="fixed_to_world" type="fixed">
      <parent>world</parent>
      <child>frame</child>
    </joint>{bins}
    <!-- /shelf/joint_states: drawer_i = how far bin i moved out (m), bin_i_tilt = tilt (rad) -->
    <plugin name="shelf_joint_states" filename="libgazebo_ros_joint_state_publisher.so">
      <ros><namespace>/shelf</namespace></ros>
      <update_rate>20</update_rate>{names}
    </plugin>
  </model>
</sdf>
"""
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "shelf.sdf")
    with open(out, "w") as f:
        f.write(sdf)
    print(f"wrote {os.path.normpath(out)}")
    print(f"levels (plate tops): {[round(p, 3) for p in PLATE_TOPS]}, roof {ROOF_TOP:.3f}")
    for i, top in enumerate(PLATE_TOPS):
        fx = SHELF_FRONT_X - OVERHANGS[i]
        print(f"bin_{i}: front wall outer face x={fx:.3f}, inner face x={fx + WALL:.3f}, y={SHELF_CENTER_Y:.3f}, "
              f"rim top z={top + BIN_HEIGHT:.3f} (world) = {top + BIN_HEIGHT - 0.03:.3f} (base_link)")
    s0 = BIN_DEPTH_X - OVERHANGS[0]
    s_min = math.sqrt((BIN_HEIGHT + GAP_ABOVE_BIN) ** 2 - BIN_HEIGHT ** 2)
    print(f"bin_0 starts tipping after ~{s0 - BIN_DEPTH_X / 2:.2f} m of pull; "
          f"stays wedged for pulls up to ~{s0 - s_min:.2f} m")


if __name__ == "__main__":
    main()
