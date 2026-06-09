"""
MuJoCo Robot Arm — Square Drawing with Trail
=============================================
Cylinders span from entry joint to next body origin.
Pink spheres leave a persistent trail of the EE path.
"""

import math, time, tempfile, os
import numpy as np
import mujoco
import mujoco.viewer
from ik_solver import IKSolver


def build_model_xml():
    deg90 = math.pi / 2

    # DH rows: (a, α, d)
    dh = [
        (0.0, 0,       0.114),
        (0.0, deg90,    0.0),
        (0.4, 0,        0.0),
        (0.3, 0,        0.1654),
        (0.0, deg90,    0.136),
        (0.0, -deg90,   0.2335),
    ]

    # Compute cylinder fromto for each body.
    # Cylinder in body i: from entry joint to next body origin.
    #   from = (0, 0, -d_i)                         — joint i center in body frame
    #   to   = (a_i, -d_{i+1}·sin(α_i), d_{i+1}·cos(α_i))  — body i+1 origin at θ=0
    # For last body: to = (0, 0, 0) (tool tip at origin)
    links = []
    for i in range(6):
        a_i, alpha_i, d_i = dh[i]
        fx, fy, fz = 0, 0, -d_i                     # entry joint
        if i < 5:
            a_next, alpha_next, d_next = dh[i+1]
            tx = a_i
            ty = -d_next * math.sin(alpha_i)
            tz = d_next * math.cos(alpha_i)
        else:
            tx, ty, tz = 0, 0, 0                     # last body: to origin
        # UR10-style: short shoulder joint → sphere, others → cylinder
        length = math.hypot(tx-fx, math.hypot(ty-fy, tz-fz))
        if length < 0.01:
            geom_type = "sphere"
            geom_size = 0.04
        else:
            geom_type = "cylinder"
            geom_size = 0.03 if i >= 3 else 0.04     # wrist links thinner

        colors = [
            "0.3 0.3 0.35 1",   # base (not used here)
            "0.85 0.25 0.25 1",  # link1: red
            "0.25 0.85 0.3 1",   # link2: green
            "0.25 0.45 0.85 1",  # link3: blue
            "0.85 0.75 0.1 1",   # link4: yellow
            "0.85 0.4 0.25 1",   # link5: orange
            "0.6 0.2 0.85 1",    # link6: purple
        ]

        links.append((f"link{i+1}", a_i, alpha_i, d_i,
                      fx, fy, fz, tx, ty, tz, geom_type, geom_size, colors[i+1]))

    xml = []
    xml.append('<?xml version="1.0" encoding="utf-8"?>')
    xml.append('<mujoco model="robot_arm">')
    xml.append('  <compiler angle="radian"/>')
    xml.append('  <option timestep="0.005" gravity="0 0 -9.81"/>')
    xml.append('  <visual><global offwidth="1200" offheight="800"/></visual>')
    xml.append('  <asset>')
    xml.append('    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0.1 0.2 0.3" width="512" height="512"/>')
    xml.append('    <texture type="2d" name="groundplane" builtin="checker" mark="edge"')
    xml.append('             rgb1="0.3 0.35 0.4" rgb2="0.15 0.2 0.25" markrgb="0.7 0.7 0.7" width="300" height="300"/>')
    xml.append('    <material name="groundplane" texture="groundplane" texuniform="true" reflectance="0.2"/>')
    for name, *_, rgba in links:
        xml.append(f'    <material name="{name}_mat" rgba="{rgba}"/>')
    xml.append('  </asset>')
    xml.append('')
    xml.append('  <worldbody>')
    xml.append('    <light directional="true" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" pos="2 2 4" dir="-1 -1 -2"/>')
    xml.append('    <light directional="true" diffuse="0.3 0.3 0.3" specular="0.1 0.1 0.1" pos="-2 1 2" dir="1 -0.5 -1"/>')
    xml.append('    <geom type="plane" size="3 3 0.01" material="groundplane"/>')
    xml.append('')
    # Base — flat cylinder like UR10 base plate
    xml.append('    <body name="base" pos="0 0 0">')
    xml.append('      <geom type="cylinder" size="0.08 0.03" pos="0 0 0.03" rgba="0.25 0.25 0.3 1"/>')

    for i, (name, a, alpha, d, fx, fy, fz, tx, ty, tz, gtype, gsize, _) in enumerate(links):
        px, py, pz = a, -d*math.sin(alpha), d*math.cos(alpha)
        qw, qx = math.cos(alpha/2), math.sin(alpha/2)
        indent = "      " + "  "*i
        xml.append(f'{indent}<body name="{name}" pos="{px:.6f} {py:.6f} {pz:.6f}" quat="{qw:.6f} {qx:.6f} 0 0">')
        xml.append(f'{indent}  <joint name="j{i+1}" type="hinge" axis="0 0 1" pos="0 0 {-d:.6f}" range="-3.1416 3.1416" damping="2"/>')
        if gtype == "cylinder":
            xml.append(f'{indent}  <geom type="cylinder" fromto="{fx:.4f} {fy:.4f} {fz:.4f} {tx:.4f} {ty:.4f} {tz:.4f}" size="{gsize:.4f}" material="{name}_mat" mass="0.3"/>')
        else:
            xml.append(f'{indent}  <geom type="sphere" size="{gsize:.4f}" pos="{fx:.4f} {fy:.4f} {fz:.4f}" material="{name}_mat" mass="0.2"/>')
    # EE red sphere
    xml.append('        <geom type="sphere" size="0.02" rgba="1 0.1 0.1 0.9"/>')
    xml.append('        <site name="ee_site" pos="0 0 0" size="0.005"/>')

    for i in range(len(links)-1, -1, -1):
        xml.append("      " + "  "*i + "</body>")
    xml.append('    </body>')

    # Trail spheres (mocap bodies, initially hidden below ground)
    N_TRAIL = 300
    for k in range(N_TRAIL):
        xml.append(f'    <body name="trail{k}" pos="0 0 -1" mocap="true">')
        xml.append(f'      <geom type="sphere" size="0.003" rgba="1 0.15 0.35 0.6"/>')
        xml.append(f'    </body>')

    xml.append('  </worldbody>')
    xml.append('</mujoco>')
    return '\n'.join(xml), len(links)


def square_trajectory(center=(0.40, 0.0, 0.30), side=0.12, n_pts=200):
    """Square in XZ plane, centered at `center`, side length `side`."""
    cx, cy, cz = center
    h = side / 2  # half side
    corners = [
        (cx - h, cy, cz - h),  # bottom-left
        (cx + h, cy, cz - h),  # bottom-right
        (cx + h, cy, cz + h),  # top-right
        (cx - h, cy, cz + h),  # top-left
        (cx - h, cy, cz - h),  # back to start
    ]
    pts = []
    seg_pts = n_pts // 4
    for c in range(4):
        x0, y0, z0 = corners[c]
        x1, y1, z1 = corners[c+1]
        for j in range(seg_pts):
            t = j / seg_pts
            pts.append((
                x0 + (x1-x0)*t,
                y0 + (y1-y0)*t,
                z0 + (z1-z0)*t,
            ))
    return pts


def run():
    ik = IKSolver()
    model_xml, n_links = build_model_xml()

    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False, encoding='utf-8') as f:
        f.write(model_xml)
        xml_path = f.name

    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    finally:
        os.unlink(xml_path)

    # Pre-compute IK for heart
    square_pts = square_trajectory()
    print("Pre-computing IK for square trajectory...")
    square_q = []
    for px, py, pz in square_pts:
        best_sol = None
        for pitch in [90, 100, 110]:
            for yaw in [-10, 0, 10]:
                r = math.radians(0)
                cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
                cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
                cr, sr = 1.0, 0.0
                R = np.array([
                    [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                    [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                    [-sp,   cp*sr,          cp*cr],
                ])
                T = np.eye(4); T[:3,:3] = R; T[:3,3] = [px, py, pz]
                sols = ik.solve(T)
                if sols:
                    prev = square_q[-1] if square_q else np.zeros(6)
                    cand = min(sols, key=lambda s: np.sum(np.abs(s-prev)))
                    if best_sol is None or np.sum(np.abs(cand-prev)) < np.sum(np.abs(best_sol-prev)):
                        best_sol = cand
                if best_sol is not None:
                    break
            if best_sol is not None:
                break
        if best_sol is not None:
            square_q.append(best_sol)
        elif square_q:
            square_q.append(square_q[-1])
    print(f"  {len(square_q)}/{len(square_pts)} waypoints ready")

    if not square_q:
        print("No valid IK solutions!")
        return

    # Init robot at first square pose
    data.qpos[:] = square_q[0]
    mujoco.mj_forward(model, data)

    square_idx = 0
    step = 0
    trail = []          # list of (x,y,z) EE positions in world frame
    MAX_TRAIL = 300

    print("\n" + "="*50)
    print("  Robot Arm — Square Drawing with Trail")
    print("  ESC to quit")
    print("="*50)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        if viewer is None:
            print("ERROR: Could not launch viewer.")
            return

        viewer.cam.lookat = [0.35, 0.0, 0.25]
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 155
        viewer.cam.elevation = -18

        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link6')

        while viewer.is_running():
            step += 1

            # Advance heart trajectory
            square_idx = (square_idx + 2) % len(square_q)
            q_target = square_q[square_idx]

            # Smooth blend
            q_cur = data.qpos.copy()
            blend = 0.12
            data.qpos[:] = blend * q_target + (1 - blend) * q_cur

            mujoco.mj_forward(model, data)

            # Record trail
            if step % 4 == 0:
                ee_pos = data.xpos[ee_id].copy()
                trail.append(ee_pos)
                if len(trail) > MAX_TRAIL:
                    trail.pop(0)

            # Update trail mocap bodies
            for k in range(min(len(trail), MAX_TRAIL)):
                tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'trail{k}')
                if tid >= 1:
                    data.mocap_pos[tid - 1] = trail[-(k+1)]

            viewer.sync()
            time.sleep(0.002)


if __name__ == "__main__":
    run()
