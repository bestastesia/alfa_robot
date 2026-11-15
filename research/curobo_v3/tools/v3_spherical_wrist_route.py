import argparse
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation
import yourdfpy

from v3_suction_cluster_navigation import ClusterCollision


def wrist_geometry(urdf_path):
    model = yourdfpy.URDF.load(urdf_path, load_meshes=False)
    definition = ET.parse(urdf_path).getroot()
    samples = {side: [] for side in ('left', 'right')}
    residuals = {side: [] for side in samples}
    generator = np.random.default_rng(20261022)
    for trial in range(12):
        model.update_cfg({joint.name: float(generator.uniform(-.8, .8)) for joint in model.actuated_joints})
        for side in samples:
            points, directions = [], []
            for index in (5, 6, 7):
                joint = definition.find(f"joint[@name='{side}_joint{index}']")
                transform = model.get_transform(joint.find('child').attrib['link'], model.base_link)
                points.append(transform[:3, 3])
                axis = np.fromstring(joint.find('axis').attrib['xyz'], sep=' ')
                directions.append(transform[:3, :3]@axis)
            directions = np.asarray(directions)
            projection = np.eye(3)[None]-np.einsum('bi,bj->bij', directions, directions)
            center = np.linalg.solve(projection.sum(0), np.einsum('bij,bj->i', projection, points))
            residuals[side].append(float(np.linalg.norm(np.cross(center-points, directions), axis=1).max()))
            tool = model.get_transform(side+'_tool0', model.base_link)
            samples[side].append((np.linalg.inv(tool)@np.r_[center, 1])[:3])
    result = {}
    for side, values in samples.items():
        center = np.mean(values, axis=0)
        variation = float(np.linalg.norm(np.asarray(values)-center, axis=1).max())
        residual = max(residuals[side])
        if max(variation, residual) > 1e-5:
            raise ValueError(f'{side}末三轴不是已验证的共同球腕交点')
        result[side] = dict(center_tool=center.tolist(), radius_m=float(np.linalg.norm(center)),
            max_axis_residual_m=residual, max_center_variation_m=variation, checked_configurations=len(values))
    return dict(urdf=str(urdf_path), urdf_sha256=hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest(), arms=result)


class WristCurve:
    def __init__(self, initial, target, center_tool, lift=0., lateral=0., timing_ratio=1.):
        self.initial, self.target = np.asarray(initial), np.asarray(target)
        self.center_tool = np.asarray(center_tool)
        self.radial_tool = -self.center_tool
        self.rotation = Rotation.from_matrix(self.initial[:3, :3])
        self.vector = (self.rotation.inv()*Rotation.from_matrix(self.target[:3, :3])).as_rotvec()
        self.angle = float(np.linalg.norm(self.vector))
        self.timing_ratio = timing_ratio
        start = self.initial[:3, 3]+self.initial[:3, :3]@self.center_tool
        end = self.target[:3, 3]+self.target[:3, :3]@self.center_tool
        if lift == 0 and lateral == 0:
            knots, values = [0., 1.], np.stack([start, end])
        else:
            direction = end-start
            normal = np.array([-direction[1], direction[0], 0.])
            normal /= max(np.linalg.norm(normal), 1e-12)
            knots = [0., .5, 1.]
            values = np.stack([start, (start+end)/2+[0,0,lift]+lateral*normal, end])
        zeros = np.zeros(3)
        self.position = make_interp_spline(knots, values, k=5, bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
        derivative = self.position.derivative(1)
        self.center_speed_bound = float(np.linalg.norm(derivative.c, axis=1).max())

    def phase(self, parameter):
        parameter = np.asarray(parameter)
        denominator = 1+(self.timing_ratio-1)*parameter
        warped = self.timing_ratio*parameter/denominator
        first = self.timing_ratio/denominator**2
        second = -2*self.timing_ratio*(self.timing_ratio-1)/denominator**3
        third = 6*self.timing_ratio*(self.timing_ratio-1)**2/denominator**4
        phase = 10*warped**3-15*warped**4+6*warped**5
        derivative = 30*warped**2*(1-warped)**2
        acceleration = 60*warped*(1-warped)*(1-2*warped)
        jerk = 60-360*warped+360*warped**2
        return phase, derivative*first, acceleration*first**2+derivative*second, jerk*first**3+3*acceleration*first*second+derivative*third

    def evaluate(self, parameter):
        parameter = np.asarray(parameter).reshape(-1)
        phase, first, second, third = self.phase(parameter)
        rotation = self.rotation*Rotation.from_rotvec(phase[:, None]*self.vector)
        centers = self.position(parameter)
        radial = rotation.apply(np.repeat(self.radial_tool[None], len(parameter), axis=0))
        positions = centers+radial
        world_axis = self.rotation.apply(self.vector)
        omega = first[:, None]*world_axis
        alpha = second[:, None]*world_axis
        velocity = self.position(parameter, 1)+np.cross(omega, radial)
        acceleration = self.position(parameter, 2)+np.cross(alpha, radial)+np.cross(omega, np.cross(omega, radial))
        angular_jerk = third[:, None]*world_axis
        jerk = (self.position(parameter, 3)+np.cross(angular_jerk, radial)
            +2*np.cross(alpha, np.cross(omega, radial))+np.cross(omega, np.cross(alpha, radial))
            +np.cross(omega, np.cross(omega, np.cross(omega, radial))))
        return dict(centers=centers, positions=positions, rotations=rotation, velocity=velocity,
            acceleration=acceleration, jerk=jerk, omega=omega, alpha=alpha, angular_jerk=angular_jerk)

    def serialize(self):
        return dict(knots=self.position.t.tolist(), coefficients=self.position.c.tolist(), degree=self.position.k,
            initial_matrix=self.initial.tolist(), target_matrix=self.target.tolist(), center_tool=self.center_tool.tolist(),
            rotation_vector=self.vector.tolist(), rotation_timing_ratio=self.timing_ratio)


def certify(curve, collision):
    endpoints = curve.evaluate([0., 1.])
    poses = np.column_stack([endpoints['positions'], endpoints['rotations'].as_quat()])
    if np.any(collision.clearance(poses) <= 0):
        return False
    lower, upper = np.array([0.]), np.array([1.])
    lever = float(np.linalg.norm(collision.local+curve.radial_tool, axis=1).max())
    for depth in range(17):
        middle = (lower+upper)/2
        states = curve.evaluate(middle)
        clearance = collision.clearance(np.column_stack([states['positions'], states['rotations'].as_quat()]))
        if np.any(clearance <= 0):
            return False
        phase = curve.phase(middle)[0]
        angular = curve.angle*np.maximum(phase-curve.phase(lower)[0], curve.phase(upper)[0]-phase)
        bound = curve.center_speed_bound*(upper-lower)/2+2*lever*np.sin(np.minimum(angular, np.pi)/2)
        unresolved = clearance <= bound+1e-10
        if not unresolved.any():
            return True
        if depth == 16:
            return False
        lower, upper, middle = lower[unresolved], upper[unresolved], middle[unresolved]
        lower, upper = np.r_[lower, middle], np.r_[middle, upper]
    return False


def dynamics(curve, states, limits):
    parameter = np.linspace(0., 1., len(states['positions']))
    speed = np.linalg.norm(states['velocity'], axis=1)
    tangent = states['velocity']/np.maximum(speed[:,None], 1e-12)
    normal = states['acceleration']-np.sum(states['acceleration']*tangent, axis=1)[:,None]*tangent
    normal[speed < 1e-8] = 0
    raw = dict(center_speed=float(np.linalg.norm(curve.position(parameter,1),axis=1).max()),
        center_acceleration=float(np.linalg.norm(curve.position(parameter,2),axis=1).max()),
        tcp_speed=float(speed.max()), tcp_acceleration=float(np.linalg.norm(states['acceleration'],axis=1).max()),
        tcp_normal=float(np.linalg.norm(normal,axis=1).max()), tcp_jerk=float(np.linalg.norm(states['jerk'],axis=1).max()),
        angular_speed=float(np.rad2deg(np.linalg.norm(states['omega'],axis=1).max())),
        angular_acceleration=float(np.rad2deg(np.linalg.norm(states['alpha'],axis=1).max())),
        angular_jerk=float(np.rad2deg(np.linalg.norm(states['angular_jerk'],axis=1).max())))
    powers = dict(center_speed=1, center_acceleration=2, tcp_speed=1, tcp_acceleration=2,
        tcp_normal=2, tcp_jerk=3, angular_speed=1, angular_acceleration=2, angular_jerk=3)
    duration = max(1., *(float(raw[name]/limits[name])**(1/powers[name]) for name in limits))*1.01
    actual = {name:value/duration**powers[name] for name,value in raw.items()}
    return duration, actual


def plan_wrist(snapshot, objects, tool_geometry, geometry, limits, offset=.08, margin=.005, solid=False):
    started = time.perf_counter()
    excluded = [item['name'] for item in objects if item['name'].startswith('robot_internal/')]
    objects = [item for item in objects if item['name'] not in excluded]
    output = dict(success=False, paths={}, scene_objects=objects, limits=limits, margin_m=margin, offset_m=offset,
        wrist_geometry=geometry, collision_mode='solid_orbit_sphere' if solid else 'actual_suction_cluster',
        scope='External scene only; vehicle body excluded; no whole-arm IK or dynamics/torque verification',
        excluded_vehicle_objects=excluded, diagnostics={})
    for side in ('left', 'right'):
        initial = np.asarray(snapshot['tcp_world'][side])
        target = np.asarray(snapshot['targets'][side]).copy()
        target[:3,3] -= offset*target[:3,2]
        wrist = geometry['arms'][side]
        model = dict(centers_tool=[wrist['center_tool']], radii=[wrist['radius_m']]) if solid else tool_geometry['arms'][side]
        collision = ClusterCollision(model, objects, margin)
        poses = np.array([np.r_[matrix[:3,3], Rotation.from_matrix(matrix[:3,:3]).as_quat()] for matrix in (initial,target)])
        clearances = collision.clearance(poses)
        output['diagnostics'][side] = dict(endpoint_clearance_m=clearances.tolist())
        if np.any(clearances <= 0):
            output['failure'] = f'{side}起/终点碰撞（含余量），净空{(clearances*1000).round(2).tolist()}mm'
            break
        began = time.perf_counter()
        candidates = []
        tested = 0
        for lift, lateral in [(0.,0.),(.15,0.),(.3,0.),(.5,0.),(.3,.15),(.3,-.15),(.5,.3),(.5,-.3)]:
            for ratio in (1., .5, 2.):
                tested += 1
                curve = WristCurve(initial,target,wrist['center_tool'],lift,lateral,ratio)
                states = curve.evaluate(np.linspace(0.,1.,513))
                candidate_poses = np.column_stack([states['positions'],states['rotations'].as_quat()])
                clearance = collision.clearance(candidate_poses)
                if np.any(clearance <= 0):
                    continue
                duration, actual = dynamics(curve,states,limits)
                length = float(np.linalg.norm(np.diff(states['centers'],axis=0),axis=1).sum())
                candidates.append((length+.02*duration,curve,states,duration,actual,float(clearance.min())))
            if candidates:
                break
        accepted = None
        for candidate in sorted(candidates,key=lambda item:item[0]):
            if certify(candidate[1],collision):
                accepted=candidate
                break
        if accepted is None:
            output['failure']=f'{side}连续球组检查未通过，未输出可播放轨迹'
            break
        score,curve,states,duration,actual,clearance = accepted
        output['paths'][side]=dict(positions=states['positions'].tolist(),centers=states['centers'].tolist(),
            quaternions_wxyz=np.roll(states['rotations'].as_quat(),1,axis=1).tolist(),
            times_s=np.linspace(0,duration,len(states['positions'])).tolist(),duration_s=duration,
            tcp_velocity=(states['velocity']/duration).tolist(), tcp_acceleration=(states['acceleration']/duration**2).tolist(),
            angular_velocity=(states['omega']/duration).tolist(), angular_acceleration=(states['alpha']/duration**2).tolist(),
            center_velocity=(curve.position(np.linspace(0,1,513),1)/duration).tolist(),
            actual_metrics=actual, total_rotation_deg=float(np.rad2deg(curve.angle)),
            minimum_sampled_clearance_m=clearance, parameters=curve.serialize(),
            compute_ms=(time.perf_counter()-began)*1000, tested_candidates=tested, c2_position_and_rotation=True,
            endpoint_velocity_zero=True, endpoint_acceleration_zero=True, continuous_collision_certified=True)
    output['success']=len(output['paths'])==2
    output['total_ms']=(time.perf_counter()-started)*1000
    return output


def default_limits(normal=.001):
    return dict(center_speed=.3,center_acceleration=.5,tcp_speed=.5,tcp_acceleration=.5,tcp_normal=normal,
        tcp_jerk=5.,angular_speed=45.,angular_acceleration=90.,angular_jerk=500.)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--solid',action='store_true')
    args=parser.parse_args()
    tool=json.loads(args.geometry.read_text())
    geometry=wrist_geometry(tool['urdf'])
    result=plan_wrist(json.loads(args.snapshot.read_text()),json.loads(args.scene.read_text())['scene_objects'],tool,geometry,default_limits(),solid=args.solid)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False))
    print(json.dumps({key:value for key,value in result.items() if key not in ('paths','scene_objects')},ensure_ascii=False))
    for side,entry in result['paths'].items():
        print(side,{key:entry[key] for key in ('duration_s','compute_ms','tested_candidates','total_rotation_deg','minimum_sampled_clearance_m','actual_metrics')})


if __name__=='__main__':
    main()
