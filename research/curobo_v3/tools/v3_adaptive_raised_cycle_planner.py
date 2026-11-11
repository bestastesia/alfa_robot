import json

from v3_fine_collision_retry import search_with_fine_retry
from v3_multiroot_search import batched_rrt_connect_multi_goal
from v3_research_assets import ROOT
from v3_single_arm_planner import SingleArmPlanner


class AdaptiveRaisedCyclePlanner(SingleArmPlanner):
    placement_clearance_m = .15
    conveyor_search_edge_resolution = .05

    def __init__(self, args):
        super().__init__(args)
        policy = json.loads((ROOT/'config/adaptive_raised_cycle.json').read_text())
        self.top_transport_tcp_z_m = policy['top_tcp_z_m']
        self.placement_clearance_m = policy['conveyor_box_center_z_m']-self.conveyor_height_m-self.box_fit['dimensions_m'][2]/2
        self.placement_atlas_override = ROOT/policy['top_atlas']
        if not self.placement_atlas_override.is_file():
            raise FileNotFoundError(self.placement_atlas_override)
        self._placement_search = False

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        if self.top_grasp:
            for pose in self.target_poses.values():
                pose['position'][2] = self.top_transport_tcp_z_m
        return result

    def plan_conveyor_target(self, *args, **kwargs):
        previous = self._placement_search
        self._placement_search = True
        try:
            return super().plan_conveyor_target(*args, **kwargs)
        finally:
            self._placement_search = previous

    def search_path(self, start, goals, lower, upper, validity, budget, seed):
        if not self._placement_search:
            return super().search_path(start,goals,lower,upper,validity,budget,seed)
        return search_with_fine_retry(super().search_path,start,goals,lower,upper,validity,budget,seed)

    def run_single_search(self, start, goals, lower, upper, validity, budget, seed, **kwargs):
        if self._placement_search:
            return search_with_fine_retry(batched_rrt_connect_multi_goal,start,goals,lower,upper,validity,budget,seed,**kwargs)
        return batched_rrt_connect_multi_goal(start,goals,lower,upper,validity,budget,seed,**kwargs)

    def full_cycle(self, boxes, progress=lambda message:None):
        result = super().full_cycle(boxes,progress)
        stats = result.get('conveyor_search_stats',result.get('conveyor_place_search_stats',{}))
        result['adaptive_raised_policy'] = dict(top_tcp_z_m=1.2, top_box_center_z_m=1.0,
                                               conveyor_box_center_z_m=1.082, initial_edge_resolution=.05,
                                               retry_edge_resolution=.03, retry_trigger='original-model fine collision only',
                                               shared_placement_budget_s=2., scene_margin_m=self.conveyor_search_margin_m,
                                               fine_collision_retry=stats.get('fine_collision_retry'))
        return result
