"""Exact tensor stand-ins for pure-Python loops in cuRobo resource builds (MOTION-443).

cuRobo lists the self-collision sphere pairs of every robot model it builds (each collision checker
and IK solver) with a Python double loop over all sphere pairs: ~0.5 s of a ~0.6 s build at the door
robot's sphere count. install() computes the same list, in the same order, with tensor ops."""
import torch


def sphere_pairs(sphere_pair_distances):
    """Sphere pairs (i, j), i < j, of cuRobo's create_from_sphere_pair_distances in its loop order: a
    row whose distances are all -inf is skipped, otherwise every j > i whose distance is not -inf."""
    distances = sphere_pair_distances.cpu()
    kept = (distances != -torch.inf).triu(1)
    kept &= (distances.max(dim=1).values != -torch.inf)[:, None]
    return torch.nonzero(kept).to(torch.int16).contiguous()


def install():
    """Route cuRobo's self-collision pair construction through sphere_pairs (idempotent)."""
    from curobo._src.robot.types.self_collision_params import SelfCollisionKinematicsCfg
    original = SelfCollisionKinematicsCfg.create_from_sphere_pair_distances
    if getattr(original, 'vectorized', False):
        return

    def create_from_sphere_pair_distances(sphere_pair_distances, sphere_padding):
        count = sphere_padding.shape[0] if sphere_padding.ndim == 1 else 0
        if not count or sphere_pair_distances.shape != (count, count):
            return original(sphere_pair_distances, sphere_padding)  # cuRobo's own errors
        return SelfCollisionKinematicsCfg(
            num_spheres=count, sphere_padding=sphere_padding,
            collision_pairs=sphere_pairs(sphere_pair_distances).to(device=sphere_pair_distances.device))

    create_from_sphere_pair_distances.vectorized = True
    SelfCollisionKinematicsCfg.create_from_sphere_pair_distances = staticmethod(create_from_sphere_pair_distances)
