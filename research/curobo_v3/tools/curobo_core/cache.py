from .scene import digest


def resource_key(snapshot, tasks, box_fit, suction_mode="side"):
    key = snapshot.geometry_key, tuple(sorted(tasks.items())), digest(box_fit)
    return key if suction_mode == "side" else (*key, suction_mode)
