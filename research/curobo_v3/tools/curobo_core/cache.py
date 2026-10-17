from .scene import digest


def resource_key(snapshot, tasks, box_fit):
    return snapshot.geometry_key, tuple(sorted(tasks.items())), digest(box_fit)
