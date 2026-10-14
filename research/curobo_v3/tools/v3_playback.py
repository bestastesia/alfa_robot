import numpy as np


def playback_frame_interval(result, maximum_joint_speed_deg_s=30.0):
    if maximum_joint_speed_deg_s <= 0:
        raise ValueError('playback speed must be positive')
    angular_indices = [index for index, name in enumerate(result['joint_names'])
                       if name.startswith(('left_joint', 'right_joint'))]
    rows = np.asarray(result['frames'], dtype=float)
    if len(rows) < 2 or not angular_indices:
        return .03
    maximum_step = float(np.max(np.abs(np.diff(rows[:, angular_indices], axis=0))))
    return maximum_step / np.deg2rad(maximum_joint_speed_deg_s) if maximum_step > 0 else .03
