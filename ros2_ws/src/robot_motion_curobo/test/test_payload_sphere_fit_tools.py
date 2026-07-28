import numpy as np

from robot_motion_curobo.collision_model_visualizer import Payload
from robot_motion_curobo.payload_sphere_fit_generator import exact_axis_protrusion
from robot_motion_curobo.payload_sphere_fit_visualizer import fitted_spheres_for_payload


def test_exact_axis_protrusion_distinguishes_contained_and_external_spheres():
    contained = exact_axis_protrusion(
        np.array([[0.0, 0.0, 0.0]]), np.array([0.1]), (0.3, 0.4, 0.4)
    )
    assert contained["maximum_m"] == 0.0
    external = exact_axis_protrusion(
        np.array([[0.14, 0.0, 0.0]]), np.array([0.02]), (0.3, 0.4, 0.4)
    )
    assert np.isclose(external["per_axis_m"], [0.01, 0.0, 0.0]).all()


def test_fitted_spheres_are_shifted_into_payload_link_frame():
    payload = Payload(
        side="left",
        box_id=11,
        link_name="left_tool0",
        grasp_mode="top_suction",
        center_xyz=(0.0, 0.0, 0.2),
        size_xyz=(0.3, 0.4, 0.4),
    )
    spheres = fitted_spheres_for_payload(
        payload,
        {"centers": [[0.01, 0.02, -0.03]], "radii": [0.04]},
    )
    assert len(spheres) == 1
    assert np.allclose(spheres[0].center, [0.01, 0.02, 0.17])
    assert spheres[0].radius == 0.04
