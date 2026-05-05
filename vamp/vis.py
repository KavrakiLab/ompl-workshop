from pathlib import Path
from typing import Sequence

import rerun as rr
from ompl import geometric as og

import vamp


def log_environment(rec: rr.RecordingStream, path: str, spheres: Sequence[vamp.Sphere]):
    """
    Log the collision-checking environment from this demo.
    """
    rec.log(
        path,
        rr.Ellipsoids3D(
            centers=[s.position for s in spheres],
            half_sizes=[[s.r] * 3 for s in spheres],
            colors=[[255, 255, 0]] * len(spheres),
            fill_mode=rr.components.FillMode.Solid,
        ),
    )
    # hack to make coordinate frames make sense
    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame=f"tf#/{path}"),
        static=True,
    )


def log_traj(rec: rr.RecordingStream, traj: og.PathGeometric):

    # `log_file_from_path` automatically uses the built-in URDF data-loader.
    urdf_path = Path(__file__).parent / "panda/panda.urdf"
    rec.log_file_from_path(urdf_path, static=True)
    # Load the URDF tree structure into memory
    urdf_tree = rr.urdf.UrdfTree.from_file_path(urdf_path)

    # The `flush` call is optional, but it helps with logging consistency,
    # because it ensures that the URDF finishes loading before continuing.
    rec.flush()

    traj.interpolate(traj.getStateCount() * 15)
    i = 0
    print(vamp.panda.joint_names())
    for q in traj.getStates():
        for theta, joint_name in zip(q, vamp.panda.joint_names()):
            # Animate joints by logging transforms
            for joint in urdf_tree.joints():
                if joint.name == joint_name:
                    # compute_transform gives you a complete transform that is ready to log,
                    # calculated from joint origin and the current angle and with the frame names set.
                    transform = joint.compute_transform(theta)
                    rec.set_time("frame_idx", sequence=i)
                    rec.log("transforms", transform)
        i += 1

    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame="panda_link0"),
        static=True,
    )
