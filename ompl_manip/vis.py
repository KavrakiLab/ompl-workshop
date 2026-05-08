from pathlib import Path

import numpy as np
import pinocchio as pin
import coal
import rerun as rr
from ompl import geometric as og

_VISUAL_URDF = Path(__file__).parent / "../panda/panda.urdf"

_ARM_JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]



def _rr_quat(R):
    q = pin.Quaternion(R)
    return rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])


def log_obstacles(rec: rr.RecordingStream, obstacles):
    """Log scene obstacles as static geometry."""
    box_centers, box_half_sizes, box_quats = [], [], []
    cap_translations, cap_lengths, cap_radii, cap_quats = [], [], [], []
    sph_centers, sph_radii = [], []

    for geom, tf in obstacles:
        t = tf.getTranslation()
        R = tf.getRotation()

        if isinstance(geom, coal.Box):
            box_centers.append(t)
            box_half_sizes.append(geom.halfSide)
            box_quats.append(_rr_quat(R))
        elif isinstance(geom, coal.Cylinder):
            cap_translations.append(t)
            cap_lengths.append(geom.halfLength * 2)
            cap_radii.append(geom.radius)
            cap_quats.append(_rr_quat(R))
        elif isinstance(geom, coal.Sphere):
            sph_centers.append(t)
            sph_radii.append(geom.radius)

    if box_centers:
        rec.log("obstacles/boxes", rr.Boxes3D(
            centers=np.array(box_centers),
            half_sizes=np.array(box_half_sizes),
            quaternions=box_quats,
        ), static=True)
        rec.log("transforms", rr.Transform3D(parent_frame="world", child_frame="tf#/obstacles/boxes"), static=True)
    if cap_translations:
        rec.log("obstacles/cylinders", rr.Cylinders3D(
            centers=np.array(cap_translations),
            lengths=np.array(cap_lengths),
            radii=np.array(cap_radii),
            quaternions=cap_quats,
        ), static=True)
        rec.log("transforms", rr.Transform3D(parent_frame="world", child_frame="tf#/obstacles/cylinders"), static=True)
    if sph_centers:
        rec.log("obstacles/spheres", rr.Spheres3D(
            centers=np.array(sph_centers),
            radii=np.array(sph_radii),
        ), static=True)
        rec.log("transforms", rr.Transform3D(parent_frame="world", child_frame="tf#/obstacles/spheres"), static=True)


def log_path(rec: rr.RecordingStream, path: og.PathGeometric, dimension: int):
    rec.log_file_from_path(_VISUAL_URDF, static=True)
    urdf_tree = rr.urdf.UrdfTree.from_file_path(_VISUAL_URDF)
    rec.flush()

    i = 0
    for q in path.getStates():
        for theta, joint_name in zip(q, _ARM_JOINT_NAMES):
            for joint in urdf_tree.joints():
                if joint.name == joint_name:
                    rec.set_time("frame_idx", sequence=i)
                    rec.log("transforms", joint.compute_transform(theta))
        i += 1

    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame="panda_link0"),
        static=True,
    )
