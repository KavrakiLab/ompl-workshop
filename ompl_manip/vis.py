import os
from datetime import timedelta
import numpy as np
import pinocchio as pin
import coal
import rerun as rr

HERE = os.path.dirname(__file__)
_VISUAL_URDF = os.path.join(HERE, "../ur5/ur5.urdf")
_MESH_DIRS = [os.path.join(HERE, "../ur5")]

# rerun Capsules3D are Y-aligned; scene cylinders are Z-aligned in their local frame.
_R_Z_TO_Y = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])


def _rr_quat(R):
    q = pin.Quaternion(R)
    return rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])


def log_obstacles(rec, obstacles):
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
            cap_quats.append(_rr_quat(R @ _R_Z_TO_Y))
        elif isinstance(geom, coal.Sphere):
            sph_centers.append(t)
            sph_radii.append(geom.radius)

    if box_centers:
        rec.log("obstacles/boxes", rr.Boxes3D(
            centers=np.array(box_centers),
            half_sizes=np.array(box_half_sizes),
            quaternions=box_quats,
        ), static=True)
    if cap_translations:
        rec.log("obstacles/cylinders", rr.Capsules3D(
            translations=np.array(cap_translations),
            lengths=np.array(cap_lengths),
            radii=np.array(cap_radii),
            quaternions=cap_quats,
        ), static=True)
    if sph_centers:
        rec.log("obstacles/spheres", rr.Spheres3D(
            centers=np.array(sph_centers),
            radii=np.array(sph_radii),
        ), static=True)


def log_path(rec, path, kinematics, dimension):
    """Log the robot visual mesh for each waypoint along the path."""
    vis_geom_model = pin.buildGeomFromUrdf(
        kinematics.model, _VISUAL_URDF, pin.GeometryType.VISUAL, _MESH_DIRS
    )
    vis_geom_data = pin.GeometryData(vis_geom_model)
    vis_data = kinematics.model.createData()
    ee_frame_id = kinematics.model.getFrameId("ee_link")

    # Triangle indices are constant — extract once up front
    robot_meshes = []
    for idx, geom_obj in enumerate(vis_geom_model.geometryObjects):
        mesh = geom_obj.geometry
        if not callable(getattr(mesh, "vertices", None)):
            continue
        tris = np.array(
            [[mesh.tri_indices(j)[k] for k in range(3)] for j in range(mesh.num_tris)],
            dtype=np.int32,
        )
        robot_meshes.append((idx, geom_obj.name, np.array(mesh.vertices()), tris))

    print(f"Visual model: {len(robot_meshes)} meshes loaded")

    ee_positions = []

    n = path.getStateCount()
    for i in range(n):
        rec.set_time("time", duration=timedelta(seconds=i / max(n - 1, 1) * 3.0))
        s = path.getState(i)
        q6 = np.array([s[j] for j in range(dimension)])
        q = kinematics.make_config(q6)
        pin.forwardKinematics(kinematics.model, vis_data, q)
        pin.updateFramePlacements(kinematics.model, vis_data)
        pin.updateGeometryPlacements(
            kinematics.model, vis_data, vis_geom_model, vis_geom_data
        )
        ee_pos = vis_data.oMf[ee_frame_id].translation.copy()
        ee_positions.append(ee_pos)

        for idx, name, local_verts, tris in robot_meshes:
            se3 = vis_geom_data.oMg[idx]
            world_verts = (se3.rotation @ local_verts.T + se3.translation[:, np.newaxis]).T
            rec.log(f"robot/{name}", rr.Mesh3D(
                vertex_positions=world_verts,
                triangle_indices=tris,
                albedo_factor=[0.6, 0.6, 0.65, 1.0],
            ))

    rec.log("trajectory", rr.LineStrips3D(
        [np.array(ee_positions)], colors=[255, 200, 0]
    ), static=True)
    rec.flush()
    print("Scrub the 'waypoint' timeline to animate the robot.")
