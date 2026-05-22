"""
ompl_manip: Motion planning for a Panda arm with coal collision checking.

Loads the spherized URDF via pinocchio (kinematic + geometry model),
parses a scene YAML into coal obstacle geometries, and plans a
collision-free trajectory with OMPL's RRTConnect.
"""

import os
from functools import partial
import numpy as np
import yaml

import pinocchio as pin
import coal
import rerun as rr

import vis

from ompl import base as ob
from ompl import geometric as og


HERE = os.path.dirname(__file__)
URDF_PATH = os.path.join(HERE, "../panda/panda_spherized.urdf")
SCENE_PATH = os.path.join(HERE, "../problems/box_panda/scene0001.yaml")
REQUEST_PATH = os.path.join(HERE, "../problems/box_panda/request0001.yaml")

ARM_JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]
DIMENSION = len(ARM_JOINT_NAMES)


# ----------------------------
# Utilities
# ----------------------------
def quat_to_matrix(q):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


# ----------------------------
# Load MBM scene and request
# ----------------------------
def load_scene(scene_yaml_path):
    """Parse a scene YAML and build coal collision geometries.

    Returns:
        list of (coal_geometry, coal_Transform3s) pairs.
    """
    with open(scene_yaml_path) as f:
        scene = yaml.safe_load(f)

    obstacles = []
    for obj in scene.get("world", {}).get("collision_objects", []):
        for prim, pose in zip(obj["primitives"], obj["primitive_poses"]):
            t = np.array(pose["position"])
            R = quat_to_matrix(pose["orientation"])  # quaternion [x, y, z, w]
            tf = coal.Transform3s(R, t)

            ptype = prim["type"]
            dims = prim["dimensions"]
            if ptype == "box":
                geom = coal.Box(dims[0], dims[1], dims[2])
            elif ptype == "cylinder":
                geom = coal.Cylinder(dims[1], dims[0])
            elif ptype == "sphere":
                geom = coal.Sphere(dims[0])
            else:
                continue
            obstacles.append((geom, tf))

    return obstacles


def load_request(request_yaml_path):
    """Return (start_q, goal_q) arm configs from a request YAML."""
    with open(request_yaml_path) as f:
        req = yaml.safe_load(f)

    start_map = dict(zip(
        req["start_state"]["joint_state"]["name"],
        req["start_state"]["joint_state"]["position"],
    ))
    goal_map = {
        jc["joint_name"]: jc["position"]
        for jc in req["goal_constraints"][0]["joint_constraints"]
    }

    start = np.array([start_map[n] for n in ARM_JOINT_NAMES])
    goal = np.array([goal_map[n] for n in ARM_JOINT_NAMES])
    return start, goal


# ----------------------------
# Robot kinematics + collision model
# ----------------------------
class PandaKinematics:
    """Pinocchio kinematic + geometry model for the Panda arm."""

    def __init__(self, urdf_path):
        # Kinematic model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Collision geometry model — gives us coal shapes and their placements
        self.geom_model = pin.buildGeomFromUrdf(
            self.model, urdf_path, pin.GeometryType.COLLISION
        )
        self.geom_data = pin.GeometryData(self.geom_model)

        # Joint indices for the 6 arm DOFs
        self.arm_joint_ids = [
            self.model.getJointId(name) for name in ARM_JOINT_NAMES
        ]

        # Geometry index pairs to check for self-collision (non-adjacent links)
        self.self_collision_pairs = self._build_self_collision_pairs()

    def _build_self_collision_pairs(self):
        """Return (i, j) geometry index pairs eligible for self-collision checks.

        Skips pairs on the same joint or on parent-child adjacent joints, which
        would collide by construction in a spherized model.
        """
        n = len(self.geom_model.geometryObjects)
        pairs = []
        for i in range(n):
            ji = self.geom_model.geometryObjects[i].parentJoint
            for j in range(i + 1, n):
                jj = self.geom_model.geometryObjects[j].parentJoint
                if ji == jj:
                    continue
                if self.model.parents[ji] == jj or self.model.parents[jj] == ji:
                    continue
                pairs.append((i, j))
        return pairs

    def make_config(self, q7):
        """Build a full pinocchio configuration from the 7 arm joint values."""
        q = pin.neutral(self.model)
        for i, jid in enumerate(self.arm_joint_ids):
            q[self.model.joints[jid].idx_q] = q7[i]
        return q

    def get_world_geometries(self, q7):
        """Return world-frame coal geometries for all robot collision objects.

        Runs FK and updates every geometry placement in one pinocchio call.

        Args:
            q7: 7-element arm configuration [panda_joint1, ..., panda_joint7].

        Returns:
            list of (coal_shape, coal_Transform3f) in world coordinates.
        """
        q = self.make_config(q7)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(
            self.model, self.data, self.geom_model, self.geom_data
        )

        world_geoms = []
        for i, geom_obj in enumerate(self.geom_model.geometryObjects):
            shape = geom_obj.geometry             # coal.Sphere / coal.Box / ...
            se3 = self.geom_data.oMg[i]           # pin.SE3: world transform
            tf = coal.Transform3s(se3.rotation, se3.translation)
            world_geoms.append((shape, tf))

        return world_geoms


# ----------------------------
# collision checker
# ----------------------------
def robot_collides(kinematics, obstacles, state):
    req = coal.CollisionRequest()

    # Forward kinematics
    q7 = np.array([state[i] for i in range(DIMENSION)])
    robot_geoms = kinematics.get_world_geometries(q7)

    # Robot - env collision
    for robot_shape, robot_tf in robot_geoms:
        for obs_geom, obs_tf in obstacles:
            res = coal.CollisionResult()
            coal.collide(robot_shape, robot_tf, obs_geom, obs_tf, req, res)
            if res.isCollision():
                return False
    
    # Robot self collision
    for i, j in kinematics.self_collision_pairs:
        shape_i, tf_i = robot_geoms[i]
        shape_j, tf_j = robot_geoms[j]
        res = coal.CollisionResult()
        coal.collide(shape_i, tf_i, shape_j, tf_j, req, res)
        if res.isCollision():
            return False

    return True


# ----------------------------
# Planning
# ----------------------------
def main():
    kinematics = PandaKinematics(URDF_PATH)
    n_geoms = len(kinematics.geom_model.geometryObjects)
    print(f"Robot collision model: {n_geoms} sphere geometries")

    obstacles = load_scene(SCENE_PATH)
    print(f"Scene: {len(obstacles)} obstacle geometries")

    start_q, goal_q = load_request(REQUEST_PATH)

    """
    TODO list for creating and invoking an OMPL planner
    """
    # 1. Create state space
    # 2. Create SpaceInformation and state validator
    # 3. Create Planner and SimpleSetup
    # 4. Run the planner

    if result:
        path = ss.getSolutionPath()
        path.interpolate(60)
        print(f"Found path with {path.getStateCount()} waypoints after interpolation:")
        for i in range(path.getStateCount()):
            s = path.getState(i)
            q = [round(s[j], 3) for j in range(DIMENSION)]
            print(f"  {q}")

        rec = rr.RecordingStream("ompl-manip-demo")
        rec.set_time("frame_idx", sequence=0)
        rec.spawn()
        vis.log_obstacles(rec, obstacles)
        vis.log_path(rec, path, DIMENSION)
    else:
        print("No solution found within time limit")


if __name__ == "__main__":
    main()
