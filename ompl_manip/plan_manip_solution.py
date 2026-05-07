"""
ompl_manip: Motion planning for a UR5 arm with coal collision checking.

Loads the spherized URDF via pinocchio (kinematic + geometry model),
parses a scene YAML into coal obstacle geometries, and plans a
collision-free trajectory with OMPL's RRTConnect.
"""

import os
import numpy as np
import yaml

import pinocchio as pin
import coal
import rerun as rr

import vis

from ompl import base as ob
from ompl import geometric as og


HERE = os.path.dirname(__file__)
URDF_PATH = os.path.join(HERE, "../ur5/ur5_spherized.urdf")
SCENE_PATH = os.path.join(HERE, "../problems/box_ur5/scene0001.yaml")
REQUEST_PATH = os.path.join(HERE, "../problems/box_ur5/request0001.yaml")

ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
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
# Scene / environment
# ----------------------------

def build_scene_obstacles(scene_yaml_path):
    """Parse a scene YAML and build coal collision geometries.

    Returns:
        list of (coal_geometry, coal_Transform3s) pairs.

    Dimension conventions (scene YAML -> coal):
        box:      dimensions = [x, y, z] full sizes  -> coal.Box takes half-extents
        cylinder: dimensions = [height, radius]      -> coal.Cylinder(radius, half_height)
        sphere:   dimensions = [radius]              -> coal.Sphere(radius)
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
                geom = coal.Box(dims[0] / 2, dims[1] / 2, dims[2] / 2)
            elif ptype == "cylinder":
                geom = coal.Cylinder(dims[1], dims[0] / 2)
            elif ptype == "sphere":
                geom = coal.Sphere(dims[0])
            else:
                continue
            obstacles.append((geom, tf))

    return obstacles


# ----------------------------
# Robot kinematics + collision model
# ----------------------------

class UR5Kinematics:
    """Pinocchio kinematic + geometry model for the UR5 arm."""

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

    def make_config(self, q6):
        """Build a full pinocchio configuration from the 6 arm joint values."""
        q = pin.neutral(self.model)
        for i, jid in enumerate(self.arm_joint_ids):
            q[self.model.joints[jid].idx_q] = q6[i]
        return q

    def get_world_geometries(self, q6):
        """Return world-frame coal geometries for all robot collision objects.

        Runs FK and updates every geometry placement in one pinocchio call.

        Args:
            q6: 6-element arm configuration [shoulder_pan, ..., wrist_3].

        Returns:
            list of (coal_shape, coal_Transform3f) in world coordinates.
        """
        q = self.make_config(q6)
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
# OMPL integration
# ----------------------------

class ManipStateValidityChecker(ob.StateValidityChecker):
    """OMPL state validity checker using coal sphere-vs-primitive queries."""

    def __init__(self, si, kinematics, obstacles):
        super().__init__(si)
        self.kinematics = kinematics
        self.obstacles = obstacles
        self._req = coal.CollisionRequest()

    def isValid(self, state):
        q6 = np.array([state[i] for i in range(DIMENSION)])
        robot_geoms = self.kinematics.get_world_geometries(q6)

        for robot_shape, robot_tf in robot_geoms:
            for obs_geom, obs_tf in self.obstacles:
                res = coal.CollisionResult()
                coal.collide(robot_shape, robot_tf, obs_geom, obs_tf, self._req, res)
                if res.isCollision():
                    return False
        return True


# ----------------------------
# Request loading
# ----------------------------

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
# Planning
# ----------------------------

def main():
    kinematics = UR5Kinematics(URDF_PATH)
    n_geoms = len(kinematics.geom_model.geometryObjects)
    print(f"Robot collision model: {n_geoms} sphere geometries")

    obstacles = build_scene_obstacles(SCENE_PATH)
    print(f"Scene: {len(obstacles)} obstacle geometries")

    start_q, goal_q = load_request(REQUEST_PATH)

    # 6-DOF joint space, all joints bounded to [-pi, pi]
    space = ob.RealVectorStateSpace(DIMENSION)
    bounds = ob.RealVectorBounds(DIMENSION)
    for i in range(DIMENSION):
        bounds.setLow(i, -np.pi)
        bounds.setHigh(i, np.pi)
    space.setBounds(bounds)

    si = ob.SpaceInformation(space)
    checker = ManipStateValidityChecker(si, kinematics, obstacles)
    si.setStateValidityChecker(checker)
    si.setStateValidityCheckingResolution(0.02)
    si.setup()

    start = si.allocState()
    goal = si.allocState()
    for i in range(DIMENSION):
        start[i] = start_q[i]
        goal[i] = goal_q[i]

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(og.RRTConnect(si))

    print(f"Planning from {np.round(start_q, 3)} to {np.round(goal_q, 3)} ...")
    result = ss.solve(30.0)

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
        vis.log_path(rec, path, kinematics, DIMENSION)
    else:
        print("No solution found within time limit")


if __name__ == "__main__":
    main()
