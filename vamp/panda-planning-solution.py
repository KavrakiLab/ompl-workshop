import time
import types
from typing import Sequence

import mbm
import rerun as rr
import vis
from ompl import base as ob
from ompl import geometric as og

import vamp

DIMENSION = 7
Q_START = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
Q_END = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]


def main():
    """
    Construct a plan for a Panda in a static environment and visualize the plan.
    """
    # Construct a VAMP environment
    env = make_environment()
    # env = mbm.load_scene("problems/bookshelf_tall_panda/scene0001.yaml")

    # Use VAMP's robot module to initialize state space and validations
    robot = vamp.panda
    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)

    # Select a planner
    planner = og.RRTConnect(si)

    # Set validators
    si.setMotionValidator(VampMotionValidator(si=si, env=env, robot=robot))
    si.setStateValidityChecker(VampStateValidityChecker(si=si, env=env, robot=robot))

    # Build SimpleSetup object
    ss = og.SimpleSetup(si)
    ss.setPlanner(planner)

    [q_start, q_end] = [Q_START, Q_END]
    # q_start, q_end = mbm.load_request("problems/bookshelf_tall_panda/request0001.yaml")
    # Create start and goal states
    start = si.allocState()
    start[:DIMENSION] = q_start
    goal = si.allocState()
    goal[:DIMENSION] = q_end
    ss.setStartAndGoalStates(start, goal)

    # Solve
    planning_start = time.time()
    result = ss.solve(5.0)
    planning_time = time.time() - planning_start

    rec = rr.RecordingStream("ompl-vamp-demo")
    rec.set_time("frame_idx", sequence=0)
    rec.spawn()
    vis.log_environment(rec, "env", env)

    if not result:
        raise TimeoutError(f"No solution found in {planning_time}s")

    print(f"Found trajectory in {planning_time}s")
    path = ss.getSolutionPath()
    print(f"Generated path with {path.getStateCount()} states")

    vis.log_traj(rec, path)


class VampMotionValidator(ob.MotionValidator):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
        super().__init__(si)
        self.env = env
        self.robot = robot

    def checkMotion(
        self, s1: ob.RealVectorStateType, s2: ob.RealVectorStateType
    ) -> bool:
        config1 = s1[: self.robot.dimension()]
        config2 = s2[: self.robot.dimension()]
        return self.robot.validate_motion(config1, config2, self.env)


class VampStateValidityChecker(ob.StateValidityChecker):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
        super().__init__(si)
        self.env = env
        self.robot = robot

    def isValid(self, s: ob.RealVectorStateType) -> bool:
        return self.robot.validate(s[: self.robot.dimension()], self.env)


class VampStateSpace(ob.RealVectorStateSpace):
    """A real-valued state space for a given VAMP robot."""

    robot: types.ModuleType
    dimension: int

    def __init__(self, robot: types.ModuleType):
        super().__init__(robot.dimension())
        self.robot = robot
        self.dimension = robot.dimension()
        bounds = ob.RealVectorBounds(self.dimension)
        upper_bounds = robot.upper_bounds()
        lower_bounds = robot.lower_bounds()

        for i in range(self.dimension):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        self.setBounds(bounds)


def make_environment() -> vamp.Environment:
    """
    Construct the collision-checking environment for this problem.
    """
    SPHERE_CENTERS = [
        [0.55, 0, 0.25],
        [0.35, 0.35, 0.25],
        [0, 0.55, 0.25],
        [-0.55, 0, 0.25],
        [0, -0.55, 0.25],
        [0.35, -0.35, 0.25],
        [0.35, 0.35, 0.8],
        [0, 0.55, 0.8],
        [-0.35, 0.35, 0.8],
        [-0.55, 0, 0.8],
        [0, -0.55, 0.8],
        [0.35, -0.35, 0.8],
    ]
    SPHERE_RADII = [0.2] * len(SPHERE_CENTERS)

    env = vamp.Environment()
    for center, radius in zip(SPHERE_CENTERS, SPHERE_RADII):
        env.add_sphere(vamp.Sphere(center, radius))

    env.add_capsule(
        vamp.Cylinder(
            [-0.35, -0.35, 0.5],  # start point
            [0.0, 0.0, 0.0],  # Euler angles rotation
            0.2,  # radius
            0.8,  # length
        )
    )
    env.add_cuboid(
        vamp.Cuboid(
            [0.3, 0.3, 0.5],  # box midpoint
            [0.0, 0.0, 1.1],  # Euler angles rotation
            [0.1, 0.3, 0.4],  # half-extents
        )
    )
    env.add_cuboid(
        vamp.Cuboid(
            [0.3, 0.3, 0.5],  # box midpoint
            [0.0, 0.0, 1.1],  # Euler angles rotation
            [0.1, 0.3, 0.4],  # half-extents
        )
    )
    return env


if __name__ == "__main__":
    main()
