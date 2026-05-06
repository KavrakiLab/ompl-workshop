import time
import types
from typing import Any, Sequence

import rerun as rr
import vis
from ompl import base as ob
from ompl import geometric as og

import vamp

SPHERE_CENTERS = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]
SPHERE_RADII = [0.2] * len(SPHERE_CENTERS)
SPHERES = [vamp.Sphere(p, r) for (p, r) in zip(SPHERE_CENTERS, SPHERE_RADII)]

DIMENSION = 7
Q_START = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
Q_END = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]


def main():
    """
    Construct a plan for a Panda in a static environment and visualize the plan.
    """

    # Use VAMP's robot module to initialize state space and validations
    robot = vamp.panda
    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)

    # Select a planner
    planner = og.RRTConnect(si)

    # Make VAMP environment and set validators
    env = make_environment(SPHERES)
    si.setMotionValidator(VampMotionValidator(si=si, env=env, robot=robot))
    si.setStateValidityChecker(VampStateValidityChecker(si=si, env=env, robot=robot))

    # Build SimpleSetup object
    ss = og.SimpleSetup(si)
    ss.setPlanner(planner)

    # Create start and goal states
    start = si.allocState()
    start[:DIMENSION] = Q_START
    goal = si.allocState()
    goal[:DIMENSION] = Q_END
    ss.setStartAndGoalStates(start, goal)

    # Solve
    planning_start = time.time()
    result = ss.solve(5.0)
    planning_time = time.time() - planning_start

    if not result:
        raise TimeoutError(f"No solution found in {planning_time}s")

    print(f"Found trajectory in {planning_time}s")
    path = ss.getSolutionPath()
    print(f"Generated path with {path.getStateCount()} states")

    rec = rr.RecordingStream("ompl-vamp-demo")
    rec.set_time("frame_idx", sequence=0)
    rec.spawn()
    vis.log_environment(rec, "spheres", SPHERES)
    vis.log_traj(rec, path)


class VampMotionValidator(ob.MotionValidator):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType):
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

    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType):
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


def make_environment(spheres: Sequence[vamp.Sphere]) -> vamp.Environment:
    """
    Construct the collision-checking environment for this problem.
    """
    env = vamp.Environment()
    for sphere in spheres:
        env.add_sphere(sphere)
    return env


if __name__ == "__main__":
    main()
