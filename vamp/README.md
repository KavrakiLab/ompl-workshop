# OMPL Workshop -- VAMP Demo

In this demonstration, we'll show you how to use sampling-based planners from the Open Motion Planning Library (OMPL) with the hardware-accelerated state validators from Vector-Accelerated Motion Planning (VAMP).

## Installation

Follow the installation instructions at the top level of this repository.

## Instructions

### Planning with a VAMP state space

All of the robots supported by VAMP have real-vector valued configurations, so we can use OMPL's `RealVectorStateSpace` to handle interpolation and sampling of configurations.
To handle all the boilerplate of bounds-checking, we've provided a `VampStateSpace` class for you.
To get started, **run the example planning problem by executing `panda-planning.py`**:

```sh
cd vamp
python3 ./panda-planning.py
```

If everything is correct, OMPL will construct a plan for the Panda robot, then the script will generate a visualization for you.
However, if you look at the visualized trajectory, something will seem a little off -- the robot happily glides through all the obstacles!

![](screenshots/colliding.png)

### Adding support for state validation

Now that we can plan for a robot, we also need to be able to validate the robot's motions.
To start, **implement the stub methods on VampMotionValidator and VampStateValidityChecker**.

```py
# Stubs also available in template file

class VampMotionValidator(ob.MotionValidator):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
        raise NotImplementedError("Implement constructor for VampMotionValidator")

    def checkMotion(
        self, s1: ob.RealVectorStateType, s2: ob.RealVectorStateType
    ) -> bool:
        raise NotImplementedError("Implement motion checker for VampMotionValidator")


class VampStateValidityChecker(ob.StateValidityChecker):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
        raise NotImplementedError("Implement constructor for VampStateValidityChecker")

    def isValid(self, s: ob.RealVectorStateType) -> bool:
        raise NotImplementedError("Implement validator for VampStateValidityChecker")
```

Once you have implemented the validators, we now need to register the validators with the SpaceInformation `si` so that the planner can use them.
To do so, in the `main` function, **construct a `VampMotionValidator` and a `VampStateValidityChecker`, then register them with `si.setMotionValidator()` and `si.setStateValidityChecker` respectively.**

### Check your work

Once you are done, **run `panda-planning.py` again**.
The visualized trajectory should now avoid collision.

![](screenshots/avoiding.png)

### Bonus: play with the environments

If you finish up early, try playing with the environments to make harder motion planning problems.
You can do this by editing `make_environment()` to have new primitives in it.

```python

def make_environment() -> vamp.Environment:
    """
    Construct the collision-checking environment for this problem.
    """
    env = vamp.Environment()

    # ... add your own things here!

    return env
```

You can add primitive objects using methods like `env.add_cuboid` and `env.add_capsule`.
You can also add point clouds with `env.add_pointcloud` with a list of 3D points.

## Planning against the MotionBenchMaker environments

To benchmark against harder problems, we've provided a loader in `mbm.py` that can construct VAMP environments for the MotionBenchMaker dataset.
To use it, all we have to do is replace our call to `make_environment` with `mbm.load_scene`, then load new start and goal configurations with `mbm.load_request`.

```diff
-   env = make_environment()
+   env = mbm.load_scene("problems/bookshelf_tall_panda/scene0001.yaml")
```

```diff
-   [q_start, q_end] = [Q_START, Q_END]
+   q_start, q_end = mbm.load_request("problems/bookshelf_tall_panda/request0001.yaml")
```

You can update the paths in these lines with different problem locations (such as `problems/cage_panda/request0002.yaml` and `problems/cage_panda/scene0002.yaml`) to try new scenes and problems.
