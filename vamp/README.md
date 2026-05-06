# OMPL Workshop -- VAMP Demo

## Installation

TODO Figure out the best way to install these packages:

- `rerun-sdk`
- `ompl`
- `vamp-planner`

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
To do so, in the `main` function, **construct a VAMP environment using the provided make_environment method. Next, construct a `VampMotionValidator` and a `VampStateValidityChecker`, then register them with `si.setMotionValidator()` and `si.setStateValidityChecker` respectively.**

### Check your work

Once you are done, **run `panda-planning.py` again**.
The visualized trajectory should now avoid collision.

![](screenshots/avoiding.png)
