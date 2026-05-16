# OMPL Workshop

Tutorial contents:
Avoid ICRA 2026 `ompl_basics`
1) Create a minimal planner
2) Adjust motion validity checker resolution with `si.setStateValidityCheckingResolution(0.005)`
3) Change planner to e.g. PRM
4) Switch to SE(2)/Reeds-Shepp
5) Plan with an optimization objective

---

## Setup for Linux / macOS

**Prerequisites:** Python version between 3.11 to 3.13, pip

**1. Clone this repository

```bash
git clone https://github.com/KavrakiLab/ompl-workshop.git
cd ompl-workshop
```

**2. Create and source a virtual environment 

```bash
python -m venv env
source env/bin/activate
```

**3. Install dependencies

```bash
python -m pip install -r requirements.txt
```

---

## Setup for Windows 11

**Prerequisites:**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) with the WSL 2 backend enabled
- A WSL 2 Linux distribution (e.g. Ubuntu, installable from the Microsoft Store)

**1. Load the image** (run once, inside a WSL 2 terminal):

```bash
docker load -i ompl_tutorial.tar
```

**2. Start the container** from the tutorial directory:

```bash
./run.sh
```

This mounts the current directory into the container, so you can edit files with any editor on your host and run them inside the container.

---

## For testers

### Task 1 - Manipulator planning in MBM environments
Run solution:
```bash
python ompl_manip/plan_manip_solution.py
```
Rerun should show a valid robot trajectory

You can try different environments and start/end goals by changing the `SCENE_PATH` and `REQUEST_PATH` at the top of `ompl_manip/plan_manip_solution.py`

Starter code and patch coming soon
