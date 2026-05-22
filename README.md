# OMPL Workshop
This repository hosts the demo code for the OMPL workshop. The workshop contains two demos, a minimal manipulator planning demo in `ompl_manip` and a SIMD accelerated manipulator planning demo in `vamp`.

## System Requirements
- Linux, MacOS, or Windows with a WSL 2 Ubuntu distribution (Linux and MacOS recommended)
- Python version between 3.11 to 3.13, pip
- git
- Eigen

On WSL/Ubuntu (other distribution package managers offer similar packages)
```bash
sudo apt install libeigen3-dev
```
On MacOS
```bash
brew install eigen
```

### Note for WSL users
The workshop uses [rerun](https://rerun.io/) for visualization. rerun is known to have some issues working out of the box on WSL, so it is recommended to test your rerun installation ahead of the tutorial and follow the instructions in [https://rerun.io/docs/getting-started/install-rerun/troubleshooting#running-on-wsl2-ubuntu](https://rerun.io/docs/getting-started/install-rerun/troubleshooting#running-on-wsl2-ubuntu).

## Setup

**1. Clone this repository**

```bash
git clone https://github.com/KavrakiLab/ompl-workshop.git
cd ompl-workshop
```

**2. Create and source a virtual environment**

```bash
python -m venv env
source env/bin/activate
```

**3. Install dependencies**

```bash
python -m pip install -r requirements.txt
```
