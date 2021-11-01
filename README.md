# F1Tenth Labs
This repo holds lab codes for F1/10 Autonomous Racing Cars course.

Tested on:
- Ubuntu 20.04 LTS
- ROS 1 Noetic
- Python 3.8
- CUDA 11.3

## Build Instructions
1. Make a new Catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
```

1. Clone this repo, [particle filter](https://github.com/shineyruan/particle_filter) and [F1Tenth Simulator](https://github.com/f1tenth/f1tenth_simulator) into the src folder of the new Catkin workspace.

```bash
cd src
git clone https://github.com/shineyruan/F1Tenth_Labs.git
git clone https://github.com/shineyruan/particle_filter.git
git clone https://github.com/f1tenth/f1tenth_simulator.git
cd ..
```

3. Install [RangeLibc](https://github.com/shineyruan/range_libc) for CUDA-accelerated particle filter localization.
4. Build the workspace.

```bash
catkin_make_isolated --install
```

5. Run one lab at a time.

**Note: Please checkout lab6/launch for running lab 6 codes!**

## Acknowledgements
This repo borrows codes/ideas from:
- [F1Tenth education website](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/index.html)
- F1Tenth Simulator
- MIT-Racecar's GPU-based particle filter
