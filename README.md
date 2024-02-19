# TOGT-Planner

**Time-Optimal Gate-Traversing (TOGT) Planner** is a light-weight racing trajectory generator that can accommondate a broad range of race tracks comprising gates with diverse shapes and sizes. It provides high trajectory quality in terms of dynamically feasibility by employing differential-flat nature of quadrotor systems and incorporating single-rotor thrust constriants.

## Related Papers

If the TOGT planner helps your academic projects, please cite our paper. Thank you!

- **Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing**, Chao Qin, Maxime SJ Michet, Jingxiang Chen, and Hugh H.-T. Liu,  IEEE International Conference on Robotics and Automation (ICRA-2024)

## Updates:

- **Feb. 19, 2024** - First version released

## Install on Ubuntu

#### 1. Clone repo

```
git clone https://github.com/FSC-Lab/TOGT-Planner/
cd TOGT-Planner
```

#### 2. Build the library

```
mkdir build; cd build
cmake ..
make
```

#### 3. Run the example

```
./tests
```

#### 4. Check the planned trajectory

```
cd ../resources/trajectory
python3 plot_traj.py
```



## Acknowledgements

- We use [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite) as the internal solver: An Easy-to-Use Header-Only L-BFGS Solver.
- We use [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) for geometric constraint elimination

