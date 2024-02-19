# TOGT-Planner

**Time-Optimal Gate-Traversing (TOGT) Planner** is a super efficient racing trajectory generator that can accommondate a broad range of race tracks comprising gates with diverse shapes and sizes. It provides high trajectory quality in terms of dynamically feasibility by employing differential-flat nature of quadrotor systems and incorporating single-rotor thrust constriants.

### Updates:

- Feb. 19, 2024 - First version released

### Related Papers

If our repo helps your academic projects, please cite our paper. Thank you!

- Time-Optimal Gate-Traversing Planner for Autonomous Drone Racing, Chao Qin, Maxime SJ Michet, Jingxiang Chen, and Hugh H.-T. Liu,  IEEE International Conference on Robotics and Automation (ICRA-2024)

  





# Drolib

## Introduction

>  A comprehensive libarary for autonomous drones
>  Coded in C++ and support cross-platform compilation and integration.

## How to Install：

### Dependencies
> 1. install Eigen library.

### CMake Build: Unix-like Systems

1. Install all dependencies using your favorite package manager, e.g. 
    ``` bash
    sudo apt-get install g++ cmake git libeigen3-dev -y --quiet --no-install-recommends
    ```

2. Clone `drolib`
    ``` bash
    git clone https://github.com/LonghaoQian/mdss.git
    cd drolib
    ```

3. Configure CMake
    ```
    mkdir build
    cmake -S . -b build
    ```

4. Build!
    ```
    cmake --build build
    ```
---

## Run the Test Files

1. Go the `build` folder
    ``` bash
    ./tests
    ```
    ```
2. Go to the `resources/trajectory` folder and plot the trajectory
    ``` bash
    python3 plot_traj.py
    ```

