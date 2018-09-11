# CarND-Controls-MPC
- Self-Driving Car Engineer Nanodegree Program
- James Di Donato
- September 2018
---


[//]: # (Image References)

[image1]: ./photos/Model.png "Model Architecture"
[image2]: ./photos/Cost.png "Cost Function"
[image3]: ./photos/State.png "State Vector"
[image4]: ./photos/Constraints.png "Actuator Constraints"



## Introduction

This is the final project in Term 2 of the Self Driving Car Engineer Nanodegree Program offered by Udacity.  In this project, I implement a Model Predictive Controller (MPC) using C++ to control a simulated self driving vehicle around a  test track. The MPC utilizies a kinematic model of the vehicle to predict the future path of the vehicle at each of the next N time steps. It then adjusts the steering and acceleration to minimize the difference between the prediced location and the reference trajetory. The  model is able to effectively handle latency and steer the vehicle around the track with a reference velocity of 70.


## Rubric Points

**Student describes their model in detail. This includes the state, actuators and update equations.**

A kinematic model was used to model the vehicle trajectory as part of the MPC algorithm. The state variables and model update equations are illustrated in the below images:


![alt text][image3]

![alt text][image1]

![alt text][image4]

The state vector contains all state variable that are calculated for each time step. Going through each,
- (x1,x2) represents the x and y position of the vehicle. 
- (phi) is the angle of the vehicle trajectory 
- (v) is the speed of the vehicle
- (cte) is the cross track error, or the predicted distance of the vehicle from its trajectory
- (epsi), or orientation error, is the angle between the prediced vehicle orientation and the trajectory orientation.

The goal of the controller is to minimize the cte and epsi by actuating the steering wheel and accelerator/brake pedal (combined into one variable for this project).  The actuators are constrained between [-25  25 degrees, while the acceleration / braking is held within [-1,1].

For each of the next N time steps, the future state variables are updated using the model equations. These equations are based on a kinematic model of the vehicle and do not consider dynamic forces on the vehicle, such as gravity, mass, tire fricton, etc. 


**Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.**

In theory, there is a trade-off between the number of time steps and the duration between each, dt. Because N * dt represents the length of the time the model is predicting the vehicle state in the future, you would want N to be lage and dt to be  small. However, the larger the N, the more computationally expensive the MPC would be. In a simiar way, you would want the dt to be small enough so that the model can acuratelly predict the model based on its frequency response. Like most paremeters, the calibration depends on the real world cirumstances, such as hardware capabilities, maximum vehicle speeds, etc.

In this project, I chose a N = 10, and dt=0.1. These seemed to work well with latency = 0 and allowed me to get up to relatively high speeds comfortably. Using a smaller dt, the vehicle oscillates heavily and if N is too large, the MPC lags behind and struggles to compensate due to the added computational latency. Additionally, a benefit of setting the time step to 100ms allowed me to easily compensate for the prescribed latency of 100ms for the project, as you can read about in the last rubric point.

**A polynomial is fitted to waypoints.  If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.**

In the program I do some pre-processing to the waypoints to shift them to the vehicles coordinate system. Doing so makes it significanly easier to compute the third order polynomial that approximates the way-points. I rotate the co-ordinate system 90 degrees, and am able to model the vertical trajectory lines with reasonable coefficients. This is done by using the state variables and basic trig in main.cpp:
```
for (int i = 0; i< ptsx.size(); i++){
  double dx = ptsx[i] - px;
  double dy = ptsy[i] - py;

  ptsx[i] = dx * cos(-psi) - dy * sin(-psi);
  ptsy[i] = dx * sin(-psi) + dy * cos(-psi);
}
```



**The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.**

In completing this project, I started by setting the latency variable to zero and trying to get MPC to work effectively. As stated earlier, I choose N = 10 and dt = 0.1. Once the vehicle was able to navigate the track, I introduced the latency=100ms and quickly realized that the model was not able to compenstate for this. 

Because I had conveniently set  dt = 100ms, my strategy was to delay the actuation of the vehicle by one time step into the future. This would then set the current acuation parameters to the accurate moment in time, post latency. This worked pretty well and the vehicle navigated the track succesfully. In the code, this occurs in MPC.cpp when setting the model constraints:

```
// To Account for the 100ms latency, shift the current commands into the
// future by one time step, or dt = 100ms..
if(t>1){
  delta0 = vars[delta_start +t -2];
  a0 = vars[a_start +t -2];
}
```

# Project Instructions:


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
