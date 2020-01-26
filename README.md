# **Model Predictive Controller** 

### Completed for Udacity Self Driving Car Engineer - 2018/09
---

Solution Video : https://www.youtube.com/watch?v=K9rbYH4H3CI&feature=youtu.be

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