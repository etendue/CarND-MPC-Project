
---

# MPC Controller


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
## Reflection

### The Model
The model contains vehicle `states` with following variables:
* x: coordinate
* y: coordinate
* psi: heading angle
* v: speed
* cte: cross to center error
* epsi: heading angle error

and 2 `actuators`:
* delta: steering 
* a: throttle 

The update equation is:
```
  x_t+1 = x_t + v_t *cos(psi_t) * dt
  y_t+1 = y_t + v_t *sin(psi_t) * dt
  psi_t+1 = psi_t + v_t/Lf *delta_t *dt
  v_t+1 = v_t + a_t * dt
  cte_t+1  = y_t+1_des - y_t+1;
  epsi_t+1 = psi_t+1 - psi_t+1_des
```
`y_t+1_des` is desired y value calculated by a optimizer as well as `psi_t+1_des` is desired
heading angle.

### Timestep Length and Elapsed Duration (N and dt)

Timestep Length `N` is chosen that CTE is reached to ~ 0 within N steps;
 while `dt` is selected based on how quick vehicle reacts to actuators.
 In simulator vehicle reacts in c.a. 150 milliseconds. So `dt` is chosen 0.15s. 

Value smaller than 0.15s causes bigger cost and CTE does not converge, but sometimes works also. I tried 0.1 previously.



### Polynomial Fitting and MPC Preprocessing
The waypoints are used to polyfit the desired trajectory with 3rd polynomial fitting.
In order to render the waypoints and polyfitted trajectory in simulator. All the points 
are transformated from global coordination system to vehicle coordination system.
This involves coordinate translation and rotation.


### Model Predictive Control with Latency

With latency the actuator values can not be translated into vehicle immediately,
the approach to coping with that is: model predicts the vehicle states after latency with current
states and actuators values. And then use the predicted states to calcuate the actuator values.
The prediction process used also the above state update equations. The idea is similar with prediction
process in kalman filter, particle filter, etc. Prediction introduces uncertainty. So if latency is 
 too big, the model may not predict accurately.
 