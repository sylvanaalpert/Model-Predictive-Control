# Model Predictive Control Project
By Sylvana Alpert

---
In this project, actuators for the steering angle and acceleration are updated based on the cross-track error and the orientation error, according to the update equations of the vehicle's kinematic model (taking into account the position, velocity and orientation). The model is tested on a car simulator on a track which displays the desired path and the modeled path of the car.

## Model Implementation

The model describes the car's state (position, velocity, orientation, current cross-track error and orientation error) and updates the actuators (steering angle and throttle) to correctly steer the vehicle to the desired path.

Here the steering angle (delta) and the acceleration (a) are chosen at every time step in order to satisfy the kinematic model's update equations and minimize the cost function:

```
x(t+1) = x(t) + v(t) * cos(psi) * dt
y(t+1) = y(t) + v(t) * sin(psi) * dt
psi(t+1) = psi(t) + v(t) / Lf * delta * dt
v(t+1) = v(t) + a * dt
cte(t+1) = cte(t) + v(t) * sin(epsi) * dt
epsi(t+1) = epsi(t) + v(t) / Lf * delta * dt
```
Where x and y describe the position, psi is the orientation, v is the velocity, cte is the cross-track error and epsi is the orientation error.
The steering angle is constrained to [-25 25] deg and the throttle's actuator to [-1 1].

The cost function includes terms that minimize the cte, the epsi, the difference to a reference velocity, the use of the actuators and the derivatives of the actuators (to avoid jerky movements).

The number of time steps modeled here (N) was chosen to be 10 and dt was chosen to be 100ms. This allowed modeling a total of 1sec ahead of time, since a longer period of time would be irrelevant for a reference velocity of 80mph. Using a smaller dt would probably produce more accurate results, but would also yield an unnecessary computationally demanding simulation.

The vehicle's coordinates obtained from the simulator were transformed to the local coordinate system by applying an affine transform (translation and rotation), effectively making the orientation angle psi zero.

In addition, the vehicle state was used to predict its future state (100ms ahead of time), using the same kinematic equations defined by the MPC model. The actuators delay was simulated by adding a 100ms pause to the code, which corresponds exactly to one time sample. The predicted state was passed to the MPC object as its initial state, counteracting the effect of the latency on the controller. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
