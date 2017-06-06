# CarND-Controls-MPC

Writeup
---


## The Model

Student describes their model in detail. This includes the state, actuators and update equations.

The model is based on the bicycle model from the homework problem. It was updated to have a 3rd order polynomial.

Because we base everything off of the car's coordinate system, we first translate the track into car coordinates. This makes the model simple:
`x, y, psi, v, cte, epsi`

x,y,and psi are set to zero, since the car is always at the origin.
we are given v, and calculate cte (cross track error) and epsi (speed error)

Here are the update functions:

```
  fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
  fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
  fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
  fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```         
          
## Timestep Length and Elapsed Duration (N & dt)

Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

First used the values from the homework:
`
N = 25
dt = 0.05
`

but it went off the road, so I changed the values:
`
N = 10
dt = 0.1
`
Smaller dt's cause the car to jerk back and forth.  Too large a dt and it can't turn quickly. Large N's cause problems.


## Polynomial Fitting and MPC Preprocessing

The waypoints are converted to the vehicle's coordinate system:

```
 double veh_x = (ptsx[i] - px) * cos(-1 * psi) - (ptsy[i] - py) * sin(-1 * psi);
 double veh_y = (ptsx[i] - px) * sin(-1 * psi) + (ptsy[i] - py) * cos(-1 * psi);
```

Then we feed them into the MPC solve function and find an optimal solution. We feed those values into an array so we can plot the MPC predicted path at the next timesteps and see where it will predict us to go. It works well in most parts of the track, around sharp curves the future predictions can get a bit far from where we would like them to be.



## Model Predictive Control with Latency

The model seemed to work fine with a 100 ms latency. It is possible to push the model forward, and assume we have made progress, by the time the next step comes back. This could help if the latency is large. I didn't see any issues.


