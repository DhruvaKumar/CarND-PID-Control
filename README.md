# CarND-Controls-PID

![alt text](./results/pid.gif)

The project was done as a part of Udacity's self-driving nanodegree program. It implements a PID controller to drive a car around a track. At every time step, [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases) sends the cross track error (distance between the vehicle and the center of the lane), speed and angle to the program. The program sends back actuator inputs: steering angle and speed. The communication is achieved with [uWebSockets](https://github.com/uWebSockets/uWebSockets).


## Dependencies

* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Reflection

*Describe the effect each of the P, I, D components had in your implementation*

The proportional component tries to steer the car towards the center of the lane so that it minimizes the cross track error. The farther the car is from the center of the lane, the higher is the cross track error, higher is the steering angle set by the P component. As the car reaches the center of the lane, the P component alone has a tendency for the car to overshoot. Oscillations might occur depending on P. 

The derivative component tries to restrain the car from moving too quickly by acting on the rate of change of the cross track error. It brings the car gradually to the center of the lane.

The integral component gets rid of steady state error by acting on the summation of the cross track error over time. Care needs to be taken to prevent integral windup.

*Describe how the final hyperparameters were chosen*

The final hyperparameters were chosen by manual tuning. Two PID controllers were used. One was used to control the steering angle based on the cross track error. The second was to control the throttle based on the absolute value of the cross track error and the absolute value of the steering angle. The idea is that if the car deviates far away from the center, increasing cross track error, or if the car makes sudden changes in steering (prominent `Kd`), the throttle controller would cause the car to slow down, allowing the steering angle PID controller to recover. Manual tuning was done by setting all the gains to 0, adding in `Kp` until the car drives for some distance after which it starts oscillating. `Kd` was then added to prevent overshooting. `Kp` and `Kd` were then tweaked to make sure the car would swing back to the center if it deviates (`Kp`), but would do so gracefully (`Kd`). `Ki` wasn't needed since there was no visible bias. 
The steering angle PID was manually tuned to the following gains: `Kp=0.065 Kd=0.6 Ki=0.0`
The throttle PID was manually tuned to the following gains: `Kp=0.05 Kd=0.1 Ki=0`. The throttle values were constrained within the range 0 to 1. Negative values were rounded to 0 since they were causing the car to brake frequently during initial tuning.



[Final video](./results/pidm.mp4)