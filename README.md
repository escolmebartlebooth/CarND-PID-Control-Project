# CarND-Controls-PID: Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 15 July 2018

---

[//]: # (Image References)

[image1]: PID_ManualTune.PNG "PID Manual Tuning"
[image2]: RMSE.PNG "RMSE Values"

## Objectives

To implement and tune PID controllers for steering and throttle so that a car can safely drive round the simulator test track without leaving the road surface at as fast a speed as possible.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Note that there are 2 additional optional parameters:

* "-t": passing this as a command line parameter will put the software into tune mode
* "-s": passing this as a command line parameter will turn a throttle controller on
* it is also possible to pass initial values for the coefficients by passing up to 6 double values: ./pid 1.0 1.0 for example will set the P and I coefficients for the steering controller to 1.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Software Structure

The software is located in /src:

* main.cpp: In this we initialise the PID Controller(s) and pass them into the message handler. In the handler, 3 main flows exist:
  * if requested to tune, a twiddle algorithm runs (see later) - this only tunes the steering controller
  * the steer value is updated according to PID control settings after the PID controller is passed the current cross-track error
  * if throttle control is switched on, the throttle value is updated according to its own control value
* PID.cpp / h: In this we have the PID controller:
  * Init: Takes initial values and sets PID control values and initial errors
  * UpdateError: Takes the current cross-track error and updates the control terms for Proportional, Integral and Derivative control
  * TotalError: Outputs the accumulated control error across the 3 control errors and in effect returns the adjustment for steering (or throttle) - the value is limited at + or - 1.

## Discussion

### How PID works

There are 3 components to PID control:

* P - the proportional part takes the measured cross-track error (how far off the optimal trajectory the object is) and changes the steering angle in inverse proportion to that error. The proportion (or gain, Kp) controls how much the proportional element adjusts the control action (steering). Too low and the steering will not adjust sufficiently. Too high and the steering will adjust too much and start to oscillate and/or become unstable
* I - the integral part adjusts the controller in proportion to the sum of the errors observed over time.
* D - the derivative part introduces an adjustment based on the rate of change of the error from one step to the next. Again, the value of the gain (Kd) affects how large an adjustment is made. This control action has the effect of dampening the

In the graph below, 5 control values are shown:
* P only at 0.105: clearly showing the oscillation
* PD with D and 1.0: This shows the dampening effect of the D control term but the continued offset from zero error
* PID at 0.0002, 0.0004, 0.002: showing the effect of increasing integral control to arrive and an error which stays close to zero over time.
![alt text][image1]


### Tuning: Manual Approaches

The initial tuning is achieved manually. The steering controller is set to 0.0 for the I and D parts. An initial value for P is tried and then either decreased or increased until the car starts to oscillate on the track.

Once the initial P value is found, D is set by experiment to try and dampen the oscillations so the driving experience is smoother.

Finally, the I part is introduced to enable the car to be more central to the road.

By trial and error, initial values of P = 0.105, I = 0.002 and D = 1.0 allowed the car to drive successfully around the track.

### Tuning: Algorithmic Approaches

### Burning it up

The same approach was tried with the throttle controller resulting in values of P = 0.5 and D = 1.0. For the throttle controller, I was left as 0.
