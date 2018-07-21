# CarND-Controls-PID: Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 15 July 2018

---

[//]: # (Image References)

[image1]: PID_ManualTune.PNG "PID Manual Tuning"

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

Note that there are some additional optional parameters which can be passed in any order:

* "-t": passing this as a command line parameter will put the software into tune mode for the steering controller
* "-s": passing this as a command line parameter will turn a throttle controller on
* it is also possible to pass initial values for the coefficients by passing up to 6 double values
  * where the 1st 3 passed will initialise the steering controller and the next 3 passed will initialise the throttle controller (even if -s is not passed).
  * If less than 3 or less than 6 are passed, the missing values will be taken from the defaults.
  * If 'bad' values are passed, they will be converted to 0, so this is not a particularly robust method.
  * examples:
    *./pid 1.0 1.0 for example will set the P and I coefficients for the steering controller to 1.
    * ./pid -t -s -g 1.0 -h -v will put the controller into tune mode and turn on the throttle controller and initialise the steering controller to 0, 1, 0 and the throttle controller to 0 with 2 default values.

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

### How PID control works

PID controllers use a measured error from an optimal setting to change actuation values of the controller so that the error is minimized.

In this project the first controller actuates the steering angle of the simulated car. There are 3 components to PID control:

* P - the proportional part takes the measured error (how far off the optimal trajectory the object is) at each time step and changes the steering angle in negative proportion to that error. The proportion (or gain, Kp) controls how much the proportional element adjusts the control action (steering). Too low and the steering will not adjust sufficiently. Too high and the steering will adjust too much and start to oscillate and/or become unstable. Alone, proportional control will at best oscillate around a central value, which could be offset from zero error.
* I - the integral part adjusts the controller in proportion to the sum of the errors observed over time. This part of the controller has the effect of adjusting for any offset present in the measurement error.
* D - the derivative part introduces an adjustment based on the rate of change of the error from one step to the next. Again, the value of the gain (Kd) affects how large an adjustment is made. This control action has the effect of dampening the proportional controller and the associated oscillations of proportional control

In the graph below, 5 control values are shown:
* P only at 0.105: clearly showing the oscillation
* PD with D at 1.0: This shows the dampening effect of the D control term but the continued offset from zero error
* PID at 0.0002, 0.0004, 0.002: showing the effect of increasing integral control to arrive and an error which stays close to zero over time.

![alt text][image1]

### Tuning: Manual Approaches

The initial tuning is achieved manually. The steering controller is set to 0.0 for the I and D parts. An initial value for P is tried and then either decreased or increased until the car starts to oscillate on the track.

Once the initial P value is found, D is set by experiment to try and dampen the oscillations so the driving experience is smoother.

Finally, the I part is introduced to enable the car to be more central to the road.

By trial and error, initial values of P = 0.105, I = 0.002 and D = 1.0 allowed the car to drive successfully around the track.

### Tuning: Algorithmic Approaches

In this project, Twiddle was used to automatically tune the PID steering control. Another approach would have been to use Stochastic Gradient Descent.

With Twiddle, the manually tuned values are used as starting values for the controller. The simulation is run for a number of steps - about as many as it takes to drive round the track once. A cumulative error is calculated - in this case mean absolute error - to act as an overall error to minimize.

In essence, the twiddle algorithm is also initialised with increment/decrement values for each of the PID control gains. The algroithm runs until a tolerance is reached which is when the sum of the increment/decrement control gains is lower than that tolerance. This indicates that the algorithm has reached a state where only minimal further tuning of the gains is possible.

The pseudo code for the algorithm is:

set initial gains
set a high best error
set initial increment for each gain
run until the tolerance is reached
* for each control gain
  * increment the control gain by the control_increment
  * initialise controller and run the simulation and capture the error
    * if error < best_error
      * best_error = error
      * control increment *= 1.1
    * else
      * control increment -= 2 * control increment
      * initialise the controller and run the simulation and capture the error
      * if error < best_error
        * best_error = error
        * control increment *= 1.1
        else
          * increment the control gain by the control_increment
          * control increment *= 0.9
  * next control gain

Once run, the settings for optimal operation were found to be: P = 0.302769; I = 0.00260938; D = 2.80355.

### Burning it up

Once the steering controller was optimised, a second controller was implemented to allow the car to run at top speed. Again, a manual tuning approach was adopted which resulted in:
  * when running the steering controller with manually tuned values, the manually tuned throttle controller gains were found to be: P = 0.5 and D = 1.0. For the throttle controller, I was left as 0 as the absolute error was used and offset isn't a problem for speed control.
  * when running the steering controller with auto tuned settings, the throttle control settings above resulted in a few crashes at top speed and more oscillation than when running with a fixed throttle of 0.3.

### In conclusion

A PID controller with manual tuning and a fixed throttle setting was able to run smoothly round the test track with reasonable approximation to the optimal line set by the cross-track error.

Although tuning using twiddle was implemented, the resulting settings didn't really seem to result in much better performance.

A second controller for throttle control enabled a much faster speed around the track but with fairly unstable performance with some crashes on some runs and even abrupt breaking and some reversing when combined with the tuned steering controller. If using the initial manual tuning settings for both controllers, less crashing occurs!

