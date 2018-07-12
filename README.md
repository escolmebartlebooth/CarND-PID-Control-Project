# CarND-Controls-PID: Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 15 July 2018

---

## Objectives

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

If a control value (for example steering) can be measured in terms of how far away from the control value is from the optimium value, then a controller can be created which returns the object being controlled to that optimium value.

In PID control, 3 control terms are applicable:
* P(roportional): the control value is reduced in proportion to itself using a constant proportional value (Kp). For example, if the control error (CTE) is 5 units, the proportional controller would adjust the controller by -Kp*CTE = -5Kp.
* I(ntegral)
* D(erivative)

Each term contributes to the overall control...

### Tuning: Manual Approaches

### Tuning: Algorithmic Approaches

### Burning it up

