# CarND-Controls-PID
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

PID controller project of Self-Driving Car Engineer Nanodegree Program

---

## Writeup

  This repo implements a PID steering controller to keep the car in track in Udacity's simulator

### PID Tuning

The PID implementation follows the base algorithm presented in the lessons. As the controller's oscillation magnitude is directly related to vehicle's velocity, the throttle set-point is made to be corrected by steering angle cross tracking error such that the vehicle decelerates when tracking error goes large.

  The major efforts of the project focus on PID parameters tuning.

  * Kp - proportional gain will be the first parameter to be set who determines controller's dynamic, larger gain leads to more responsive controller behaviour but also creates larger overshoot and oscillation so destabilise the system.

  * Ki - integral gain will help the controller reach the precision by eliminate the steady-state error but in the mean time destabilise as well the system

  * Kd - derivative gain predicts controller's behaviour in the future will contribute to reduce the overshoot so improves the setting time and stability, but large Kd could increase the sensitivity to perturbations.  

The three gain Kp, Ki, Kd are tuned manually assuring the correct behaviour of the vehicle. I first tuned Kp and Kd to get a balanced tradeoff of controller's rapidity (ability to correct error quickly, especially at a sharp turn) and stability (reasonable oscillation level to keep vehicle in the drivable area). I then activated Ki and iteratively tuned all three parameters to find the most robust controller possible.

Here is the [link](https://youtu.be/MDS1-7wqETM) the recorded video showing that the PID controller successfully drove the car two loops in the simulator.

[![](http://img.youtube.com/vi/MDS1-7wqETM/0.jpg)](http://www.youtube.com/watch?v=MDS1-7wqETM "")




### Dependencies
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
