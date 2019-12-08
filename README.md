# Robot Controller Module (GMock)
[![Build Status](https://travis-ci.org/Prasheel24/Robot_Controller_Module.svg?branch=GmockForCredit)](https://travis-ci.org/Prasheel24/Robot_Controller_Module)
[![Coverage Status](https://coveralls.io/repos/github/Prasheel24/Robot_Controller_Module/badge.svg?branch=GmockForCredit)](https://coveralls.io/github/Prasheel24/Robot_Controller_Module?branch=GmockForCredit)
[![License MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://github.com/Prasheel24/Robot_Controller_Module/blob/GmockForCredit/LICENSE)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](https://github.com/Prasheel24/tree/GmockForCredit/docs)
---

## Authors
For AIP Development :
* **Raj Prakash Shinde** - *Sprint 1- Driver & Sprint 2- Navigator* - [GitHub](https://github.com/RajPShinde)
<br>I am a Masters in Robotics Engineering student at the University of Maryland, College Park. My primary area of interest are Legged Robotics and Automation. 
* **Prasheel Renkuntla** - *Sprint 1- Navigator & Sprint 2- Driver* - [GitHub](https://github.com/Prasheel24)
<br>I am a Master's in Robotics Engineering student at the University of Maryland, College Park. My primary area of interest is in Vision integrated Robot Systems.

For GMock - **Prasheel Renkuntla**

## Overview
This is a controller module for a robot that uses an Ackermann Steering Model, This controller is to be implemented in a four wheeled Robot made by ACME Robotics.

#### Description
A controller is build for a 4 wheeled robot with an Ackermann steering to navigate through its environment. The controller consist of a PID algorithm which  ensures that the velocity converges to the set point, and a Steer Algorithm that helps the robot turn.
<br>The PID Algorithm is a control loop mechanism that calculates the error and applies correction through proportional, integral and derivative gains. The Steer Algorithm is developed to turn the robot, which is done by calculating the length of an arc inscribed between the current robot heading and target heading, the length when divided by the robot velocity gives the time for which the wheels need to be kept at angles given by Ackermann steering Model.
<br>The input to the controller will be provided by the perception model developed by the ACME Robotics.
<br>The Demonstration of the controller will be given by plotting a graph that shows convergence of velocity & Heading angle to the targets with respect to time.

<br>Also, GMock is implemented on the module for the functions in Navigation class namely - Get and set Kp, Ki, Kd gains, calculatePID and calculate(to compute convergence) methods.


#### Features
* Velocity control during turning.
* Protection against Skidding, by limiting velocity during turning.
* Protection agains over volting the Motors.

#### Application
* Mobile Wheeled Robots

## Sprint Planning and Discussion
Sprint- https://docs.google.com/document/d/1w6U49tyKj9MFhaVziZ5MEGcaylmxSBOyGzClIz9lFA8/edit?usp=sharing

## Agile Iterative Process Log
Log- https://docs.google.com/spreadsheets/d/1LFQMKbuGeusgmI7IMbjiw-RJrt9jNgej0F8SvvfyJjY/edit?usp=sharing

## Dependencies
1. C++ 11/14/17
2. gnuplot- http://stahlke.org/dan/gnuplot-iostream/
<br>Install gnuplot
```
sudo apt-get install gnuplot
```
3. boost
```
sudo apt-get install libboost-all-dev
```
4. googletest
```
sudo apt isntall libgtest-dev
cd /usr/src/gtest/
sudo cmake -DBUILD_SHARED_LIBS=ON
sudo make
sudo cp *.so /usr/lib
```

## Build the specific branch
```
git clone --single-branch --branch GMock_Extra_Credit https://github.com/Prasheel24/Robot_Controller_Module.git 
cd <path to repository>
mkdir build
cd build
cmake ..
make
```
## Run
#### Run Program
```
./app/shell-app
```
#### Run Test with gmock
```
./test/cpp-test
```
## Doxygen Report
To generate doxygen documentation needs a dependency of doxygen.
To install doxygen: 
```
sudo apt-get install doxygen
sudo apt install doxygen-gui
```
To generate documentation:
```
doxywizard
```
Click on the tool for various options to get your required level of documentation. Also, documentation can be accessed from here: [Docs](https://github.com/Prasheel24/Robot_Controller_Module/tree/GmockForCredit/docs)

## Demo
Run the program. Once the velocity and heading converge to the target then graphs will be displayed as below. Also, converged values will be shown in the terminal.

<p align="center">
<h5> Heading Convergence</h5>
<img src="/output/HeadingConvergence.png">
</p>
</br>
<p align="center">
<h5> Velocity Convergence</h5>
<img src="/output/VelocityConvergence.png">
</p>
</br>
<p align="center">
<h5> Terminal Output</h5>
<img src="/output/TerminalOutput.png">
</p>

## Bugs
None

## References
* Ackermann Steering- https://www.sciencedirect.com/topics/engineering/ackermann
* PID Controller- https://en.wikipedia.org/wiki/PID_controller
* gnuplot- http://stahlke.org/dan/gnuplot-iostream/
* googletest - https://github.com/google/googletest
