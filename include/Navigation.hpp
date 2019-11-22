/************************************************************************
MIT License
Copyright © 2019 Raj Shinde
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************/

/**
 *  @copyright MIT License, Copyright © 2019 Raj Shinde
 *  @file    Navigation.hpp
 *  @author  Sprint-1 Raj Shinde- driver and Prasheel Renkuntla- navigator
 *  @author  Sprint-2 Prasheel Renkuntla- driver and Raj Shinde- navigator
 *  @date    10/10/2019
 *  @version 6.0
 *  @brief   Mid Term Project
 *  @section Header file for Navigation through PID control
 */

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include "SteerAlgorithm.hpp"

#include <vector>
/**
 *  @brief Class Navigation 
 *  Contains the methods of Pid Algorithm
 */
class Navigation : public SteerAlgorithm {
 private:
    double kp_, ki_, kd_, diffTime_, error_, previousError_;
    const double maxMotorSpeed_ = 60;

 public:
    bool motorDirection = true;

   /**
    *  @brief Constructor of class Navigation
    */
    Navigation();

   /**
    *  @brief Destructor of class Navigation
    */
    virtual ~Navigation();

    /**
    *  @brief Function to calculate new velocity using PID in m/s
    *  @param double setPoint, Setpoint of the robot
    *  @param double currentVelocity, current Velocity of the robot 
    *  @return double std::vector<double>, values of error, Proportional
    *                                      output, Integral output,
    *                                      derivative output
    */
    virtual std::vector<double> calculatePID(double setPoint,
                                             double currentVelocity);
    /**
    *  @brief Function to use GNUplot for velocity convergence graph
    *  @param std::vector<std::pair<double, double>> points vector
    *  @param double newVelocity,  New velocity from code
    *  @param bool flag, to test the gnuplot working
    *  @return none
    */
    bool gnuVelocityGraph(std::vector<std::pair<double, double>>
                          vectorPointsVel, double newVelocity, bool flag);
    /**
    *  @brief Function to use GNUplot for Steer angle convergence graph
    *  @param std::vector<std::pair<double, double>> points vector
    *  @param double steerAngle,  steer angle from code
    *  @param bool flag, to test the gnuplot working
    *  @return none
    */
    bool gnuSteerAngleGraph(std::vector<std::pair<double, double>>
                            vectorPointsSteer, double steerAngle, bool flag);

   /**
    *  @brief Function to calculate new velocity in m/s with a PID 
    *  Algorithm using kp, ki & kd
    *  @param double targetHeading, Target heading of the robot
    *  @param double currentVelocity, current velocity of robot
    *  @param double setPoint, Target Velocity
    *  @param int flag, to enable while
    *  @return double newVelocity
   */
    virtual double calculate(double targetHeading, double currentVelocity,
                     double setPoint, int flag);

   /**
    *  @brief Function to get kp_
    *  @param none
    *  @return double
    */
    virtual double getKp_();

   /**
    *  @brief Function to get ki_
    *  @param none
    *  @return double 
    */
    virtual double getKi_();

   /**
    *  @brief Function to get kd_
    *  @param none
    *  @return double 
    */
    virtual double getKd_();

   /**
    *  @brief Function to set kp_
    *  @param double kp
    *  @return boolean true
    */
    virtual bool setKp_(double kp);

   /**
    *  @brief Function to set ki_
    *  @param double ki
    *  @return boolean true
    */
    virtual bool setKi_(double ki);

   /**
    *  @brief Function to set kd_
    *  @param double kd
    *  @return boolean true
    */
    virtual bool setKd_(double kd);
};

#endif  // INCLUDE_NAVIGATION_HPP_
