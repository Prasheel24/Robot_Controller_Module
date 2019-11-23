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
 *  @file    SteerAlgorithm.cpp
 *  @author  Sprint-1 Raj Shinde- driver and Prasheel Renkuntla- navigator
 *  @author  Sprint-2 Prasheel Renkuntla- driver and Raj Shinde- navigator
 *  @author  Gmock Prasheel Renkuntla
 *  @date    11/22/2019
 *  @version 7.0
 *  @brief   Mid Term Project
 *  @section Implements Ackermann Steering
 */

#include <iostream>
#include <cmath>

#include "SteerAlgorithm.hpp"
/**
 *  @brief Constructor of class SteerAlgorithm
 */
SteerAlgorithm::SteerAlgorithm() {
//   Initialised variables
lWheelAngle_ = 0;
rWheelAngle_ = 0;
heading = 0;
robotAngle_ = 0;
corrRadius_ = 30;
dir = 1;
targetHeading = 1;
}

/**
 *  @brief Destructor of class SteerAlgorithm
 */
SteerAlgorithm::~SteerAlgorithm() {}

/**
 *  @brief Function to get corresponding radius in meters
 *  @param none
 *  @return double corresponding radius
 */
double SteerAlgorithm::getCorrRadius_() {
return corrRadius_;
}

/**
 *  @brief Function to set corresponding radius in meters
 *  @param double radius
 *  @return bool true
 */
bool SteerAlgorithm::setCorrRadius_(double r) {
bool flag = true;
corrRadius_ = r;
if(corrRadius_ != r) {
       flag = false;
}
return flag;
}

/**
 *  @brief Function to calculate the length of arc in meters
 *  to be traced in order to head in target direction
 *  @param double diffAngle, difference in current and 
 *  target heading
 *  @param double corrRadius, corresponding radius
 *  @return double arclength
 */
double SteerAlgorithm::arcLength(double diffAngle, double corrRadius) {
        if ( diffAngle == 0 || diffAngle == 360 ) {
             diffAngle = 0;
        } else if (diffAngle == 90 || diffAngle == 180) {
             diffAngle = diffAngle + 0;
        } else if (diffAngle < 90) {
             diffAngle = diffAngle + 270;
        } else {
        }
return ((diffAngle/360)*(2*M_PI*corrRadius));
}

/**
 *  @brief Function to calculate the angles in degrees for left and
 *  right wheels as per ackermann model, and then feed them
 *  to corresponding servos
 *  @param double corrRadius, corresponding radius
 *  @param double shaftLength, length between wheels
 *  @param double shafDistance, distance between rear and
 *  front shaft
 *  @return double maxWheelAngle
 */
double SteerAlgorithm::changeWheelAngles(double corrRadius,
                                         double shaftLength,
                                         double shaftDistance) {
        if ( targetHeading > 0 ) {
               dir = 1;
        } else {
               dir = -1;
        }

//  To calculate the angles, follow this link
//  https://www.sciencedirect.com/topics/engineering/ackermann
lWheelAngle_ = (90 - (180/M_PI)*std::atan((corrRadius +
               (shaftLength * 0.5))/ shaftDistance) * dir);
rWheelAngle_ = (90 - (180/M_PI)*std::atan((corrRadius -
               (shaftLength * 0.5))/ shaftDistance) * dir);
return std::max(lWheelAngle_, rWheelAngle_);
}

/**
 *  @brief Function to set wheel angles to 0
 *  @param none
 *  @return bool true
 */
bool SteerAlgorithm::resetWheel() {
bool flag = true;
lWheelAngle_ = 0;
rWheelAngle_ = 0;
if(lWheelAngle_ != 0 && rWheelAngle_ != 0) {
       flag = false;
}
return flag;
}

/**
 *  @brief Function to calculate time in seconds required to
 *  turn or keep wheels at the an angle
 *  @param double arcLength, length of arc to be traced
 *  @param double newVelocity, velocity
 *  @return double time
 */
double SteerAlgorithm::turnTime(double arclength, double newVelocity) {
return (arclength/newVelocity);
}
