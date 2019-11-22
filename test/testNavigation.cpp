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
 *  @file    testNavigation.cpp
 *  @author  Sprint-1 Raj Shinde- driver and Prasheel Renkuntla- navigator
 *  @author  Sprint-2 Prasheel Renkuntla- driver and Raj Shinde- navigator
 *  @date    10/10/2019
 *  @version 6.0
 *  @brief   Mid Term Project
 *  @section Tests functionality of Navigation code
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>

#include "Navigation.hpp"

/**
 *  @brief Test to check the set functions
 *  for kp_,ki_ and kd_
 */
TEST(Navigation, testSetPid) {
Navigation testn;
//  Expect true if we are able to set the gains
EXPECT_TRUE(testn.setKp_(1));
EXPECT_TRUE(testn.setKi_(1));
EXPECT_TRUE(testn.setKd_(1));
}

/**
 *  @brief Test to check that values of
 *  kp_,ki_ and kd_ are below 1 or not
 */
TEST(Navigation, testGetPid) {
Navigation testn;
//  Expect true if it returns the gains set from above
EXPECT_LT(testn.getKp_(), 1);
EXPECT_LT(testn.getKi_(), 1);
EXPECT_LT(testn.getKd_(), 1);
}

/**
 *  @brief Test to check the output of calculatePID
 *  function to check errorvalue and output.
 */
TEST(Navigation, testPID) {
Navigation testn;
std::vector<double> out{0.0, 0.0, 0.0, 0.0}; 
EXPECT_EQ(testn.calculatePID(10, 10), out);
}

/**
 *  @brief Test to check the output of calculate
 *  function converges with the setPoint and
 *  Heading
 */
TEST(Navigation, testConvergence) {
Navigation testn;
//  Calculates the convergence for setpoint:40, Current Velocity: 10
//  target heading: 0, and flag is 1 as we do not need graphs
ASSERT_NEAR(testn.calculate(0, 10, 40, 1), 40, 2);
//  Calculates the convergence for setpoint:40, Current Velocity: 10
//  target heading: 120, and flag is 0 as we do not need graphs
ASSERT_NEAR(testn.calculate(120, 10, 40, 0), 120, 1);
}


