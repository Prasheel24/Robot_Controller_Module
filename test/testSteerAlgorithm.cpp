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
 *  @file    testSteerAlgorithm.cpp
 *  @author  Sprint-1 Raj Shinde- driver and Prasheel Renkuntla- navigator
 *  @author  Sprint-2 Prasheel Renkuntla- driver and Raj Shinde- navigator
 *  @author  Gmock Prasheel Renkuntla
 *  @date    11/22/2019
 *  @version 7.0
 *  @brief   Mid Term Project
 *  @section Tests functionality of SteerAlgorithm Code
 */
#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>

#include "SteerAlgorithm.hpp"

/**
 *  @def TEST(SteerAlgorithm, testCorrRadius)
 *  @brief Tests to check that the set functions works 
 *  and the value of corrRadius in below setlimit
 */
TEST(SteerAlgorithm, testCorrRadius) {
SteerAlgorithm tests;
EXPECT_TRUE(tests.setCorrRadius_(100));
EXPECT_LT(tests.getCorrRadius_(), 150);
}

/**
 *  @def TEST(SteerAlgorithm, testwheel)
 *  @brief Tests to check if the wheel angles get 
 *  resetted and the Ackeermann kinematic model is
 *  is properly implemented
 */
TEST(SteerAlgorithm, testwheel) {
SteerAlgorithm tests;
EXPECT_TRUE(tests.resetWheel());
EXPECT_LT(tests.changeWheelAngles(10, 4, 8), 46);
}

/**
 *  @def TEST(SteerAlgorithm, testCalculations)
 *  @brief Test to check the functions arclength 
 *  and turnTime provide right length and time values
 */
TEST(SteerAlgorithm, testCalculations) {
SteerAlgorithm tests;
EXPECT_LT(tests.arcLength(360, 10), 70);
EXPECT_GT(tests.turnTime(100, 20), 0);
}

