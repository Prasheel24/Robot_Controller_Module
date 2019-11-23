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
 *  @file    mockNavigation.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/22/2019
 *  @version 7.0
 *  @brief   Mid Term Project
 *  @section Tests functionality of Navigation code using GMock
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include "iostream"

#include "Navigation.hpp"

/**
 *  @brief Mock Class Navigation 
 *  To implement google mock
 */
class MockNavigation : public Navigation{
  public :
   /**
    *  @brief Initialise the constructor
    */
    MockNavigation() : Navigation() {}

   /**
    *  @brief Mock method to get Kp_
    */
    MOCK_METHOD0( getKp_, double());

   /**
    *  @brief Mock method to set Kp_
    */
    MOCK_METHOD1( setKp_, bool(double));

   /**
    *  @brief Mock method to get Ki_
    */
    MOCK_METHOD0( getKi_, double());

   /**
    *  @brief Mock method to set Ki_
    */
    MOCK_METHOD1( setKi_, bool(double));

   /**
    *  @brief Mock method to get Kd_
    */
    MOCK_METHOD0( getKd_, double());

   /**
    *  @brief Mock method to set Kd_
    */
    MOCK_METHOD1( setKd_, bool(double));

   /**
    *  @brief Mock method to calculate PID
    */
    MOCK_METHOD2( calculatePID, std::vector<double>(double, double));

   /**
    *  @brief Mock method to compute the convergence
    */
    MOCK_METHOD4( calculate, double(double targetHeading,
                                    double currentVelocity,
                                    double setPoint,
                                    int flag) );    
};

/**
 *  @def TEST(MockNavigation, checkKp)
 *  @brief Test Kp through gmock
 */
TEST(MockNavigation, checkKp) {
MockNavigation mn;
//  Expect call to set kp
EXPECT_CALL(mn, setKp_(0.4)).Times(1);
mn.setKp_(0.4);
//  Expect call to get kp
EXPECT_CALL(mn, getKp_()).Times(1).WillOnce(::testing::Return(0.4));
EXPECT_EQ(0.4, mn.getKp_());
}

/**
 *  @def TEST(MockNavigation, checkKi)
 *  @brief Test Ki through gmock
 */
TEST(MockNavigation, checkKi) {
MockNavigation mn;
//  Expect call to set ki
EXPECT_CALL(mn, setKi_(0.04)).Times(1);
mn.setKi_(0.04);
//  Expect call to get ki
EXPECT_CALL(mn, getKi_()).Times(1).WillOnce(::testing::Return(0.04));
EXPECT_EQ(0.04, mn.getKi_());
}

/**
 *  @def TEST(MockNavigation, checkKd)
 *  @brief Test Kd through gmock
 */
TEST(MockNavigation, checkKd) {
MockNavigation mn;
//  Expect call to set kd
EXPECT_CALL(mn, setKd_(0.01)).Times(1);
mn.setKd_(0.01);
//  Expect call to get kd
EXPECT_CALL(mn, getKd_()).Times(1).WillOnce(::testing::Return(0.01));

EXPECT_EQ(0.01, mn.getKd_());
}

/**
 *  @def TEST(MockNavigation, checkPID)
 *  @brief Test PID through gmock
 */
TEST(MockNavigation, checkPID) {
MockNavigation mn;
//  Expect call to compute PID output
EXPECT_CALL(mn, calculatePID(40, 10)).Times(1);
std::vector<double> out; 
EXPECT_EQ(mn.calculatePID(40, 10), out);
}

/**
 *  @def TEST(MockNavigation, checkNavigation)
 *  @brief Test Convergence through gmock
 */
TEST(MockNavigation, checkNavigation) {
MockNavigation mn;
//  Expect call to compute Convergence
EXPECT_CALL(mn, calculate(90, 10, 40, 1)).Times(2);
mn.calculate(90, 10, 40, 1);
EXPECT_EQ(mn.calculate(90, 10, 40, 1), 0);
}
