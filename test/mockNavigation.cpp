#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include "iostream"

#include "Navigation.hpp"

using ::testing::_;

class MockNavigation : public Navigation{
  public :
    MockNavigation() : Navigation() {}
    MOCK_METHOD0( getKp_, double());
    MOCK_METHOD1( setKp_, bool(double));
    MOCK_METHOD0( getKi_, double());
    MOCK_METHOD1( setKi_, bool(double));
    MOCK_METHOD0( getKd_, double());
    MOCK_METHOD1( setKd_, bool(double));
    MOCK_METHOD2( calculatePID, std::vector<double>(double, double));
    MOCK_METHOD4( calculate, double(double targetHeading,
                                    double currentVelocity,
                                    double setPoint,
                                    int flag) );    
};

TEST(MockNavigation, checkKp) {
MockNavigation mn;
EXPECT_CALL(mn, setKp_(0.4)).Times(1);
mn.setKp_(0.4);
EXPECT_CALL(mn, getKp_()).Times(1).WillOnce(::testing::Return(0.4));

EXPECT_EQ(0.4, mn.getKp_());
}

TEST(MockNavigation, checkKi) {
MockNavigation mn;
EXPECT_CALL(mn, setKi_(0.04)).Times(1);
mn.setKi_(0.04);
EXPECT_CALL(mn, getKi_()).Times(1).WillOnce(::testing::Return(0.04));

EXPECT_EQ(0.04, mn.getKi_());
}

TEST(MockNavigation, checkKd) {
MockNavigation mn;
EXPECT_CALL(mn, setKd_(0.01)).Times(1);
mn.setKd_(0.01);
EXPECT_CALL(mn, getKd_()).Times(1).WillOnce(::testing::Return(0.01));

EXPECT_EQ(0.01, mn.getKd_());
}

TEST(MockNavigation, checkPID) {
MockNavigation mn;
EXPECT_CALL(mn, calculatePID(40, 10)).Times(1);
std::vector<double> out; 
EXPECT_EQ(mn.calculatePID(40, 10), out);
}

TEST(MockNavigation, checkNavigation) {
MockNavigation mn;
EXPECT_CALL(mn, calculate(90, 10, 40, 1)).Times(2);
mn.calculate(90, 10, 40, 1);
EXPECT_EQ(mn.calculate(90, 10, 40, 1), 0);
}
