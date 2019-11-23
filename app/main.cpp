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
 *  @file    main.cpp
 *  @author  Sprint-1 Raj Shinde- driver and Prasheel Renkuntla- navigator
 *  @author  Sprint-2 Prasheel Renkuntla- driver and Raj Shinde- navigator
 *  @author  Gmock Prasheel Renkuntla
 *  @date    11/22/2019
 *  @version 7.0
 *  @brief   Mid Term Project
 *  @section Graphical demo of Ackermann Steering on PID Control
 */

#include <iostream>

#include "Navigation.hpp"
#include "SteerAlgorithm.hpp"

/**
 *  @brief  Main Function to call the methods and show demostration
 *  @param  None
 *  @return 0
 */
int main() {
Navigation nav;
nav.setKp_(0.5);
nav.setKi_(0.05);
nav.setKd_(0.001);
nav.calculate(90, 10, 40, 2);
std::cout << "For Kp :" << nav.getKp_()
          << " Ki : " << nav.getKi_()
          << " Kd : " << nav.getKd_() << std::endl;
return 0;
}
