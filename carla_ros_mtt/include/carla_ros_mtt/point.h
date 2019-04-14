//
// Created by pedro on 14-04-2019.
//

/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/

#ifndef ROS_BRIDGE_POINT_H
#define ROS_BRIDGE_POINT_H

/**
\file
\brief Point class declaration
*/

// System Includes
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>

using namespace std;

using Eigen::Vector2d;

class Point;
///Shared pointer to the Point class
typedef boost::shared_ptr<Point> PointPtr;


/**
\brief Simple point class
*/
class Point
{
    public:

        Point();

        /**
        \brief Set all the point fields to zero
        */
        void setZero();

        /**
        \brief Overload += operator to allow class addition
        */
        Point& operator+=(const Point &rhs);

        /**
        \brief Scale all point values by a factor
        \param val scale factor
        */
        void scale(double val);

        Vector2d toVector2d();

        ///x coordinate (Cartesian)
        double x;
        ///y coordinate (Cartesian)
        double y;
        ///z coordinate (Cartesian)
        double z;
        ///r coordinate (Polar)
        double r;
        ///t coordinate (Polar)
        double t;
        ///auxiliary identifier
        double n;
        ///auxiliary identifier 2
        double ni;

        ostream& operator<<(ostream& o);

    private:
};

#endif //ROS_BRIDGE_POINT_H
