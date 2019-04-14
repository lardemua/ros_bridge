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

#ifndef ROS_BRIDGE_MEASUREMENT_H
#define ROS_BRIDGE_MEASUREMENT_H

/**
\file
\brief Measurement class declaration
*/

// System Includes
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include <vector>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <mtt/point.h>
#include <mtt/cluster.h>

using Eigen::Vector2d;
using Eigen::Vector3d;

class Cluster;

///Shared pointer to the Cluster class
typedef boost::shared_ptr<Cluster> ClusterPtr;

/**
\brief Measurement class type
Nonholonomic measurement class
*/
class Measurement
{
    public:

        ///Vector of the support points
        vector<PointPtr> points;
        ///Centroid of the measurement
        Point centroid;
        ///Id of the measurement
        long id;

        Vector3d state;

        ///Hypotheses cluster association vector
        vector<ClusterPtr> assigned_clusters;

        Measurement();

        ~Measurement();

        Measurement & operator=(const Measurement &rhs);

        /**
        \brief Break point detection used as clustering criterion
        \param pt first point
        \param pt_m1 next point
        \return true if the next point belongs to a new cluster
        */
        bool breakPointDetector(PointPtr&pt,PointPtr&pt_m1);

        /**
        \brief Calculate the centroid of the cluster
        */
        void calculateCentroid(void);

        bool operator<(Measurement&);
        bool operator>(Measurement&);

        // 		friend ostream& operator<<(ostream& o, Measurement& m);

    private:
};

///Shared pointer to the Measurement class
typedef boost::shared_ptr<Measurement> MeasurementPtr;

ostream& operator<<(ostream& o, Measurement& m);
ostream& operator<<(ostream& o,vector<MeasurementPtr>& m);

/**
\brief This function compares two measurements just to sort them
The sorting is preformed using the id variable in the Measurement class.
\param m1 first measurement
\param m2 second measurement
\return true if m1\<m2
*/
bool compareMeasurements(MeasurementPtr m1,MeasurementPtr m2);
using namespace std;

#endif //ROS_BRIDGE_MEASUREMENT_H
