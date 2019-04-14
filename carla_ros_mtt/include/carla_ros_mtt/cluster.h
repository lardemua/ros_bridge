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

#ifndef ROS_BRIDGE_CLUSTER_H
#define ROS_BRIDGE_CLUSTER_H

/**
\file
\brief Hypothesis cluster definition
*/

// System Includes
#include <vector>
#include <boost/shared_ptr.hpp>
#include <mtt/measurement.h>
#include <mtt/hypothesis.h>

using namespace std;

class Hypothesis;
typedef boost::shared_ptr<Hypothesis> HypothesisPtr;

class Measurement;
typedef boost::shared_ptr<Measurement> MeasurementPtr;

/**
 * \brief Hypotheses cluster class
 *
 * This class contains the representation of a single hypotheses cluster.
 * The cluster contains a set of associated measurements (assigned_measurements) and a set of assigned hypotheses (assigned_hypotheses).
 * These two sets constitute the ambiguity problem and will be solved together, creating associations between the targets in the hypotheses and the measurements.
 */
class Cluster
{
    public:
        ///Id of the current cluster.
        long id;
        ///Measurements assigned to this cluster, these measurements are in conflict with the cluster targets
        vector<MeasurementPtr> assigned_measurements;
        ///Set of hypotheses belonging to the cluster
        vector<HypothesisPtr> assigned_hypotheses;

        ///Cluster constructor, variable initialization
        Cluster();

        ///Cluster destructor, no task
        ~Cluster();

        /**
         * \brief Check if the cluster is a candidate for deletion
         *
         * Tests the number of hypotheses and their status. If there are non hypotheses or all hypotheses are dead this cluster is removed.
         *
         * \return true if the cluster is ready for deletion
         */
        bool isEmpty();

        ///Print function
        friend ostream& operator<<(ostream& o, Cluster& c);
};

///Shared pointer to the Cluster class
typedef boost::shared_ptr<Cluster> ClusterPtr;

/**
 * \brief Compare two clusters
 *
 * This function is used to perform sorting operations.
 * \param c1 first cluster
 * \param c2 second cluster
 * \return true if c2 id is larger that c1
 */
bool compareClusters(ClusterPtr c1,ClusterPtr c2);

///Cluster vector print function
ostream& operator<<(ostream& o,vector<ClusterPtr>& c);

#endif //ROS_BRIDGE_CLUSTER_H
