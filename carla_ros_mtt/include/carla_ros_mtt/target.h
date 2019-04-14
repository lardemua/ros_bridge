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

#ifndef ROS_BRIDGE_TARGET_H
#define ROS_BRIDGE_TARGET_H

/**
\file
\brief Target class declaration
*/

// System Includes
#include <vector>
#include <string>
#include <mtt/measurement.h>
#include <mtt/motionModel.h>
#include <mtt/metrics.h>

using namespace std;

class Measurement;
///Shared pointer to the Measurement class
typedef boost::shared_ptr<Measurement> MeasurementPtr;

class Target;
///Shared pointer to the Target class
typedef boost::shared_ptr<Target> TargetPtr;

class Target: public MotionModel
{
    public:

        double stt;

        long _hypothesis;
        long _cluster;
        long _id;
        long _missed_associations;
        string _variant;
        long _uid;
        long _exuid;
        // 		static long _euid;
        // 		static long _ntotal;
        static long _euid;
        static long _ntotal;

        double _aux1;

        vector<MeasurementPtr> _assigned_measurements;
        vector<long> _concurrent_targets;
        vector<Vector5d> _past_states;

        Target();

        ~Target();

        Target(TargetPtr prev_target);

        Target(TargetPtr prev_target,MeasurementPtr measurement);

        Target(MeasurementPtr measurement);

        void incrementMissedAssociations(int inc=1);

        void decrementMissedAssociations(int dec=1);

        void zeroMissedAssociations(void);

        double getDistance(MeasurementPtr& mes,int algorithm=0);

        double getDistanceToPredictedPosition(TargetPtr& target,int algorithm=0);

        void cleanTargetAssociations(void);

        friend ostream& operator<<(ostream& o,Target& t);
};

bool compareTargetsById(TargetPtr t1,TargetPtr t2);
bool compareTargetsByAux1(TargetPtr t1,TargetPtr t2);
bool compareTargetsByAux1Descending(TargetPtr t1,TargetPtr t2);

#endif //ROS_BRIDGE_TARGET_H
