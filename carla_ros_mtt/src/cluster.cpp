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
/**
\file
\brief Hypotheses cluster class source code
*/

// System Includes
#include <carla_ros_mtt/cluster.h>

Cluster::Cluster()
{
    id=0;
}

Cluster::~Cluster()
{
// 	cout<<"deleted: "<<*this<<endl;
// 	assigned_measurements.clear();
// 	assigned_hypotheses.clear();
}

bool Cluster::isEmpty()
{
    if(assigned_hypotheses.size()==0)
        return true;

    bool empty=true;

    for(uint i=0;i<assigned_hypotheses.size();i++)
        if(assigned_hypotheses[i]->_status!=DEAD)
            empty=false;

    if(empty)
        return true;

    return false;
}

ostream& operator<<(ostream& o, Cluster& c)
{
    o<<"cluster: "<<c.id<<endl;

    o<<"\tmeasurements: "<<endl;

    if(c.assigned_measurements.size()==0)
        o<<"\t\t[]"<<endl;

    for(uint i=0;i<c.assigned_measurements.size();++i)
        o<<"\t\t"<<*(c.assigned_measurements[i])<<endl;

    o<<"\thypotheses: "<<endl;

    if(c.assigned_hypotheses.size()==0)
        o<<"\t\t[]"<<endl;

    for(uint i=0;i<c.assigned_hypotheses.size();++i)
    {
        HypothesisPtr hypothesis = c.assigned_hypotheses[i];

        o<<"\t\t"<<hypothesis->_uid;

        switch(hypothesis->_status)
        {
            case DEAD:
                o<<"D";
                break;
            case NORMAL:
                o<<"N";
                break;
            case PARENT:
                o<<"P";
                break;
            case FORCED_PARENT:
                o<<"FP";
                break;
            case DEAD_FORCED_PARENT:
                o<<"DFP";
                break;
            case ERROR:
                o<<"E";
                break;
        }

        o<<" (";

        for(uint t=0;t<hypothesis->_targets.size();t++)
        {
            o<<hypothesis->_targets[t]->_id;
            o<<" mi "<<hypothesis->_targets[t]->_missed_associations;

            if(t<hypothesis->_targets.size()-1)
                o<<", ";
        }

        o<<")"<<endl;

// 				if(i<c.assigned_hypotheses.size()-1)
// 					o<<", ";
    }

    return o;
}

bool compareClusters(ClusterPtr c1,ClusterPtr c2)
{
    return c1->id<c2->id;
}

ostream& operator<<(ostream& o,vector<ClusterPtr>& c)
{
    for(uint i=0;i<c.size();i++)
    {
        o<<*c[i];
    }
    return o;
}
