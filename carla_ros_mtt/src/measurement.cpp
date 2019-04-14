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
\brief Measurement class source code
*/

// System Includes
#include <carla_ros_mtt/measurement.h>

Measurement::Measurement()
{
    id=0;
}

Measurement::~Measurement()
{
//	cout<<"deleted: measurement: "<<id<<endl;
//	assigned_clusters.clear();
//	points.clear();
}

ostream& operator<<(ostream& o, Measurement& m)
{
// 	o<<"measurement: "<<m.id<<" centroid: "<<m.centroid<<" clusters: ";
    o<<"measurement: "<<m.id<<" clusters: ";

    if(m.assigned_clusters.size()==0)
        o<<"[]";

    for(uint c=0;c<m.assigned_clusters.size();c++)
    {
        o<<(m.assigned_clusters[c])->id;

        if(c<m.assigned_clusters.size()-1)
            o<<", ";
    }

    return o;
}

Measurement& Measurement::operator=(const Measurement &rhs)
{
    id=rhs.id;
    centroid=rhs.centroid;

    PointPtr p;

    for(uint i=0;i<rhs.points.size();i++)
    {
        p.reset(new Point);
        *p=*(rhs.points[i]);

        points.push_back(p);
    }

    return *this;
}

bool Measurement::breakPointDetector(PointPtr&pt,PointPtr&pt_m1)
{
    /**
    \todo change the clustering criterion to something more mathematically accurate
    */

    double d = sqrt( pow(pt->x-pt_m1->x,2) + pow(pt->y-pt_m1->y,2));

    if(d>2.0)
        return true;

    return false;
}

void Measurement::calculateCentroid(void)
{
    centroid.setZero();

    for(uint i=0;i<points.size();i++)
        centroid+=*points[i];

    centroid.scale(1./points.size());
}

bool Measurement::operator<(Measurement &m)
{
    cout<<"calling this function"<<endl;
    return this->id<m.id;
}

bool Measurement::operator>(Measurement &m)
{
    return this->id>m.id;
}


ostream& operator<<(ostream& o,vector<MeasurementPtr>& m)
{
    for(uint i=0;i<m.size();++i)
    {
        o<<"measurement: "<<m[i]->id<<" = ";

        if(m[i]->assigned_clusters.size()==0)
            o<<"[]";

        for(uint c=0;c<m[i]->assigned_clusters.size();++c)
        {
            o<<m[i]->assigned_clusters[c];
            if(c<m[i]->assigned_clusters.size()-1)
                o<<" ,";
        }
        o<<endl;
    }

    return o;
}