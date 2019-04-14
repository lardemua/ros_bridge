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
\brief Hypothesis class source code
*/

// System Includes
#include <carla_ros_mtt/hypothesis.h>

Hypothesis::Hypothesis(int iteration, int cluster, int status)
{
    Hypothesis();

    _iteration=iteration;
    _cluster=cluster;
    _status=status;
}

Hypothesis::~Hypothesis()
{
//	cout<<"deleted: "<<*this<<endl;
//		_targets.clear();
}

string Hypothesis::nameUI()
{
    boost::format fm("H (%d-%d) %ld #%ld\n%.3f :%d");
    fm % _iteration % _cluster % _id % _uid % _probability % _targets.size();
    return fm.str();
}

string Hypothesis::name()
{
    boost::format fm("H (%d-%d) %ld #%ld");
    fm % _iteration % _cluster % _id % _uid;
    return fm.str();
}

void Hypothesis::setAttribute(string name,string value)
{
    if(find(_attribute_names.begin(),_attribute_names.end(),name)==_attribute_names.end())//Not found
    {
        _attribute_names.push_back(name);
        _attribute_values.push_back(value);

        return;
    }

    for(uint i=0;i<_attribute_names.size();++i)
        if(_attribute_names[i]==name)
        {
            _attribute_values[i]=value;
            return;
        }
}

Hypothesis::Hypothesis()
{
    _euid++;
    _uid=_euid;
    _id=0;
    _status=0;
    _aux1=-1;
    _cluster=-1;
    _iteration=-1;
    _probability=0;
    _targets.clear();
    _attribute_names.clear();
    _attribute_values.clear();

    _n_det=0;
    _n_occ=0;
    _n_del=0;
    _n_new=0;
    _n_fal=0;
    _prod=1;

    _parent_uid=-1;

// 	_n_children=0;
}

ostream& operator<<(ostream& o,Hypothesis& h)
{
    o<<"hypothesis uid: "<<h._uid;
    o<<" cluster: "<<h._cluster;
    o<<" iteration: "<<h._iteration;
    o<<" status: "<<h._status;
    return o;
}


bool compareHypothesesByProbability(HypothesisPtr h1,HypothesisPtr h2)
{
    return h1->_probability < h2->_probability;
}

bool compareHypothesesByProbabilityDescending(HypothesisPtr h1,HypothesisPtr h2)
{
    return h1->_probability > h2->_probability;
}

bool compareHypotheses(HypothesisPtr h1,HypothesisPtr h2)
{
    return h1->_uid<h2->_uid;
}