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
\brief Target class source code
*/

// System Includes
#include <carla_ros_mtt/target.h>

ostream& operator<<(ostream& o,Target& t)
{
    o<<"target: "<<t._id;
    o<<" (#"<<t._uid<<")";
    o<<" cluster: "<<t._cluster;
    o<<" hypothesis: "<<t._hypothesis;
    o<<" miss associations: "<<t._missed_associations;
    o<<" measurements size: "<<t._assigned_measurements.size();
    return o;
}

void Target::incrementMissedAssociations(int inc)
{
    stt=10;

    _missed_associations+=inc;
}

void Target::decrementMissedAssociations(int dec)
{
    if(_missed_associations>0)
        _missed_associations-=dec;
}

void Target::zeroMissedAssociations(void)
{
    _missed_associations=0;
}

double Target::getDistance(MeasurementPtr& mes,int algorithm)
{
    //calculate the Mahalanobis between the node and the cluster
    Vector2d measurement;
    Vector2d mean;
    Matrix2d covariance;
// 	measurement(0)=mes->centroid.x;
// 	measurement(1)=mes->centroid.y;

    measurement(0)=mes->state[0];
    measurement(1)=mes->state[1];

    mean(0)=_xp(0);
    mean(1)=_xp(1);

    covariance(0,0)=_P(0,0);
    covariance(0,1)=_P(0,1);

    covariance(1,0)=_P(1,0);
    covariance(1,1)=_P(1,1);

    if(algorithm==0)//Mahalanobis
    {
        double val=mahalanobis(measurement,mean,covariance);
// 		cout<<"Dist calc, t: "<<_id<<" m: "<<mes->id<<" mah: "<<val<<endl;
        return val;
    }
    else if(algorithm==1)//Bivariate pdf
    {
        double val=biVariatePDF(measurement,mean,covariance);
// 		cout<<"Dist calc, t: "<<_id<<" m: "<<mes->id<<" biv: "<<val<<endl;
        return val;
    }
    else if(algorithm==2)//Euclidean
    {
        double val=sqrt(pow(measurement(0)-mean(0),2)+pow(measurement(1)-mean(1),2));
        return val;
    }

    return -1;
}

Target::Target()
{
    _hypothesis = -1;
    _cluster = -1;
// 	_id = -1;
    _uid = _euid++;
    _missed_associations = -1;
}

Target::~Target()
{
//	cout<<"deleted: "<<*this<<endl;
//	_assigned_measurements.clear();
//	_past_states.clear();
}

Target::Target(TargetPtr prev_target)
{
    //No association
    *this=*prev_target;

    //Copy estimator state
    initEstimator(prev_target->_x,prev_target->_P);

// 	kVector xx = _estimator.getX();
// 	kMatrix PP = _estimator.calculateP();
// 	cout<<"XX: "<<xx<<endl;
// 	cout<<"PP: "<<PP<<endl;

    assert(is_finite(prev_target->_x));
    assert(is_finite(prev_target->_P));

    _id=prev_target->_id;
    _uid = _euid++;

    Vector3d z;

    z(0)=prev_target->_xp(0);
    z(1)=prev_target->_xp(1);
    z(2)=prev_target->_xp(3);

    incrementMissedAssociations();

    _estimator.miss_associations++;
    _estimator.life_time++;

    _assigned_measurements.clear();

// 			cout<<"step in no association"<<endl;
//
    stepEstimator(z);
// 			cout<<"step done"<<endl;

    if(_past_states.size()>500)
        _past_states.erase(_past_states.begin());

    _past_states.push_back(_x);//Make a log of all past positions

// 	if(!is_finite(_xp))
// 	{
// 		cout<<"_x: "<<_x<<endl;
// 		cout<<"p _x: "<<prev_target->_x<<endl;
// 		cout<<"z: "<<z<<endl;
// 		cout<<"miss association"<<endl;
// 	}

    if(!is_finite(_xp))
    {
        const char COL_RESET[] = "\x1b[0m";
        const char RED[]       = "\x1b[31m";
        cout << RED << "Error!!, Major" << COL_RESET << endl;
        cout<<"Bad failed target: "<<*this<<endl;
    }

// 	assert(is_finite(_xp));
// 	assert(is_finite(_P));
}

Target::Target(TargetPtr prev_target,MeasurementPtr measurement)
{
// 	cout<<"NORMAL ASSOCIATION"<<endl;
    //Good association

    *this=*prev_target;

    initEstimator(prev_target->_x,prev_target->_P);

    _estimator.miss_associations=0;
    _estimator.life_time++;

    _id=prev_target->_id;
    _uid = _euid++;


    _assigned_measurements.clear();
    _assigned_measurements.push_back(measurement);

    zeroMissedAssociations();

    assert(is_finite(measurement->state));

// 	cout<<"step in normal"<<endl;
    stepEstimator(measurement->state);

    if(_past_states.size()>500)
        _past_states.erase(_past_states.begin());

    _past_states.push_back(_x);//Make a log of all past positions

    if(!is_finite(_xp))
        cout<<"Bad normal target: "<<*this<<endl;
}

Target::Target(MeasurementPtr measurement)
{
    //New target
    Vector3d z = measurement->state;
//	Vector3d

    assert(is_finite(measurement->state));

// 	cout<<"here"<<endl;
    initEstimator(z);
// 	cout<<"t"<<endl;

    _past_states.clear();//New target
    _past_states.push_back(_x);//Make a log of all past positions

    _hypothesis = 0;
    _cluster = 0;
    _missed_associations = 0;

    _estimator.miss_associations=0;
    _estimator.life_time=0;

    if(!is_finite(_xp))
        cout<<"Bad new target: "<<*this<<endl;

    _assigned_measurements.clear();
    _assigned_measurements.push_back(measurement);

// 	cout<<"new target"<<endl;
// 	assert(is_finite(_xp));
// 	assert(is_finite(_P));

// 	_id=_uid++;

    _ntotal++;
    _id = _ntotal;//To regular use
// 	_id = measurement->id;//Just with matlab ground truth data, the measurement id IS the ground truth.
    //We initialize each target with the correct id and check at each iteration if the association was good
    _uid = _euid++;


    _exuid = measurement->id;

// 	cout<<"tc: "<<_id<<" uid: "<<_uid<<endl;
    cout<<"||||||||!!!!!!!!"<<endl;
    cout<<"_ntotal: "<<_ntotal<<endl;
}

double Target::getDistanceToPredictedPosition(TargetPtr& target,int algorithm)
{
    //calculate the Mahalanobis between the current target and the new target
    Vector2d predicted_position;
    Vector2d mean;
    Matrix2d covariance;

    predicted_position(0)=target->_xp(0);
    predicted_position(1)=target->_xp(1);

    mean(0)=_xp(0);
    mean(1)=_xp(1);

    covariance(0,0)=_P(0,0);
    covariance(0,1)=_P(0,1);

    covariance(1,0)=_P(1,0);
    covariance(1,1)=_P(1,1);

    if(algorithm==0)
    {
        double val=mahalanobis(predicted_position,mean,covariance);
// 		cout<<"Dist calc, t: "<<_id<<" m: "<<mes->id<<" mah: "<<val<<endl;
        return val;
    }
    else
    {
        double val=biVariatePDF(predicted_position,mean,covariance);
// 		cout<<"Dist calc, t: "<<_id<<" m: "<<mes->id<<" biv: "<<val<<endl;
        return val;
    }
}

void Target::cleanTargetAssociations(void)
{
    _assigned_measurements.clear();
}

bool compareTargetsById(TargetPtr t1,TargetPtr t2)
{
    return t1->_id<t2->_id;
}

bool compareTargetsByAux1(TargetPtr t1,TargetPtr t2)
{
    return t1->_aux1<t2->_aux1;
}

bool compareTargetsByAux1Descending(TargetPtr t1,TargetPtr t2)
{
    return t1->_aux1>t2->_aux1;
}