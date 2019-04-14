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
\brief Constant velocity Kalman Filter source code
*/

// System Includes
#include <carla_ros_mtt/types_declaration.h>

void constant_velocity_ekfilter::SetIdentity(Matrix& M,int size,double value)
{
    for(int l=0;l<size;l++)
        for(int c=0;c<size;c++)
            if(c==l)
                M(l,c)=value;
            else
                M(l,c)=0;
}

void constant_velocity_ekfilter::SetZero(Matrix& M,int size)
{
    for(int l=0;l<size;l++)
        for(int c=0;c<size;c++)
            M(l,c)=0;
}

void constant_velocity_ekfilter::InitFilter(Vector4d& x_init)
{
    setDim(4,0,4,2,2);
    dt=0;
    lt=Time::now();

    Vector x_0(4);

    x_0(0)=x_init(0);//x
    x_0(1)=x_init(1);//y
    x_0(2)=x_init(2);//vx
    x_0(3)=x_init(3);//vy

    Matrix P_0(4,4);

    SetIdentity(P_0,4,10.);

    init(x_0,P_0);
}

void constant_velocity_ekfilter::makeCommonProcess()
{
// 	dt=Time::now().toSec()-lt.toSec();
// 	lt=Time::now();
    dt=1./50.;
}

void constant_velocity_ekfilter::makeBaseA()
{
    A(0,0)=1.0;
    A(0,1)=0.0;
// 	A(0,2)=dt;
    A(0,3)=0.0;

    A(1,0)=0.0;
    A(1,1)=1.0;
    A(1,2)=0.0;
// 	A(1,3)=dt;

    A(2,0)=0.0;
    A(2,1)=0.0;
    A(2,2)=1.0;
    A(2,3)=0.0;

    A(3,0)=0.0;
    A(3,1)=0.0;
    A(3,2)=0.0;
    A(3,3)=1.0;
}

void constant_velocity_ekfilter::makeA()
{
    A(0,2)=dt;
    A(1,3)=dt;
}

void constant_velocity_ekfilter::makeBaseH()
{
    H(0,0) = 1.0;
    H(1,1) = 1.0;
}

void constant_velocity_ekfilter::makeBaseW()
{
    SetIdentity(W,4,1.0);
}

void constant_velocity_ekfilter::makeBaseV()
{
    SetIdentity(V,2,1.0);
}

void constant_velocity_ekfilter::makeR()
{
    double dir = atan2(dz(1),dz(0));

    Matrix2d eigval;
    Matrix2d eigvct;

    double magnitude = sqrt(dz(1)*dz(1)+dz(0)*dz(0));

    //for a small error i want a circle, for a large one i want a ellipse
    //small error val1=val2, big error val1>val2

    double max_v=0.1;
    double min_v=0.005;

    double nv = ((min_v - max_v)/max_v)*magnitude + max_v;
    nv=nv>min_v?nv:min_v;

// 	cout<<"Nv:"<<nv<<endl;
// 	cout<<"dir:"<<dir<<endl;
    eigval << max_v, 0,
            0,  nv;

    eigvct << 1, 0,
            0, 1;

    Matrix2d rot;
    rot << cos(dir), sin(dir),
            -sin(dir), cos(dir);

    eigvct= eigvct*rot;

    Matrix2d cov = eigvct*eigval*eigvct.inverse();

    R(0,0) = cov(0,0);
    R(0,1) = cov(0,1);

    R(1,0) = cov(1,0);
    R(1,1) = cov(1,1);
}

void constant_velocity_ekfilter::makeQ()
{
    SetZero(Q,4);
    double sigma=10.;
    double factor=pow(sigma,2)*dt/6.;

    Q(0,0)=factor*2*pow(dt,2);//x x
    Q(1,1)=factor*2*pow(dt,2);//y y

    Q(0,2)=factor*3*pow(dt,1);//x vx
    Q(2,1)=factor*3*pow(dt,1);//vx v

    Q(1,3)=factor*3*pow(dt,1);//y vy
    Q(3,1)=factor*3*pow(dt,1);//vy y

    Q(2,2)=factor*6;//vx vx
    Q(3,3)=factor*6;//vy vy
}

void constant_velocity_ekfilter::makeProcess()
{
    Vector x_(x.size());

    x_(0) = x(0) + x(2)*dt;//X
    x_(1) = x(1) + x(3)*dt;//Y
    x_(2) = x(2);//Vx
    x_(3) = x(3);//Vy

    x.swap(x_);
}

void constant_velocity_ekfilter::makeMeasure()
{
    z(0)=x(0); //X
    z(1)=x(1); //Y
}

t_point::t_point()
{
    x=0;
    y=0;
    z=0;
    r=0;
    t=0;
    n=0;
    ni=0;
}

void t_point::SetZero()
{
    x=0;
    y=0;
    z=0;
    r=0;
    t=0;
    n=0;
    ni=0;
}

t_point& t_point::operator+=(const t_point &rhs)
{
    x+=rhs.x;
    y+=rhs.y;
    z+=rhs.z;
    r+=rhs.r;
    t+=rhs.t;

    return *this;
}

void t_point::Scale(double val)
{
    x*=val;
    y*=val;
    z*=val;
    r*=val;
    t*=val;
}


t_cluster::t_cluster()
{
    id=0;

// 	points.clear();
// 	associations.clear();
}

t_cluster& t_cluster::operator=(const t_cluster &rhs)
{
    id=rhs.id;
    centroid=rhs.centroid;

    t_pointPtr p;

    for(uint i=0;i<rhs.points.size();i++)
    {
        p.reset(new t_point);
        *p=*(rhs.points[i]);

        points.push_back(p);
    }

    return *this;
}

bool t_cluster::BreakPointDetector(t_pointPtr&pt,t_pointPtr&pt_m1)
{
    /**
    \todo change the clustering criterion to something more mathematically accurate
    */

    double d = sqrt( pow(pt->x-pt_m1->x,2) + pow(pt->y-pt_m1->y,2));

    if(d>2.0)
        return true;

    return false;
}

void t_cluster::CalculateCentroid(void)
{
    centroid.SetZero();

    for(uint i=0;i<points.size();i++)
        centroid+=*points[i];

    centroid.Scale(1./points.size());
}


void t_node::IncreaseAge(int increment)
{
    age+=increment;
}

void t_node::Step()
{
    Vector u(0);
    Vector x_(4);

    //Convert from eigen format to ekfilter
    Vector z_(2);
    z_(0)=z(0);
    z_(1)=z(1);

    //Makes one prediction-correction step
    estimator.step(u,z_);

    //Returns the corrected state (a posteriori state estimate)
    x_=estimator.getX();

    //Convert from ekfilter to eigen format
    x(0)=x_(0);
    x(1)=x_(1);
    x(2)=x_(2);
    x(3)=x_(3);

    //Returns the a posteriori error covariance estimate matrix
    Matrix P_=estimator.calculateP();

    //Convert from ekfilter to eigen format
    P(0,0)=P_(0,0);
    P(0,1)=P_(0,1);
    P(0,2)=P_(0,2);
    P(0,3)=P_(0,3);

    P(1,0)=P_(1,0);
    P(1,1)=P_(1,1);
    P(1,2)=P_(1,2);
    P(1,3)=P_(1,3);

    P(2,0)=P_(2,0);
    P(2,1)=P_(2,1);
    P(2,2)=P_(2,2);
    P(2,3)=P_(2,3);

    P(3,0)=P_(3,0);
    P(3,1)=P_(3,1);
    P(3,2)=P_(3,2);
    P(3,3)=P_(3,3);

    //Returns the predicted state vector (a priori state estimate)
    Vector x_p_=estimator.predict(u);

    //Convert from ekfilter to eigen format
    x_p(0)=x_p_(0);
    x_p(1)=x_p_(1);
    x_p(2)=x_p_(2);
    x_p(3)=x_p_(3);
}

void t_node::CommonConstructor()
{
    //Step on the new measurements
    Step();
}

t_node::t_node(t_cluster&cluster,long iteration)
{
    mode=NEW;

    failed_associations_counter=0;
    successful_association_counter=1;
    age=0;

    id=cluster.id;
    data_iteration=iteration;

    local_cluster=cluster;

    cluster.associations.push_back(id);

    z(0)=local_cluster.centroid.x;//x position measurement
    z(1)=local_cluster.centroid.y;//y position measurement

    Vector4d x_init;

    x_init(0)=z(0);
    x_init(1)=z(1);
    x_init(2)=0;
    x_init(3)=0;

    estimator.InitFilter(x_init);

    CommonConstructor();
}

t_node::t_node(t_node&node,t_cluster&cluster,long iteration)
{
    mode=MAIN;

    failed_associations_counter=node.failed_associations_counter;
    successful_association_counter=node.successful_association_counter+1;
    age=0;

    id=node.id;
    data_iteration=iteration;

    cluster.associations.push_back(id);
    local_cluster=cluster;

    estimator=node.estimator;//i don't know if this works

    z(0)=local_cluster.centroid.x;
    z(1)=local_cluster.centroid.y;

    CommonConstructor();
}

t_node::t_node(t_node&node,long iteration)
{
    mode=FAILED;

    failed_associations_counter=node.failed_associations_counter;
    failed_associations_counter++;
    age=0;

    successful_association_counter=node.successful_association_counter;

    id=node.id;
    data_iteration=iteration;

    local_cluster=node.local_cluster;

    estimator=node.estimator;

    //Use the previous node predicted position as measurement
    z(0)=node.x_p(0);
    z(1)=node.x_p(1);

    CommonConstructor();
}

t_node::~t_node(void)
{
// 	cout<<"Erase this shit:"<<GetName()<<endl;
}

string t_node::GetName()
{
    return "N"+ boost::lexical_cast<string>(id) + ".(" + boost::lexical_cast<string>(data_iteration) + ")";
// 	return "N"+ boost::lexical_cast<string>(id) + "." + boost::lexical_cast<string>(age) + ".(" + boost::lexical_cast<string>(data_iteration) + ")";
}

ostream& operator<< (ostream &o, const t_nodePtr &i)
{
    return o << i->GetName();

}
