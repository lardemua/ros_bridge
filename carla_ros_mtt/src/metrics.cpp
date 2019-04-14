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
\brief Distance metrics source code
*/

// System Includes
#include <carla_ros_mtt/metrics.h>

double mahalanobis(Vector2d&y,Vector2d&mean,Matrix2d& cov)
{
    VectorXd a=y-mean;
    MatrixXd i= cov.inverse();
    double squared=a.transpose()*i*a;
    return sqrt(squared);
}

double biVariatePDF(Vector2d& x,Vector2d& m,Matrix2d& cov)
{
    double a,b,c,d,e;
    double s1,s2,rho;

    s1=sqrt(cov(0,0));
    s2=sqrt(cov(1,1));
    rho=cov(0,1)/(s1*s2);

    a=1./(2*M_PI*s1*s2*sqrt(1-rho*rho));
    b=1./(2*(1-rho*rho));
    c=pow(x(0)-m(0),2)/(s1*s1);
    d=pow(x(1)-m(1),2)/(s2*s2);
    e=(2*rho*(x(0)-m(0))*(x(1)-m(1)))/(s1*s2);

    double f = a*exp(-b*(c+d-e));

    if(f<1e-12)
        f=0;

    return f;
}
