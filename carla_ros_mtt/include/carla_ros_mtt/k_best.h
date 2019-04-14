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

#ifndef ROS_BRIDGE_K_BEST_H
#define ROS_BRIDGE_K_BEST_H

/**
\file
\brief K-best Murtys' algorithm header and auxiliary classes declaration
*/

// System Includes
#include <boost/shared_ptr.hpp>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <mtt/munkres.h>
#include <sys/time.h>
#include <iostream>
#include <map>
#include <vector>
#include <limits>
#include <algorithm>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class Node
{
    public:

        Node()
        {
            cost=-1;
        }

        vector<orderedPairPtr> fixed;
        vector<orderedPairPtr> excluded;
        vector<orderedPairPtr> unspecific;
        double cost;

    private:
};

typedef boost::shared_ptr<Node> NodePtr;

ostream &operator<<(ostream &o,vector<orderedPairPtr>& op);

class Assignments
{
    public:
        Assignments()
        {
            cost=-1;
        }

        vector<orderedPairPtr> pairs;
        double cost;

        friend ostream& operator<<(ostream& o, Assignments& a)
        {
            o<<"cost: "<<a.cost<<" ";
            o<<a.pairs;

            return o;
        }

    private:

};

typedef boost::shared_ptr<Assignments> AssignmentsPtr;

int factorial(int x);

class Timer
{
    public:

        //initial timer values
        map<int,timeval> timers;

        //timer status
        map<int,bool> active;

        //final values in seconds
        map<int,double> values;

        Timer()
        {
        }

        ~Timer()
        {
            timers.clear();
            active.clear();
            values.clear();
        }

        void tic(int t=0)
        {
            //get start time
            timeval start;
            gettimeofday(&start, NULL);

            //put in the timer
            timers[t] = start;
            //set active
            active[t] = true;
            //set final value to zero
            values[t] = 0;
        }

        double toc(int t=0)
        {
            //if not active return the last value
            if(!active[t])
                return values[t];

            //get the stop time
            timeval end;
            gettimeofday(&end, NULL);

            //compute final value in seconds
            values[t] = (end.tv_sec - timers[t].tv_sec); // sec
            values[t]+= (end.tv_usec - timers[t].tv_usec) / 1000000.0; // us to sec

            active[t] = false;

            return values[t];
        }

        double toMiliseconds(int t=0)
        {
            return values[t]*1000.;
        }

        double value(int t=0)
        {
            return values[t];
        }
};

vector<vector<int> > convertToStdVector(MatrixXd&mat);
double munkers_wrapper(MatrixXd&cost_mat,vector<orderedPairPtr>&assignments);
vector<AssignmentsPtr> k_best_assignment(MatrixXd& cost_mat,uint k);
vector<NodePtr> partitionNode(Node&node_in,MatrixXd&cost_mat);
NodePtr calcNodeCost(Node&node_in,MatrixXd&cost_mat,bool non_square_matrix=false);
template<class type>
uint countRepeatingValues(vector<type> sorted_values);
vector<int> getRows(vector<orderedPairPtr>& pairs);
vector<int> getCols(vector<orderedPairPtr>& pairs);
bool compareOrdered_pair(orderedPairPtr& p1,orderedPairPtr& p2);
ostream &operator<<(ostream &o,vector<AssignmentsPtr>& assing);
ostream &operator<<(ostream &o,NodePtr& n);
ostream &operator<<(ostream &o,Node& n);

#endif //ROS_BRIDGE_K_BEST_H
