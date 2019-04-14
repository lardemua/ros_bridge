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

#ifndef ROS_BRIDGE_MUNKRES_H
#define ROS_BRIDGE_MUNKRES_H

// System Includes
#include <mtt/matrix.h>
#include <cmath>
#include <list>
#include <utility>
#include <vector>
//#include <eigen3/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <boost/shared_ptr.hpp>

using Eigen::MatrixXd;

using namespace std;

typedef struct
{
    int row, col;

}orderedPair;

typedef boost::shared_ptr<orderedPair> orderedPairPtr;

class Munkres {
    public:
        void solve(Matrix<double> &m);
        double solve(MatrixXd& m_in,vector<orderedPairPtr>& results);

    private:
        static const int NORMAL = 0;
        static const int STAR = 1;
        static const int PRIME = 2;
        inline bool find_uncovered_in_matrix(double,int&,int&);
        inline bool pair_in_list(const std::pair<int,int> &, const std::list<std::pair<int,int> > &);
        int step1(void);
        int step2(void);
        int step3(void);
        int step4(void);
        int step5(void);
        int step6(void);
        Matrix<int> mask_matrix;
        Matrix<double> matrix;
        bool *row_mask;
        bool *col_mask;
        int saverow, savecol;
};

#endif //ROS_BRIDGE_MUNKRES_H
