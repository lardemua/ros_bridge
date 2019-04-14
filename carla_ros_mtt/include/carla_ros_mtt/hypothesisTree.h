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

#ifndef ROS_BRIDGE_HYPOTHESISTREE_H
#define ROS_BRIDGE_HYPOTHESISTREE_H

/**
\file
\brief Hypotheses tree class declaration
*/

// System Includes
#include <algorithm>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <mtt/tree.h>
#include <mtt/tree_util.h>
#include <mtt/hypothesis.h>

/**
\brief Hypotheses tree
This in the main hypotheses tree, each node corresponds to a different hypothesis, this class is a inheritance of the tree class
using the HypothesisPtr has the node type.
Only a few additional functions are added to the basic tree.
*/
class HypothesisTree: public tree<HypothesisPtr>
{
    public:

        ///Multi tread mutex to regulate access during drawing operations
        pthread_mutex_t _draw_mutex;

        ///Constructor
        HypothesisTree();
};

///Shared pointer to the HypothesisTree class
typedef boost::shared_ptr<HypothesisTree> hypothesisTreePtr;

#endif //ROS_BRIDGE_HYPOTHESISTREE_H
