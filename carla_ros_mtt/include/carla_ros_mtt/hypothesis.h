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

#ifndef ROS_BRIDGE_HYPOTHESIS_H
#define ROS_BRIDGE_HYPOTHESIS_H

/**
\file
\brief Hypothesis class declaration
*/

// System Includes
#include <mtt/target.h>

class Target;
///Shared pointer to the Target class
typedef boost::shared_ptr<Target> TargetPtr;

///Possible hypothesis status
enum hypotheses_status {NORMAL=0, MOVED, FORCED_PARENT, PARENT, DEAD, DEAD_FORCED_PARENT,ERROR};

/**
\brief Hypotheses tree nodes
This class is used as the main node for the hypotheses tree.
Each hypothesis is a set of possible targets, a group of id values and a probability associated with it.
*/
class Hypothesis
{
    public:
        ///Auxiliary int variable, multi use, value not assured between calls
        int _aux1;
        ///Id of the cluster this hypothesis belongs to
        int _cluster;
        ///Iteration for this hypothesis
        int _iteration;
        ///Main id of the hypothesis
        long _id;
        ///Current status
        int _status;
        ///Static extremely unique id for all hypotheses (since program start)
        static long _euid;//extremely unique id
        ///Unique id for all currently existing hypotheses
        long _uid;
        ///Hypothesis probability
        double _probability;
        ///Number of detected targets
        int _n_det;
        ///Number of occluded targets
        int _n_occ;
        ///Number of deleted targets
        int _n_del;
        ///Number of new targets
        int _n_new;
        ///Number of failed targets
        int _n_fal;
        ///Probability of current target to measurement assignment
        double _prod;

        friend ostream& operator<<(ostream& o,Hypothesis& h);

        ///Id of the parent hypothesis
        long _parent_uid;

        ///Attribute names used in the graphviz representation of this hypothesis
        vector<string> _attribute_names;
        ///Values for the corresponding attributes
        vector<string> _attribute_values;

        ///List of hypothesis targets
        vector<TargetPtr> _targets;

        /**
         * \brief Create hypothesis name formated to the user interface
         *
         * \return hypothesis UI name
         */
        string nameUI();

        /**
         * \brief Create hypothesis name
         *
         * \return hypothesis name
         */
        string name();

        /**
         * \brief Add a new attribute to the hypothesis, these attributes are only used in the graphviz plugin
         *
         * \param name name of the new attribute
         * \param value value of the corresponding attribute
         */
        void setAttribute(string name,string value);

        /**
         * \brief Hypothesis constructor, variable initialization
         */
        Hypothesis();

        /**
         * \brief Hypothesis constructor, specification of iteration cluster and status
         *
         * \param iteration of this new hypothesis
         * \param cluster of this new hypothesis
         * \param status of the hypothesis
         */
        Hypothesis(int iteration, int cluster, int status);

        /**
         * \brief Class destructor
         */
        ~Hypothesis();
};

///Shared pointer to the Hypothesis class
typedef boost::shared_ptr<Hypothesis> HypothesisPtr;


/**
 * \brief Compare two hypotheses by uid
 *
 * \param h1 first hypothesis
 * \param h2 second hypothesis
 *
 * \return true if h2 uid is larger that h1 uid
 */
bool compareHypotheses(HypothesisPtr h1, HypothesisPtr h2);

/**
 * \brief Compare two hypotheses by probability
 *
 * \param h1 first hypothesis
 * \param h2 second hypothesis
 *
 * \return true if h2 probability is larger that h1 probability
 */
bool compareHypothesesByProbability(HypothesisPtr h1, HypothesisPtr h2);

/**
 * \brief Compare two hypotheses by probability, but descending
 *
 * \param h1 first hypothesis
 * \param h2 second hypothesis
 *
 * \return true if h2 probability is smaller that h1 probability
 */
bool compareHypothesesByProbabilityDescending(HypothesisPtr h1, HypothesisPtr h2);


#endif //ROS_BRIDGE_HYPOTHESIS_H
