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

#ifndef ROS_BRIDGE_MHT_DECLARATION_H
#define ROS_BRIDGE_MHT_DECLARATION_H

/**
\file
\brief MHT class declaration
*/

// System Includes
#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <mtt/hypothesisTree.h>
#include <mtt/k_best.h>
#include <boost/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <colormap/colormap.h>
#include <boost/thread.hpp>
#include <mtt/metrics.h>

using namespace std;

/**
 * \brief Create a ellipse using parameters
 *
 * \param cov covariance
 * \param mu mean
 * \param MahThreshold maximum Mahalanobis distance
 * \return matrix of ellipse points
 */
MatrixXd ellipseParametric(Matrix2d& cov,Vector2d& mu,double MahThreshold);

extern string fileName;

template<class T>
bool removeLocal(vector<T>& v, T i)
{
    typename vector<T>::iterator it;

    it=find(v.begin(),v.end(),i);

    if(it==v.end())
        return false;

    v.erase(it);
    return true;
}

/**
 * \brief Color class, extension to std_msgs::ColorRGBA
 *
 * This class allows to change the brightness of the color and output it has a string.
 *
 */
class Color: public std_msgs::ColorRGBA
{
    public:

        /**
         * \brief Class constructor, needs to be created from a std_msgs::ColorRGBA
         *
         * \param c std_msgs::ColorRGBA
         */
        Color(std_msgs::ColorRGBA c);

        /**
         * \brief Change the brightness of a color
         *
         * \param correctionFactor if <1 makes a color darker, if >1 makes a color lighter, may change the color tone
         *
         * \return a pointer to the own class
         */
        Color& changeColorBrightness(double correctionFactor);

        /**
         * \brief Output a html style string color code
         *
         * \return the html code
         */
        string str();

        /**
         * \brief prints the color in 0-1 range format
         */
        ostream& operator<<(ostream&);
};

ostream& operator<<(ostream& o,vector<vector<int> >& measurements_assignemnt);

/**
\brief Big and complicated class that implements the MHT (Multiple Hypotheses Tracking) algorithm
Create empty tree \n
Obtain new measurements
for each leaf \n
	Gate measurements with tree leafs \n
	Add gated measurements as leaf children \n
	If no association was created iterate on void \n
		Remove branch if needed \n
end for
for each measurement \n
	if no associations \n
		add new tree root node \n
end for
common iteration processing \n
tree pruning \n
	nth pruning //delayed decision \n
	age deletion //careful only works if age is bigger that nth pruning
*/
class Mht
{
    public:

        ///Vector of current existing hypotheses clusters
        vector<ClusterPtr> _clusters;
        string _aux1;

        /**
        \brief Mht constructor
        */
        Mht(void);

        /**
        \brief Mht destructor
        */
        ~Mht(void);

        ///Return the maximum Mahalanobis gating distance
        double getMaxGating(void);

        /**
        \brief Go through all clusters, all hypotheses and finally all targets, removing target to measurement associations from past targets
        */
        void clearTargetAssociations(void);

        void simplifySingleTargetClusters(vector<MeasurementPtr>& global_measurements);

        void solveCluster(ClusterPtr cluster);

        /**
        \brief Currently the main mht function that does all the work
        This function starts by going through all the current leafs on the tree, for each leaf/hypothesis go through all
        targets and for each target check which measurements fall within its gate and add those measurements to the hypothesis cluster.
        This is called measurement to cluster assignment, the end result is the cluster_assignemnt map.
        The following step checks if measurements belong to more that one cluster and merges those clusters, if a measurement is
        shared this means that both clusters should be evaluated simultaneously. The merging of clusters is complicated: the first
        step is to define which cluster name will remain and which will disappear, I choose the first assigned to the measurement
        (for no particular reason). Then we go through all clusters associated with the measurement changing the associated hypothesis
        to the new cluster and all measurements associated with this cluster changing their association as well.
        In the end all clusters have different measurements without any overlap.
        The next step is to create the list of problems to solve, each problem is a association of all the measurements of that cluster
        with the hypothesis that the cluster contains. This problems will be solved using the Murty algorithm with the Hungarian as a lower
        layer.
        There are still a few steps missing ...
        In the end, all measurements that were not associated with any cluster will create new clusters with a single hypothesis
        and a single target. This step makes the global tree grow.
        \param measurements input vector of measurements
        */
        void iterate(vector<MeasurementPtr>& measurements);

        hypothesisTreePtr getHypothesisTree(void);

        vector<TargetPtr> getTargets(void);

        void clear(void);

        string getReportName(string extra);

        void solveClusterMultiTread(ClusterPtr cluster);

        void solveHypothesis(HypothesisPtr parent_hypothesis,vector<MeasurementPtr>& measurements, vector<HypothesisPtr>& list_of_hypotheses);

        ///K parameter used in the Murty's algorithm, must be configured form outside class
        int _k;
        ///J parameter used in the propagation of hypotheses, must be configured form outside class
        int _j;

    private:

        ///Main tree holding variable
        hypothesisTreePtr _hypothesisTree;

        ///Leaf auxiliary iterator
        HypothesisTree::leaf_iterator _hleaf;
        ///Simple iterator
        HypothesisTree::iterator _iterator;

        ///Vector to store outgoing targets
        vector<TargetPtr> _current_targets;

        ///Current iteration
        long iteration;
        ///Maximum gating threshold
        double _max_gating;
        ///Maximum number of miss associations
        int _miss_association_threshold;
        ///Minimum representativity value in propagating hypotheses
        double _minimum_representativity;

        ///Number of previous state positions to store
        uint _history_size;
        ///Flag that determines if dead hypotheses are to be deleted, used in graphviz
        bool _remove_unused;
        ///Flag that determines if the main parents are to be tagged, used in graphviz
        bool _parent_tagging;
        ///Flag that controls the use of the minimum representativity rule
        bool _use_minimum_representativity;
        ///Flag that controls the removal of empty clusters
        bool _remove_empty_clusters;
        ///Flag that controls the removal of old hypotheses, with changes to the tree structure
        bool _clean_old_modifying_tree;
        ///Cluster id seed
        long _cluster_id;
        ///Remove the hypothesis and possible parent if empty
        void recursiveHypothesisRemove(HypothesisTree::iterator it);
        ///Remove branches that were not propagated and old hypotheses
        void removeIrrelevant(void);
        ///Remove dead hypotheses
        void removeTheDead(void);

        /**
         * \brief Complete deletion of a cluster
         * \param id of the cluster to remove
         * \return cluster vector iterator
         */
        vector<ClusterPtr>::iterator completeRemoveCluster(long id);

        /**
        \brief Remove the cluster with a given id along with all the hypotheses associated with it
        */
        vector<ClusterPtr>::iterator completeRemoveCluster(vector<ClusterPtr>::iterator it_cluster);

        MeasurementPtr findMeasurement(vector<MeasurementPtr>& measurements, long id);

        /**
        \brief Change hypothesis color based on its current cluster
        This functions checks the cluster id of the hypothesis and calculates the corresponding color by doing cluster_id mod 8.\n
        The result will be the color id.
        \param hypothesis the working node
        */
        void switchColor(HypothesisPtr hypothesis);

        /**
        \brief Find all hypotheses with a given cluster id
        This function will find all hypotheses that are currently leafs of the main tree with a given cluster.
        This function will create a std::vector of the HypothesisPtr and return it.
        \warning Only LEAF nodes will be returned! Not the whole branch!
        \param cl cluster id
        \param it_past select the iteration from which the hypotheses will be extracted (default 1, previous iteration)
        \return vector of hypothesisPtr
        */
        vector<HypothesisPtr> findHypotheses(int cl,int it_past = 1);

        ClusterPtr findCluster(vector<ClusterPtr>& clusters, int id);

        void createClusters(void);

        /**
        \brief Distribute the measurements by the existing clusters
        */
        void clusterAssignemnt(vector<MeasurementPtr>& measurements);

        /**
        \brief Remove empty clusters
        */
        void cleanClusters(void);

        void removeEmptyClusters(void);

        bool consistencyCheck(void);

        void breakClusters(vector<MeasurementPtr>& measurements);

        HypothesisTree::iterator insertForcedParent(long cluster);

        vector<TargetPtr> createCopy(vector<TargetPtr>& original);

        /**
        \brief Check if clusters need merging and do it!!
        This is quite complicated, merging involves a total reevaluation of targets, measurements and probabilities.
        */
        void clusterMerger(vector<MeasurementPtr>& measurements);

        TargetPtr associate(TargetPtr& target,MeasurementPtr& measurement);

        bool normalize(vector<HypothesisPtr>& hypotheses,uint j=10e3);

        void getProbability(HypothesisPtr hypothesis);

        double getProbability(uint n_det, uint n_occ, uint n_del, uint n_new, uint n_fal, double prod, double previous);

        HypothesisPtr getHypothesisPtr(long id,ClusterPtr& cluster);

        void updateTargetList(void);

        void checkCurrentTargets(vector<MeasurementPtr>& measurements);
};


#endif //ROS_BRIDGE_MHT_DECLARATION_H
