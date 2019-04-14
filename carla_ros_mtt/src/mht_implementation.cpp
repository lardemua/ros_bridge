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
\brief MHT class source code
*/

// System Includes
#include <carla_ros_mtt/mtt_declaration.h>

ostream null_stream(NULL);

ostream& ldb(int level=0)
{
    if(level==0)
        return null_stream;
    if(level==1)
        return cout;

    return cout;
}

MatrixXd ellipseParametric(Matrix2d& cov,Vector2d& mu,double MahThreshold)
{
// 	cout<<"Cov: "<<cov<<endl;
// 	cout<<"Mu: "<<mu<<endl;
// 	cout<<"Thres: "<<MahThreshold<<endl;

    //How to calculate Major and Minor axes and rotation from Covariance?

    //Check if inputs are valid
    assert(is_finite(mu));
    assert(is_finite(cov));

    //First calculate the eigen values and vectors
    Eigen::SelfAdjointEigenSolver<Matrix2d> eigensolver(cov);
    if(eigensolver.info() != Eigen::Success)
        abort();

    Matrix2d eigvectors=eigensolver.eigenvectors();

    Vector2d ev1=eigvectors.row(0);
    Vector2d ev2=eigvectors.row(1);

    //Use eigenvectors directions to find the major and minor axes
    Vector2d p;

    //Step in the outgoing direction
    double k_step=0.2;
    //Stepping value
    double k=0;
    //Current value for the Mahalanobis distance
    double d;
    //Major and minor axes of the ellipse
    double ea,eb;
    //Angle of the ellipse
    double g;

    //Calc a point along the main direction and get its distance
    do
    {
        p=k*ev1+mu;
        k+=k_step;

        d=mahalanobis(p,mu,cov);

        if(MahThreshold-d < 0.5)
            k_step=0.01;

    }while(d<MahThreshold);

    k_step=0.2;
    ea=k;

    k=0;
    do
    {
        p=k*ev2+mu;
        k+=k_step;

        d=mahalanobis(p,mu,cov);

        if(MahThreshold-d < 0.5)
            k_step=0.01;

    }while(d<MahThreshold);

    eb=k;

    g=atan2(ev1(1),ev1(0));

    //Angular step for the parametric curve
    double t_step=(2*M_PI)/20;
// 	double t_step=0.02;
    //Maximum angle for the parametric curve
    double t_max=2*M_PI+t_step;
    //Total number of nodes of the parametric curve
    double steps=ceil(t_max/t_step);

    MatrixXd el(2,steps);

    for(uint i=0;i<steps;i++)
    {
        double t=i*t_step;
        el(0,i)=mu(0)+ea*cos(t)*cos(g)-eb*sin(t)*sin(g);
        el(1,i)=mu(1)+ea*cos(t)*sin(g)+eb*sin(t)*cos(g);
    }

    return el;
}

Color::Color(std_msgs::ColorRGBA c)
{
    r=c.r;
    g=c.g;
    b=c.b;
    a=c.a;
}

Color& Color::changeColorBrightness(double correctionFactor)
{
    if (correctionFactor < 0)
    {
        correctionFactor = 1 + correctionFactor;
        r *= correctionFactor;
        g *= correctionFactor;
        b *= correctionFactor;
    }
    else
    {
        r = (1. - r) * correctionFactor + r;
        g = (1. - g) * correctionFactor + g;
        b = (1. - b) * correctionFactor + b;
    }

    if(r>1)
        r=1;

    if(g>1)
        g=1;

    if(b>1)
        b=1;

    return *this;
}

string Color::str()
{
    boost::format fm("#%02x%02x%02x");
    fm % (int)(r*255) % (int)(g*255) % (int)(b*255);
    return fm.str();
}


ostream& Color::operator<<(ostream& o)
{
    o<<"rbg("<<this->r<<","<<this->g<<","<<this->b<<")";
    return o;
}

Mht::Mht(void)
{
// 	_max_gating = 5.0;
    _max_gating = 6.0;
// 	_max_gating = 8.0;
// 	_max_gating = 7.0;
// 	_max_gating = 10.0;
// 	_max_gating = 20.0;

// 			_max_gating = 2.5;
// 	_miss_association_threshold = 5;
// 	_miss_association_threshold = 10;
// 	_miss_association_threshold = 50;
// 	_miss_association_threshold = 50;
    _miss_association_threshold = 80;

    _hypothesisTree.reset(new HypothesisTree);

    _k = -1;
    _j = -1;

    //GNN
// 	_k = 1; //uptodate 0-9
// 	_j = 1;

    //MHT
// 	_k = 2;//uptodate
// 	_j = 3;

    //MHT
// 			_k = 2;
// 			_j = 3;

// 	_history_size = 4;
    _history_size = 6;

// 			_minimum_representativity = 0.98;
    _minimum_representativity = 1.0;

    _remove_unused = true;
    _parent_tagging = true;
    _use_minimum_representativity = true;
    _remove_empty_clusters = true;
    _clean_old_modifying_tree = true;//not yet coded
    iteration = 0;
    _cluster_id = 0;
}

Mht::~Mht(void)
{
}

double Mht::getMaxGating(void)
{
    return _max_gating;
}

void Mht::clearTargetAssociations(void)
{
    //Go through all clusters, all hypotheses and finally all targets, removing target to measurement associations from past targets
    for(uint c=0;c<_clusters.size();c++)
    {
        ClusterPtr cluster = _clusters[c];

        for(uint h=0;h<cluster->assigned_hypotheses.size();h++)
        {
            HypothesisPtr hypothesis = cluster->assigned_hypotheses[h];

            for(uint t=0;t<hypothesis->_targets.size();t++)
            {
                TargetPtr target = hypothesis->_targets[t];
                target->cleanTargetAssociations();
            }
        }
    }
}

void Mht::simplifySingleTargetClusters(vector<MeasurementPtr>& global_measurements)
{
    for(uint c=0;c<_clusters.size();c++)
    {
        ClusterPtr cluster = _clusters[c];

// 				vector<HypothesisPtr> hypotheses = cluster->assigned_hypotheses;

        //Get the measurements vector
        vector<MeasurementPtr> measurements;

        for(uint m=0;m<cluster->assigned_measurements.size();m++)
        {
            //Find the measurement with the right id
// 					MeasurementPtr measurement = findMeasurement(global_measurements,cluster->assigned_measurements[m]);

            //Put it in the measurments vector
            measurements.push_back(cluster->assigned_measurements[m]);
        }

        bool single_target = true;

        for(vector<HypothesisPtr>::iterator it=cluster->assigned_hypotheses.begin();it!=cluster->assigned_hypotheses.end();it++)
            if((*it)->_targets.size()>1)
                single_target=false;

// 				cout<<"smp single target: "<<single_target<<" cluster: "<<cluster->id<<endl;

        if(measurements.size() == 1 && cluster->assigned_hypotheses.size() > 1 && single_target)
        {

// 					cout<<"smp simplify"<<endl;

            HypothesisPtr parent_hypothesis = cluster->assigned_hypotheses[0];

            for(vector<HypothesisPtr>::iterator it=cluster->assigned_hypotheses.begin()+1;it!=cluster->assigned_hypotheses.end();)
            {
                HypothesisPtr aux_hypothesis = *it;

// 						parent_hypothesis->_targets.insert(parent_hypothesis->_targets.begin(),aux_hypothesis->_targets.begin(),aux_hypothesis->_targets.end());

                aux_hypothesis->_targets.clear();
                aux_hypothesis->_status=DEAD;

                HypothesisTree::iterator ith = find(_hypothesisTree->begin(),_hypothesisTree->end(),aux_hypothesis);

                recursiveHypothesisRemove(ith);

// 						cout<<"mer erased: "<<**it<<endl;
                it=cluster->assigned_hypotheses.erase(it);
            }

            assert(parent_hypothesis->_targets.size()>0);

            for(uint t=0;t<parent_hypothesis->_targets.size();t++)
            {
                parent_hypothesis->_targets[t]->_hypothesis = parent_hypothesis->_uid;
                parent_hypothesis->_targets[t]->_variant="";
            }
        }
    }
}

void Mht::solveHypothesis(HypothesisPtr parent_hypothesis,vector<MeasurementPtr>& measurements, vector<HypothesisPtr>& list_of_hypotheses)
{
    cout<<"solve hypo stx"<<endl;
    static boost::mutex mtx;

    vector<TargetPtr> targets = parent_hypothesis->_targets;
    assert(targets.size()>0);

    if(parent_hypothesis->_status==DEAD)
        return;

    //Get list of targets associated with this hypotheses

    //Another trivial hypothesis, just one association possible in this hypothesis
    if(targets.size()==1 && measurements.size()==1)
    {
//		cout<<"trivail version 2"<<endl;
        //create a single hypothesis from this one
        HypothesisPtr hypothesis(new Hypothesis);

        hypothesis->_parent_uid = parent_hypothesis->_uid;
        hypothesis->_iteration=iteration;
        hypothesis->_cluster=parent_hypothesis->_cluster;//

        hypothesis->_n_det=1;//no detected target
        cout<<"get dist"<<endl;
        hypothesis->_prod = parent_hypothesis->_targets[0]->getDistance(measurements[0],1);//this will calculate the bivariate pdf

        TargetPtr new_target = associate(targets[0],measurements[0]);

        if(new_target!=NULL)
        {
            new_target->_hypothesis = hypothesis->_uid;
            new_target->_cluster = hypothesis->_cluster;

            if(is_finite(new_target->_xp))
                hypothesis->_targets.push_back(new_target);//Add the new target to the new hypothesis

            //Add this hypothesis to the list of hypothesis for this problem if the association was successful
            if(hypothesis->_targets.size()!=0)
            {
                mtx.lock();
                list_of_hypotheses.push_back(hypothesis);
                mtx.unlock();
            }
        }else
        {
            const char COL_RESET[] = "\x1b[0m";
            const char RED[]       = "\x1b[31m";
            cout << RED << "Error!!, Major" << COL_RESET << endl;
            cout<<"failed a trivial association"<<endl;
        }

        return;
    }

    if(measurements.size()==0)
    {
        //All the targets will correspond to miss associations

        //Create new hypothesis
        HypothesisPtr hypothesis(new Hypothesis);

        hypothesis->_parent_uid = parent_hypothesis->_uid;
        hypothesis->_iteration=iteration;
        hypothesis->_cluster=parent_hypothesis->_cluster;

        cout<<"c2"<<endl;

        for(uint t=0;t<targets.size();t++)
        {
            if(targets[t]->_missed_associations < _miss_association_threshold)
            {
                //Iterate over failed association
                cout<<"iterate over failed"<<endl;
                TargetPtr new_target(new Target(targets[t]));

                new_target->_hypothesis=hypothesis->_uid;
                new_target->_cluster=hypothesis->_cluster;

                if(is_finite(new_target->_xp))
                {
                    hypothesis->_targets.push_back(new_target);
                    hypothesis->_n_occ++;//increment deleted target counter
                }

                cout<<"done iterate over failed"<<endl;
            }else
            {
                hypothesis->_n_del++;//increment deleted target counter
                //Do not add this target to the new hypothesis
            }
        }

        //Add this hypothesis to the list of hypothesis for this problem
        if(hypothesis->_targets.size()>0)
        {
            mtx.lock();
            list_of_hypotheses.push_back(hypothesis);
            mtx.unlock();
        }

        return;
    }

    cout<<"Matrixxd"<<endl;

    //Define the cost matrix
    MatrixXd probability(measurements.size(),targets.size());

    TargetPtr empty;
    //Calculate the measurement to target association cost
    for(uint it=0;it<targets.size();++it)//For all targets
    {
        assert(targets[it]!=empty);
        for(uint im=0;im<measurements.size();++im)//For all measurements
        {
            probability(im,it)=targets[it]->getDistance(measurements[im],1);//this will calculate the bivariate pdf
// 							cout<<"p: "<<probability(im,it)<<endl;
        }
    }

    uint nT = targets.size();
    uint nM = measurements.size();

    //Convert the probability matrix to a cost matrix using -log()
    MatrixXd cost(probability.rows(),probability.cols());

    for(uint r=0;r<cost.rows();++r)
        for(uint c=0;c<cost.cols();++c)
        {
            cost(r,c)=-log(probability(r,c));
            if(std::isinf(cost(r,c)))
                cost(r,c)=1e6;
        }

// 					cout<<"\n\tprobability: "<<probability<<endl;
// 					cout<<"\tcost: "<<cost<<endl<<endl;

    //Now that we have the cost matrix ready, we must apply the Murty algorithm k_best_assignment()

// 					cout<<"solve k-best, cluster: "<<cluster->id<<endl;
// 					cout<<"ntarters: "<<nT<<" nmeasurements: "<<nM<<endl;
    assert(_k!=-1);
    vector<AssignmentsPtr> assignments = k_best_assignment(cost,_k);
// 					cout<<"done"<<endl;
// 					cout<<"Assignments:\n"<<assignments<<endl;

    //Handle the new assignments, create a new target if the measurement is associated with an invalid target (it>=nT)
    //Increment invalid associations if this target is associated with an invalid measurement (im>=nM)
    //For all k or less solutions from the Murty
    for(uint i=0;i<assignments.size();++i)
    {
        //Get current assignment (just to simplify the code)
        vector<orderedPairPtr> assignment = assignments[i]->pairs;

        //Create new hypothesis
        HypothesisPtr hypothesis(new Hypothesis);

        hypothesis->_parent_uid = parent_hypothesis->_uid;
        hypothesis->_iteration=iteration;
        hypothesis->_cluster=parent_hypothesis->_cluster;

        //Now i must check if there are unassigned measurements or targets
        vector<int> mes(nM,0);
        vector<int> tar(nT,0);

        for(uint e=0;e<assignment.size();++e)
        {
            mes[assignment[e]->row]=1;
            tar[assignment[e]->col]=1;
        }

        if(nM>nT)//Unassigned measurements
        {
            //Now i must check which measurements were not assigned and create new targets with those measurements

            for(uint im=0;im<mes.size();++im)
                if(mes[im]==0)//Measurement not assigned
                {
                    //New target
                    TargetPtr new_target(new Target(measurements[im]));

                    new_target->_hypothesis=hypothesis->_uid;
                    new_target->_cluster=hypothesis->_cluster;

                    if(is_finite(new_target->_xp))
                    {
                        hypothesis->_n_new++;//increment new target counter
                        hypothesis->_targets.push_back(new_target);
                    }
                }

        }else if(nT>nM)//Unassigned targets
        {
            for(uint it=0;it<tar.size();++it)
                if(tar[it]==0)//Target not assigned
                {
                    if(targets[it]->_missed_associations < _miss_association_threshold)
                    {
                        //Iterate over failed association
                        TargetPtr new_target(new Target(targets[it]));

                        new_target->_hypothesis=hypothesis->_uid;
                        new_target->_cluster=hypothesis->_cluster;

                        if(is_finite(new_target->_xp))
                        {
                            hypothesis->_targets.push_back(new_target);
                            hypothesis->_n_occ++;//increment deleted target counter
                        }

                    }else
                    {
                        hypothesis->_n_del++;//increment deleted target counter
                        //Do not add this target to the new hypothesis
                    }
                }
        }

        //Create new targets from associations
        for(uint p=0;p<assignment.size();++p)
        {
            uint im = assignment[p]->row;
            uint it = assignment[p]->col;

            TargetPtr new_target = associate(targets[it],measurements[im]);

            if(new_target!=NULL)
            {
                new_target->_hypothesis=hypothesis->_uid;
                new_target->_cluster=hypothesis->_cluster;

                if(is_finite(new_target->_xp))
                {
                    hypothesis->_targets.push_back(new_target);
                    hypothesis->_n_det++;//increment deleted target counter
                    hypothesis->_prod*=targets[it]->getDistance(measurements[im],1);//using the bivariate pdf

                    if(new_target->_missed_associations>0)//If the new targets is the result of miss association
                        hypothesis->_n_occ++;//increment number of occulded targets
                }
            }else
            {
                hypothesis->_n_del++;//increment deleted target counter
            }
        }

        //Add this hypothesis to the list of hypothesis for this problem
        if(hypothesis->_targets.size()>0)
        {
            mtx.lock();
            list_of_hypotheses.push_back(hypothesis);
            mtx.unlock();
        }
    }

    cout<<"solve hypo enx"<<endl;
}

void Mht::solveClusterMultiTread(ClusterPtr cluster)
{
    vector<HypothesisPtr> list_of_hypotheses;

    //Get the measurements vector
    vector<MeasurementPtr> measurements = cluster->assigned_measurements;
    vector<HypothesisPtr> hypotheses = cluster->assigned_hypotheses;

    assert(hypotheses.size()>0);

    cout<<"start"<<endl;
    //Check if this problem is trivial, 1 measurement with 1 hypothesis and 1 target
    if(measurements.size() == 1 && hypotheses.size() == 1 && hypotheses[0]->_targets.size()==1)
    {
        cout<<"trivial problem"<<endl;
        HypothesisPtr parent_hypothesis = hypotheses[0];

        //If this problem is trivial no Murty algorithm is needed and only one hypothesis exists
        //The association is strait forward, and we only need to branch once the previous hypothesis

        //This problem will surely have a solution because if we are here the association is within the limits
        //The probability of this hypothesis is 1, because of normalization factors we don't need to calculate it

        //Create new hypothesis
        HypothesisPtr hypothesis(new Hypothesis);

// 				hypothesis->_parent = parent_hypothesis;
        hypothesis->_parent_uid = parent_hypothesis->_uid;
        hypothesis->_iteration = iteration;
        hypothesis->_cluster = parent_hypothesis->_cluster;//The new hypothesis belongs to the same cluster has the old one
        hypothesis->_n_det = 1;//no detected target
        hypothesis->_prod = parent_hypothesis->_targets[0]->getDistance(measurements[0],1);//this will calculate the bivariate pdf
        //the rest is set to zero, nocc, ndel, nfal

        //Associate the target 0 with measurement 0 of the current hypothesis
        TargetPtr new_target = associate(parent_hypothesis->_targets[0],measurements[0]);
        //The previous association must occur

        cout<<"cenas"<<endl;

        if(new_target!=NULL)
        {
            new_target->_hypothesis = hypothesis->_uid;
            new_target->_cluster = hypothesis->_cluster;

            if(is_finite(new_target->_xp))
                hypothesis->_targets.push_back(new_target);

            //Add this hypothesis to the list of hypotheses for this problem
            if(hypothesis->_targets.size()>0)
                list_of_hypotheses.push_back(hypothesis);
        }else
        {
            const char COL_RESET[] = "\x1b[0m";
            const char RED[]       = "\x1b[31m";
            cout << RED << "Error!!, Major" << COL_RESET << endl;
            cout<<"failed a trivial association"<<endl;
        }

    }else//Not trivial problem
    {
        cout<<"non trivial"<<endl;
        //For all hypotheses in this problem

        //Create lots of threads

        vector<boost::thread*> threads;

        for(uint h=0;h<hypotheses.size();++h)
        {
            boost::thread*thr = new boost::thread(boost::bind(&Mht::solveHypothesis,this,boost::ref(hypotheses[h]),boost::ref(measurements),boost::ref(list_of_hypotheses)));
            threads.push_back(thr);
        }

        for(uint c=0;c<threads.size();c++)
        {
            threads[c]->join();
            delete threads[c];
        }
    }

    cout<<"here"<<endl;
    //Calculate the probabilities of all the child hypotheses
    for(uint i=0;i<list_of_hypotheses.size();++i)
    {
        HypothesisPtr h = list_of_hypotheses[i];
        getProbability(h);
    }

    //If we choose to wait to do a reduce branching at this point all the k-best solutions to all the previous
    //hypotheses should be ranked and the j-best of them should be extracted and propagated
    sort(list_of_hypotheses.begin(),list_of_hypotheses.end(),compareHypothesesByProbabilityDescending);

    //Normalize only the j hypotheses
    assert(_j!=-1);
    normalize(list_of_hypotheses,_j);

    double probability_sum = 0;

    //Now we have the list of all k-best hypotheses to solve this problem
    //I will now find only the j best
    for(uint i=0;i<(uint)_j && i<list_of_hypotheses.size();++i)
    {
        HypothesisPtr hypothesis = list_of_hypotheses[i];
        HypothesisTree::iterator it;

        for(uint t=0;t<hypothesis->_targets.size();t++)
        {
            TargetPtr target = hypothesis->_targets[t];
            if(!is_finite(target->_xp) || !is_finite(target->_P))
            {
                cout<<"trying to add invalid target, iteration:  "<<iteration<<endl;
                cout<<*target<<endl;
                cout<<*hypothesis<<endl;
                cout<<target->_xp<<endl;
            }

            assert(is_finite(target->_xp));
            assert(is_finite(target->_P));
        }

        for(HypothesisTree::iterator fi=_hypothesisTree->begin();fi!=_hypothesisTree->end();fi++)
            if((*fi)->_uid==hypothesis->_parent_uid)
            {
                it=fi;
                break;
            }

        _hypothesisTree->append_child(it,hypothesis);//Append a new node

        cluster->assigned_hypotheses.push_back(hypothesis);

        probability_sum+=hypothesis->_probability;

        if(_use_minimum_representativity && probability_sum>_minimum_representativity)//we already added enough hypotheses
            break;
    }
}

void Mht::solveCluster(ClusterPtr cluster)
{
// 	cout<<"solveCluster: "<<*cluster<<endl;

    vector<HypothesisPtr> list_of_hypotheses;

    //Get the measurements vector
    vector<MeasurementPtr> measurements = cluster->assigned_measurements;
    vector<HypothesisPtr> hypotheses = cluster->assigned_hypotheses;

    assert(hypotheses.size()>0);

    //Check if this problem is trivial, 1 measurement with 1 hypothesis and 1 target
    if(measurements.size() == 1 && hypotheses.size() == 1 && hypotheses[0]->_targets.size()==1)
    {
// 		cout<<"trivial problem"<<endl;
        HypothesisPtr parent_hypothesis = hypotheses[0];

        //If this problem is trivial no Murty algorithm is needed and only one hypothesis exists
        //The association is strait forward, and we only need to branch once the previous hypothesis

        //This problem will surely have a solution because if we are here the association is within the limits
        //The probability of this hypothesis is 1, because of normalization factors we don't need to calculate it

        //Create new hypothesis
        HypothesisPtr hypothesis(new Hypothesis);

// 				hypothesis->_parent = parent_hypothesis;
        hypothesis->_parent_uid = parent_hypothesis->_uid;
        hypothesis->_iteration = iteration;
        hypothesis->_cluster = parent_hypothesis->_cluster;//The new hypothesis belongs to the same cluster has the old one
        hypothesis->_n_det = 1;//no detected target
        hypothesis->_prod = parent_hypothesis->_targets[0]->getDistance(measurements[0],1);//this will calculate the bivariate pdf
        //the rest is set to zero, nocc, ndel, nfal

        //Associate the target 0 with measurement 0 of the current hypothesis
        TargetPtr new_target = associate(parent_hypothesis->_targets[0],measurements[0]);
        //The previous association must occur

        if(new_target!=NULL)
        {
            new_target->_hypothesis = hypothesis->_uid;
            new_target->_cluster = hypothesis->_cluster;

// 				cout<<"n1"<<endl;
// 				assert(is_finite(new_target->_xp));
// 				assert(is_finite(new_target->_P));

            if(is_finite(new_target->_xp))
                hypothesis->_targets.push_back(new_target);

            //Add this hypothesis to the list of hypotheses for this problem
            if(hypothesis->_targets.size()>0)
                list_of_hypotheses.push_back(hypothesis);
        }else
        {
            const char COL_RESET[] = "\x1b[0m";
            const char RED[]       = "\x1b[31m";
            cout << RED << "Error!!, Major" << COL_RESET << endl;
            cout<<"failed a trivial association"<<endl;
        }

    }else//Not trivial problem
    {
// 		cout<<"non trivial problem"<<endl;
        //For all hypotheses in this problem

        //Create lots of threads

        for(uint h=0;h<hypotheses.size();++h)
        {
            HypothesisPtr parent_hypothesis = hypotheses[h];
            vector<TargetPtr> targets = parent_hypothesis->_targets;

            assert(targets.size()>0);

            if(parent_hypothesis->_status==DEAD)
                continue;

// 					cout<<"handling hypothesis: "<<prev_hypothesis->name()<<endl;
// 					cout<<"number of targets: "<<prev_hypothesis->_targets.size()<<endl;

            //Get list of targets associated with this hypotheses

            //Another trivial hypothesis, just one association possible in this hypothesis
            if(targets.size()==1 && measurements.size()==1)
            {
// 						cout<<"trivail version 2"<<endl;
                //create a single hypothesis from this one
                HypothesisPtr hypothesis(new Hypothesis);

// 						hypothesis->_parent = parent_hypothesis;
                hypothesis->_parent_uid = parent_hypothesis->_uid;
                hypothesis->_iteration=iteration;
                hypothesis->_cluster=parent_hypothesis->_cluster;//

                hypothesis->_n_det=1;//no detected target
                hypothesis->_prod = parent_hypothesis->_targets[0]->getDistance(measurements[0],1);//this will calculate the bivariate pdf

                TargetPtr new_target = associate(targets[0],measurements[0]);

                if(new_target!=NULL)
                {
                    new_target->_hypothesis = hypothesis->_uid;
                    new_target->_cluster = hypothesis->_cluster;

// 						cout<<"n2"<<endl;
// 						assert(is_finite(new_target->_xp));
// 						assert(is_finite(new_target->_P));

                    if(is_finite(new_target->_xp))
                        hypothesis->_targets.push_back(new_target);//Add the new target to the new hypothesis

                    //Add this hypothesis to the list of hypothesis for this problem if the association was successful
                    if(hypothesis->_targets.size()!=0)
                        list_of_hypotheses.push_back(hypothesis);

                }else
                {
                    const char COL_RESET[] = "\x1b[0m";
                    const char RED[]       = "\x1b[31m";
                    cout << RED << "Error!!, Major" << COL_RESET << endl;
                    cout<<"failed a trivial association"<<endl;
                }

                continue;//Jump the rest of the treatment of this hypothesis, jump to the next
            }

            if(measurements.size()==0)
            {
                //All the targets will correspond to miss associations

                //Create new hypothesis
                HypothesisPtr hypothesis(new Hypothesis);

// 						hypothesis->_parent = parent_hypothesis;
                hypothesis->_parent_uid = parent_hypothesis->_uid;
                hypothesis->_iteration=iteration;
                hypothesis->_cluster=parent_hypothesis->_cluster;

                for(uint t=0;t<targets.size();t++)
                {
                    if(targets[t]->_missed_associations < _miss_association_threshold)
                    {
                        //Iterate over failed association
                        TargetPtr new_target(new Target(targets[t]));

                        new_target->_hypothesis=hypothesis->_uid;
                        new_target->_cluster=hypothesis->_cluster;

// 								cout<<"n3"<<endl;
// 								assert(is_finite(new_target->_xp));
// 								assert(is_finite(new_target->_P));

                        if(is_finite(new_target->_xp))
                        {
                            hypothesis->_targets.push_back(new_target);
                            hypothesis->_n_occ++;//increment deleted target counter
                        }

                    }else
                    {
                        hypothesis->_n_del++;//increment deleted target counter
// 								cout<<"fa, removing target due to missed associations"<<endl;
                        //Do not add this target to the new hypothesis
                    }
                }

                //Add this hypothesis to the list of hypothesis for this problem
                if(hypothesis->_targets.size()>0)
                    list_of_hypotheses.push_back(hypothesis);

                continue;
            }

// 					cout<<"get probability and cost matrices target:"<<targets.size()<<" mes: "<<measurements.size()<<endl;

            //Define the cost matrix
            MatrixXd probability(measurements.size(),targets.size());

            TargetPtr empty;
            //Calculate the measurement to target association cost
            for(uint it=0;it<targets.size();++it)//For all targets
            {
                assert(targets[it]!=empty);
                for(uint im=0;im<measurements.size();++im)//For all measurements
                {
// 							cout<<"target("<<it<<") "<<*targets[it]<<endl;
// 							cout<<"measurement("<<im<<") "<<*measurements[im]<<endl;
                    probability(im,it)=targets[it]->getDistance(measurements[im],1);//this will calculate the bivariate pdf
// 							cout<<"p: "<<probability(im,it)<<endl;
                }
            }

            uint nT = targets.size();
            uint nM = measurements.size();

            //Convert the probability matrix to a cost matrix using -log()
            MatrixXd cost(probability.rows(),probability.cols());

            for(uint r=0;r<cost.rows();++r)
                for(uint c=0;c<cost.cols();++c)
                {
                    cost(r,c)=-log(probability(r,c));
                    if(std::isinf(cost(r,c)))
                        cost(r,c)=1e6;
                }

// 					cout<<"\n\tprobability: "<<probability<<endl;
// 					cout<<"\tcost: "<<cost<<endl<<endl;

            //Now that we have the cost matrix ready, we must apply the Murty algorithm k_best_assignment()

// 					cout<<"solve k-best, cluster: "<<cluster->id<<endl;
// 					cout<<"ntarters: "<<nT<<" nmeasurements: "<<nM<<endl;
            assert(_k!=-1);
            vector<AssignmentsPtr> assignments = k_best_assignment(cost,_k);
// 					cout<<"done"<<endl;
// 					cout<<"Assignments:\n"<<assignments<<endl;

            //Handle the new assignments, create a new target if the measurement is associated with an invalid target (it>=nT)
            //Increment invalid associations if this target is associated with an invalid measurement (im>=nM)
            //For all k or less solutions from the Murty
            for(uint i=0;i<assignments.size();++i)
            {
// 						cout<<*assignments[i]<<endl;

                //Get current assignment (just to simplify the code)
                vector<orderedPairPtr> assignment = assignments[i]->pairs;

                //Create new hypothesis
                HypothesisPtr hypothesis(new Hypothesis);

// 						hypothesis->_parent=parent_hypothesis;
                hypothesis->_parent_uid = parent_hypothesis->_uid;
                hypothesis->_iteration=iteration;
                hypothesis->_cluster=parent_hypothesis->_cluster;

                //Now i must check if there are unassigned measurements or targets
                vector<int> mes(nM,0);
                vector<int> tar(nT,0);

                for(uint e=0;e<assignment.size();++e)
                {
                    mes[assignment[e]->row]=1;
                    tar[assignment[e]->col]=1;
                }

                if(nM>nT)//Unassigned measurements
                {
                    //Now i must check which measurements were not assigned and create new targets with those measurements

                    for(uint im=0;im<mes.size();++im)
                        if(mes[im]==0)//Measurement not assigned
                        {
                            //New target
                            TargetPtr new_target(new Target(measurements[im]));

                            new_target->_hypothesis=hypothesis->_uid;
                            new_target->_cluster=hypothesis->_cluster;

// 									cout<<"n4"<<endl;
// 									assert(is_finite(new_target->_xp));
// 									assert(is_finite(new_target->_P));

                            if(is_finite(new_target->_xp))
                            {
                                hypothesis->_n_new++;//increment new target counter
                                hypothesis->_targets.push_back(new_target);
                            }
                        }

                }else if(nT>nM)//Unassigned targets
                {
                    for(uint it=0;it<tar.size();++it)
                        if(tar[it]==0)//Target not assigned
                        {
                            if(targets[it]->_missed_associations < _miss_association_threshold)
                            {
// 	 										cout<<"bellow min threshold"<<endl;
                                //Iterate over failed association
                                TargetPtr new_target(new Target(targets[it]));

                                new_target->_hypothesis=hypothesis->_uid;
                                new_target->_cluster=hypothesis->_cluster;

// 										cout<<"n5"<<endl;
// 										assert(is_finite(new_target->_xp));
// 										assert(is_finite(new_target->_P));

                                if(is_finite(new_target->_xp))
                                {
                                    hypothesis->_targets.push_back(new_target);
                                    hypothesis->_n_occ++;//increment deleted target counter
                                }

                            }else
                            {
                                hypothesis->_n_del++;//increment deleted target counter

// 											cout<<"fa, removing target due to missed associations"<<endl;
                                //Do not add this target to the new hypothesis
                            }
                        }
                }

                //Create new targets from associations
                for(uint p=0;p<assignment.size();++p)
                {
                    uint im = assignment[p]->row;
                    uint it = assignment[p]->col;

// 							cout<<*(measurements[im])<<endl;

                    TargetPtr new_target = associate(targets[it],measurements[im]);

                    if(new_target!=NULL)
                    {
                        new_target->_hypothesis=hypothesis->_uid;
                        new_target->_cluster=hypothesis->_cluster;

// 								cout<<*hypothesis<<endl;
// 								cout<<*new_target<<endl;

// 								cout<<"n6"<<endl;

// 								assert(is_finite(new_target->_xp));
// 								assert(is_finite(new_target->_P));
// 								cout<<"o6"<<endl;

                        if(is_finite(new_target->_xp))
                        {
                            hypothesis->_targets.push_back(new_target);
                            hypothesis->_n_det++;//increment deleted target counter
                            hypothesis->_prod*=targets[it]->getDistance(measurements[im],1);//using the

                            if(new_target->_missed_associations>0)//If the new targets is the result of miss association
                                hypothesis->_n_occ++;//increment number of occulded targets
                        }
                    }else
                    {
                        hypothesis->_n_del++;//increment deleted target counter
                    }
                }

                //Add this hypothesis to the list of hypothesis for this problem
                if(hypothesis->_targets.size()>0)
                    list_of_hypotheses.push_back(hypothesis);
            }
        }
    }

// 	cout<<"total hypotheses: "<<list_of_hypotheses.size()<<endl;

    //Calculate the probabilities of all the child hypotheses
    for(uint i=0;i<list_of_hypotheses.size();++i)
    {
        HypothesisPtr h = list_of_hypotheses[i];
        getProbability(h);
    }

// 			cout<<"calculated the probabilities of theis problem"<<endl;

    //If we choose to wait to do a reduce branching at this point all the k-best solutions to all the previous
    //hypotheses should be ranked and the j-best of them should be extracted and propagated
    sort(list_of_hypotheses.begin(),list_of_hypotheses.end(),compareHypothesesByProbabilityDescending);

    //Normalize only the j hypotheses
    assert(_j!=-1);
    normalize(list_of_hypotheses,_j);

//			cout<<"nomalizing j: "<<_j<<" hypotheses"<<endl;

    double probability_sum = 0;

// 			cout<<"add the leafs"<<endl;
    //Now we have the list of all k-best hypotheses to solve this problem
    //I will now find only the j best
    for(uint i=0;i<(uint)_j && i<list_of_hypotheses.size();++i)
    {
        HypothesisPtr hypothesis = list_of_hypotheses[i];
        HypothesisTree::iterator it;

        for(uint t=0;t<hypothesis->_targets.size();t++)
        {
            TargetPtr target = hypothesis->_targets[t];
            if(!is_finite(target->_xp) || !is_finite(target->_P))
            {
                cout<<"trying to add invalid target, iteration:  "<<iteration<<endl;
                cout<<*target<<endl;
                cout<<*hypothesis<<endl;
                cout<<target->_xp<<endl;
            }

            assert(is_finite(target->_xp));
            assert(is_finite(target->_P));
        }

// 				assert(hypothesis->_iteration==(hypothesis->_parent->_iteration+1));
// 				assert(hypothesis->_cluster==hypothesis->_parent->_cluster);

        for(HypothesisTree::iterator fi=_hypothesisTree->begin();fi!=_hypothesisTree->end();fi++)
            if((*fi)->_uid==hypothesis->_parent_uid)
            {
                it=fi;
                break;
            }

// 				it = find(_hypothesisTree->begin(),_hypothesisTree->end(),hypothesis->_parent);
        _hypothesisTree->append_child(it,hypothesis);//Append a new node

// 		cout<<"adding new hypothesis"<<endl;

        cluster->assigned_hypotheses.push_back(hypothesis);

        probability_sum+=hypothesis->_probability;

// 				cout<<"appended, child: "<<hypothesis->name()<<" parent: "<<(*_hleaf)->name()<<endl;

        if(_use_minimum_representativity && probability_sum>_minimum_representativity)//we already added enough hypotheses
            break;
    }
}


// static void* Mht::clusterThread(void *ptr)
// {
// 	ClusterPtr cluster = *(ClusterPtr*)ptr;
// 	solveProblem(cluster);
// }

void Mht::iterate(vector<MeasurementPtr>& measurements)
{
    ldb(1)<<endl<<endl;
    ldb(1)<<"*********************************************************************"<<endl;
    ldb(1)<<"k = "<<iteration<<endl<<endl;;

    _hypothesisTree->ready_from_draw=false;

    HypothesisTree::iterator it;
    HypothesisTree::sibling_iterator sib;

    ldb()<<"createClusters"<<endl;
    createClusters();

    ldb()<<"clearTargetAssociations"<<endl;
    clearTargetAssociations();
    consistencyCheck();

    //Process all hypothesis tree leafs, to obtain the current cluster vector
    ldb()<<"clusterAssignemnt"<<endl;
    clusterAssignemnt(measurements);
    consistencyCheck();

    ldb()<<"breakClusters"<<endl;
// 	cout<<"breakClusters"<<endl;
    breakClusters(measurements);
    cleanClusters();
    ldb()<<"done breakClusters"<<endl;
    consistencyCheck();

    //Now i must check if there are measurements with double assignments and merge those clusters
    ldb()<<"clusterMerger"<<endl;
    clusterMerger(measurements);
    //No consistency check may be performed before the call of cleanClusters, there are a few invalid clusters left from the merger
    ldb()<<"cleanClusters"<<endl;
    cleanClusters();
    consistencyCheck();

    ldb()<<"simplifySingleTargetClusters"<<endl;
    simplifySingleTargetClusters(measurements);
    consistencyCheck();

// 	vector<boost::thread*> threads;
    for(uint c=0;c<_clusters.size();c++)
    {
// 		solveCluster(_clusters[c]);
        ldb()<<"solveClusterMultiTread"<<endl;
        solveClusterMultiTread(_clusters[c]);
        ldb()<<"done solveClusterMultiTread"<<endl;
// 		boost::thread* thr = new boost::thread(boost::bind(&Mht::solveClusterMultiTread,this,boost::ref(_clusters[c])));
// 		threads.push_back(thr);
    }

// 	for(uint c=0;c<threads.size();c++)
// 	{
// 		threads[c]->join();
// 		delete threads[c];
// 	}

    ldb()<<"consistencyCheck"<<endl;
    consistencyCheck();

// 			cout<<"\nsolved all clusters"<<endl;

    //Mark all hypotheses that will not be propagated as DEAD
    for(_hleaf=_hypothesisTree->begin_leaf();_hleaf!=_hypothesisTree->end_leaf();++_hleaf)
        if((*_hleaf)->_iteration != iteration)
            (*_hleaf)->_status=DEAD;

// 			consistencyCheck();

// 			cout<<"\nRemove alive past hypotheses"<<endl;
    //Now we can remove the hypotheses from the previous iteration from the cluster list
    for(uint c=0;c<_clusters.size();c++)
    {
        ClusterPtr cluster = _clusters[c];

        vector<HypothesisPtr>::iterator it;
        for(it=cluster->assigned_hypotheses.begin();it!=cluster->assigned_hypotheses.end();)
        {
            HypothesisPtr hypothesis = *it;

            //Remove only past hypotheses that are not DEAD, the DEAD ones will be remove further
            if(hypothesis->_iteration<iteration && hypothesis->_status!=DEAD)
            {
                //Remove from the assignment list
                it = cluster->assigned_hypotheses.erase(it);
            }else
                it++;
        }
    }

// 	for(uint c=0;c<_clusters.size();c++)
// 		cout<<*_clusters[c]<<endl;

// 			consistencyCheck();

    ldb()<<"removeTheDead"<<endl;
    //Remove dead hypotheses and clusters that have only dead children
    removeTheDead();
    ldb()<<"removeIrrelevant"<<endl;
// 			consistencyCheck();
    removeIrrelevant();
// 			consistencyCheck();

    //Now i must check if there are measurements without assignment and add them has a new cluster
    for(uint m=0;m<measurements.size();m++)
    {
        cout<<"measurement: "<<measurements[m]->id<<" assigned to: "<<measurements[m]->assigned_clusters.size()<<" clusters"<<endl;
        if(measurements[m]->assigned_clusters.size()==0)//Measurement not assigned to any previous cluster
        {

            //Create new hypothesis
            HypothesisPtr hypothesis(new Hypothesis);

            hypothesis->_iteration=iteration;
            hypothesis->_n_new=1;//One new target

            hypothesis->_probability=1;//This is the only hypothesis in this cluster so its probability is 1

            //Add new cluster
            ClusterPtr cluster(new Cluster);

            cluster->id=_cluster_id++;
            hypothesis->_cluster=cluster->id;

            cluster->assigned_measurements.push_back(measurements[m]);
            cluster->assigned_hypotheses.push_back(hypothesis);

            measurements[m]->assigned_clusters.push_back(cluster);

            //Create new target
            TargetPtr target(new Target(measurements[m]));

            target->_hypothesis=hypothesis->_uid;
            target->_cluster=hypothesis->_cluster;

            getProbability(hypothesis);

            //Add target to the hypothesis
            hypothesis->_targets.push_back(target);

            //Insert hypothesis in the tree as a root branch, effectively creating a new cluster
            _hypothesisTree->insert(_hypothesisTree->begin(),hypothesis);//insert root node

            //It any of this clusters interfere with each other they will be merged in the next iteration

            _clusters.push_back(cluster);
        }
    }

// 	for(uint c=0;c<_clusters.size();c++)
// 		cout<<*_clusters[c]<<endl;

    updateTargetList();
    checkCurrentTargets(measurements);

    //Just change the color of hypothesis given their branch in the tree
    for(_iterator=_hypothesisTree->begin();_iterator!=_hypothesisTree->end();++_iterator)
        switchColor(*_iterator);

    _hypothesisTree->need_layout=true;
    iteration++;//iterations start in 0

// 			cout<<"\nfinal clusters, iteration: "<<iteration<<endl<<endl;
// 			for(uint c=0;c<_clusters.size();c++)
// 				cout<<*_clusters[c]<<endl;

    _clusters.clear();
// 			cout<<"full tree:"<<endl;
// 			for(HypothesisTree::iterator ih=_hypothesisTree->begin();ih!=_hypothesisTree->end();ih++)
// 				cout<<"\t"<<**ih<<" use count: "<< (*ih).use_count()<<endl;

// 			if(iteration==306)
// 			{
// 				cout<<"exited by command"<<endl;
// 				exit(0);
// 			}

    ldb(1)<<endl<<"Iteration end: "<<iteration-1<<endl;
}

hypothesisTreePtr Mht::getHypothesisTree(void)
{
    return _hypothesisTree;
}

vector<TargetPtr> Mht::getTargets(void)
{
// 			cout<<endl<<"get current targets"<<endl;
    return _current_targets;
}

void Mht::clear(void)
{
    _hypothesisTree->clear();
    _current_targets.clear();
    _clusters.clear();
    iteration=0;
    Hypothesis::_euid=0;
    Hypothesis::_euid=0;
    Target::_euid=0;
    _cluster_id = 0;
}

string Mht::getReportName(string extra)
{
    stringstream stream(fileName);//get stream to file full name

    string name;

    while(getline(stream,name,'/'));//get the file name, into the name variable

    stream.clear();//clear the stream
    stream.str(name);//set the name to the stream

    getline(stream,name,'.');//get the name without the file extension

    //Create the report name
    string report_name = _aux1;

    report_name+=name;
    report_name+=extra;
    report_name+=".txt";

    return report_name;
}

void Mht::recursiveHypothesisRemove(HypothesisTree::iterator it)
{
    if(_hypothesisTree->number_of_children(it)==0)
    {
        HypothesisTree::iterator parent_it = _hypothesisTree->parent(it);
        //Remove child
// 				cout<<"removing g1: "<<**it<<" "<<*it<<endl;
        for(uint t=0;t<(*it)->_targets.size();t++)
            (*it)->_targets[t]->_assigned_measurements.clear();
        (*it)->_targets.clear();

        _hypothesisTree->erase(it);

        if(_hypothesisTree->is_valid(parent_it))
            recursiveHypothesisRemove(parent_it);
    }
}

void Mht::removeIrrelevant(void)
{
    for(uint c=0;c<_clusters.size();++c)
    {
        ClusterPtr cluster = _clusters[c];

        HypothesisTree::iterator first = find(_hypothesisTree->begin(),_hypothesisTree->end(),cluster->assigned_hypotheses[0]);

        assert(_hypothesisTree->is_valid(first));

        HypothesisTree::iterator min = first;
        HypothesisTree::iterator aux;

        if(cluster->assigned_hypotheses.size()==1)
        {
            //If there is only one hypothesis in this cluster i must get its parent from a fixed iteration

            min=first;
            for(uint h=0;h<_history_size-1;++h)
            {
                if(_hypothesisTree->is_valid(_hypothesisTree->parent(min)))
                    min=_hypothesisTree->parent(min);
                else
                    break;
            }

            aux=min;
            while(_hypothesisTree->is_valid(_hypothesisTree->parent(aux)))
                aux=_hypothesisTree->parent(aux);

            (*min)->_status=PARENT;

            if(_remove_unused)
            {
                _hypothesisTree->insert_subtree(aux,min);

                //Must clear the vectors by hand, should not be necessary

                for(uint t=0;t<(*aux)->_targets.size();t++)
                    (*aux)->_targets[t]->_assigned_measurements.clear();
                (*aux)->_targets.clear();

// 						cout<<"removing: "<<**aux<<" "<<*aux<<" use count: "<<p.use_count()<<endl;
                _hypothesisTree->erase(aux);
            }

// 					cout<<"just one hypothesis, out of cluster: "<<_clusters[c]->id<<endl;
            continue;
        }

        for(uint h=1;h<cluster->assigned_hypotheses.size();++h)
        {
            HypothesisPtr hypothesis = cluster->assigned_hypotheses[h];

// 					cout<<*hypothesis<<endl;

            HypothesisTree::iterator current = find(_hypothesisTree->begin(),_hypothesisTree->end(),hypothesis);

            assert(_hypothesisTree->is_valid(current));
            assert(_hypothesisTree->is_valid(first));

// 					cout<< *(*current) <<endl;
// 					cout<< *(*first) <<endl;

            aux=_hypothesisTree->lowest_common_ancestor(first,current);

            assert(_hypothesisTree->is_valid(aux));

            if((*aux)->_iteration<(*min)->_iteration)
                min=aux;
        }

        if( (iteration - (*min)->_iteration) > _history_size )
        {
            int iter = iteration - _history_size + 1;

// 					cout<<"minimum valid iteration: " <<iter<<" min iteration: "<<(*min)->_iteration<<endl;
            //Get all siblings at iteration _history_size and move them to the same parent

            //Get all the tree nodes from the right cluster at the right iteration
            vector<HypothesisPtr> positions;
            HypothesisPtr parent;
            HypothesisPtr minhyp;
            HypothesisTree::iterator aux_parent;
            HypothesisTree::iterator abs_parent;

            for(aux=_hypothesisTree->begin();aux!=_hypothesisTree->end();aux++)
            {
                if((*aux)->_cluster==cluster->id)
                    if((*aux)->_iteration==iter)
                    {
                        if(parent==NULL && _hypothesisTree->is_valid(_hypothesisTree->parent(aux)))
                        {
                            aux_parent=_hypothesisTree->parent(aux);
                            parent=*aux_parent;
                        }else
                            positions.push_back(*aux);
                    }
            }

            parent=*aux_parent;
            minhyp=*min;

            parent->_status=PARENT;

            //First insert a new subtree with parent, at the min position
            _hypothesisTree->insert_subtree(min,aux_parent);

            for(uint i=0;i<positions.size();i++)
            {
                //Reparent all the hypothesis in the list
                //First get the iteration to the position

                aux_parent=find(_hypothesisTree->begin(),_hypothesisTree->end(),parent);
                aux=find(_hypothesisTree->begin(),_hypothesisTree->end(),positions[i]);
                _hypothesisTree->append_child(aux_parent,aux);

                _hypothesisTree->erase(aux);
            }

            min=find(_hypothesisTree->begin(),_hypothesisTree->end(),minhyp);

            _hypothesisTree->erase(min);

            //Check the tree for repeated hypotheses

            for(aux=_hypothesisTree->begin();aux!=_hypothesisTree->end();aux++)
            {
                int c=0;
                for(HypothesisTree::iterator aux2=_hypothesisTree->begin();aux2!=_hypothesisTree->end();aux2++)
                    if(*aux==*aux2)
                        c++;

                if(c>1)
                    cout<<"Repeated hypothesis ("<<c<<") : "<<**aux<<endl;
                assert(c==1);
            }

            continue;
        }

        aux=min;
        while(_hypothesisTree->is_valid(_hypothesisTree->parent(aux)))
            aux=_hypothesisTree->parent(aux);

        if((*aux)->name()==(*min)->name())//Do nothing, the root not is the minimum common ancestor
            continue;
        else
        {
            (*min)->_status=PARENT;

            if(_remove_unused)
            {
                _hypothesisTree->insert_subtree(aux,min);
                _hypothesisTree->erase(aux);
            }
        }
    }
}

void Mht::removeTheDead(void)
{
    vector<ClusterPtr>::iterator it_cluster;

    for(it_cluster=_clusters.begin();it_cluster!=_clusters.end();)
    {
        ClusterPtr cluster = *it_cluster;

        vector<HypothesisPtr>::iterator it_assigned;

        for(it_assigned=cluster->assigned_hypotheses.begin();it_assigned!=cluster->assigned_hypotheses.end();)
        {
            HypothesisPtr hypothesis = *it_assigned;

            if(hypothesis->_status == DEAD)
            {
                it_assigned = cluster->assigned_hypotheses.erase(it_assigned);

                HypothesisTree::iterator it = find(_hypothesisTree->begin(),_hypothesisTree->end(),hypothesis);

// 						cout<<"\tremoving hypothesis: "<<**it<<endl;
                recursiveHypothesisRemove(it);
            }else
                it_assigned++;

            if(it_assigned==cluster->assigned_hypotheses.end())
                break;
        }

        if(cluster->assigned_hypotheses.size()==0)
            it_cluster = completeRemoveCluster(it_cluster);
        else
            it_cluster++;

        if(it_cluster==_clusters.end())
            break;
    }
}

vector<ClusterPtr>::iterator Mht::completeRemoveCluster(long id)
{
    cout<<"removing cluster id: "<<id<<endl;

    vector<ClusterPtr>::iterator it;

    for(it=_clusters.begin();it!=_clusters.end();it++)
        if((*it)->id==id)
            return completeRemoveCluster(it);

    return _clusters.end();
}

vector<ClusterPtr>::iterator Mht::completeRemoveCluster(vector<ClusterPtr>::iterator it_cluster)
{
    ClusterPtr cluster = *it_cluster;
    cout<<"removing cluster it id: "<<cluster->id<<endl;


    //Get all hypotheses
    vector<HypothesisPtr> hypotheses = cluster->assigned_hypotheses;

// 	if(hypotheses.size()==0)
// 		cout<<"removing empty cluster: "<<*cluster<<endl;

    //If we have hypotheses to delete
    if(hypotheses.size()>0)
    {
        //Find a iterator in the tree
        HypothesisTree::iterator aux = find(_hypothesisTree->begin(),_hypothesisTree->end(),hypotheses[0]);

        if(aux!=NULL)
        {
// 			cout<<"erasing hypotheses from tree"<<endl;

            //Get the topmost parent of the hypothesis
            while(_hypothesisTree->is_valid(_hypothesisTree->parent(aux)))
                aux=_hypothesisTree->parent(aux);

            //Erase the topmost parent, this will erase all children
            _hypothesisTree->erase(aux);
        }
    }

    //Clear assigned measurements
    (*it_cluster)->assigned_measurements.clear();
    //Clear all assigned hypotheses
    (*it_cluster)->assigned_hypotheses.clear();

    //Remove cluster from the list of clusters
    it_cluster = _clusters.erase(it_cluster);

    return it_cluster;
}

MeasurementPtr Mht::findMeasurement(vector<MeasurementPtr>& measurements, long id)
{
    for(uint i=0;i<measurements.size();i++)
        if(measurements[i]->id==id)
            return measurements[i];
    return MeasurementPtr();
}

void Mht::switchColor(HypothesisPtr hypothesis)
{
    assert(hypothesis!=0);

    static class_colormap*colormap=NULL;
    int total = 10;

    if(!colormap)
        colormap = new class_colormap(string("hsv"),total,1.0);

    Color color(colormap->color(hypothesis->_cluster));

    hypothesis->setAttribute(string("style"),string("bold, filled"));
    hypothesis->setAttribute(string("fillcolor"),string("white"));
    hypothesis->setAttribute(string("color"),color.str());

    switch(hypothesis->_status)
    {
        case NORMAL://normal, just a plain child of someone else
            break;

        case MOVED://moved from its original cluster to a new one
            color.changeColorBrightness(0.7);
            hypothesis->setAttribute(string("fillcolor"),color.str());
            break;

        case PARENT://Lowest common ancestor of the current set of hypotheses in a cluster
            color.changeColorBrightness(0.0);
            hypothesis->setAttribute(string("fillcolor"),color.str());
            break;

        case FORCED_PARENT://new forced parent

            color.changeColorBrightness(0.2);
            hypothesis->setAttribute(string("fillcolor"),color.str());
            break;

        case DEAD://no children hypothesis, without its offspring this hypothesis is dead
            hypothesis->setAttribute(string("fillcolor"),string("#D8D8D8"));

// 					hypothesis->setAttribute(string("color"),string("#FFFF33"));
// 					hypothesis->setAttribute(string("fillcolor"),string("#FF0000"));
// 					hypothesis->setAttribute(string("fontcolor"),string("white"));
            break;

        case DEAD_FORCED_PARENT:
            hypothesis->setAttribute(string("color"),string("white"));
            hypothesis->setAttribute(string("fillcolor"),string("black"));
            hypothesis->setAttribute(string("fontcolor"),string("white"));
            break;

        case ERROR://Error code, bright red with yellow line, and white text
            hypothesis->setAttribute(string("color"),string("#FFFF33"));
            hypothesis->setAttribute(string("fillcolor"),string("#FF0000"));
            hypothesis->setAttribute(string("fontcolor"),string("white"));
            break;
    }
}

vector<HypothesisPtr> Mht::findHypotheses(int cl,int it_past)
{
    vector<HypothesisPtr> hypotheses;

    HypothesisTree::leaf_iterator hleaf;

    for(hleaf=_hypothesisTree->begin_leaf();hleaf!=_hypothesisTree->end_leaf();++hleaf)
    {
        HypothesisPtr hypothesis = *hleaf;

        if(hypothesis->_status==DEAD)
            continue;

        if(it_past==-1)//do not select iteration
        {
            if(hypothesis->_cluster==cl)
                hypotheses.push_back(*hleaf);
        }else
        {
            if( (hypothesis->_cluster==cl) && (hypothesis->_iteration==iteration-it_past))
                hypotheses.push_back(hypothesis);
        }
    }

    return hypotheses;
}

ClusterPtr Mht::findCluster(vector<ClusterPtr>& clusters, int id)
{
    ClusterPtr empty;

    for(uint i=0;i<clusters.size();++i)
        if(clusters[i]->id==id)
            return clusters[i];

    return empty;
}

void Mht::createClusters(void)
{
    assert(_clusters.size()==0);//Check if the input cluster vector is clean

    ClusterPtr empty;

    //Create an empty list of all clusters only with the associated hypotheses listed
    for(_hleaf=_hypothesisTree->begin_leaf();_hleaf!=_hypothesisTree->end_leaf();++_hleaf)
    {
        HypothesisPtr hypothesis = *_hleaf;

        if( hypothesis->_iteration < (iteration-1))
        {
// 					if(hypothesis->_status!=DEAD && hypothesis->_status!=ERROR)
// 					{
            const char COL_RESET[] = "\x1b[0m";
            const char RED[]       = "\x1b[31m";
            cout << RED << "Error!!, Major" << COL_RESET;
// 						cout<<" leaf from the wrong iteration: "<<(*_hleaf)->name()<<" not marked DEAD or ERROR"<<endl;

            cout<<" leaf from the wrong iteration: "<<(*_hleaf)->name()<<endl;
// 						hypothesis->_status=DEAD;//mark as dead
            hypothesis->_status=ERROR;//mark as error

            //Remove this hypothesis from the tree

// 						HypothesisTree::iterator it = find(_hypothesisTree.begin(),_hypothesisTree.end(),hypothesis);

// 						_hypothesisTree->erase(_hleaf);

// 						_hleaf=_hypothesisTree->begin_leaf();//reset iterator

// 						exit(0);
// 					}

            //Must add this to the clusters to be eliminated further down the line

// 					continue;//Jump leafs from other iterations
        }

        if(findCluster(_clusters,hypothesis->_cluster)==empty)
        {
// 					cout<<"created cluster:"<<(*_hleaf)->_cluster<<endl;
            ClusterPtr cluster(new Cluster);
            cluster->id=hypothesis->_cluster;
            cluster->assigned_hypotheses = findHypotheses(cluster->id);

            _clusters.push_back(cluster);
        }
    }
}

void Mht::clusterAssignemnt(vector<MeasurementPtr>& measurements)
{
    TargetPtr empty;

    //Process all targets, first go through all clusters
    for(uint c=0;c<_clusters.size();c++)
    {
        ClusterPtr cluster = _clusters[c];
        vector<HypothesisPtr> hypotheses = cluster->assigned_hypotheses;

        //Then all hypotheses associated with this cluster
        for(uint h=0;h<hypotheses.size();h++)
        {
            HypothesisPtr hypothesis = hypotheses[h];

            //Then all targets in this hypothesis
            for(uint t=0;t<hypothesis->_targets.size();++t)//Go through all targets in this hypothesis
            {
                //Get a target pointer just to simplify
                TargetPtr target=hypothesis->_targets[t];

                assert(target!=empty);

                //Go through all measurements
                for(uint m=0;m<measurements.size();++m)
                {
// 					bool association = false;

                    if(target->getDistance(measurements[m])<_max_gating)
                    {
                        target->_assigned_measurements.push_back(measurements[m]);
// 						association = true;
                    }else
                        continue;//No association

                    //If measurement already assigned with this cluster jump
                    if(find(measurements[m]->assigned_clusters.begin(),measurements[m]->assigned_clusters.end(),cluster)!=measurements[m]->assigned_clusters.end())
                        continue;

                    //If measurement is within gate of the target add it to the assignment
                    measurements[m]->assigned_clusters.push_back(cluster);

                    if(find(cluster->assigned_measurements.begin(),cluster->assigned_measurements.end(),measurements[m])!=cluster->assigned_measurements.end())
                        continue;

                    cluster->assigned_measurements.push_back(measurements[m]);
                }
            }
        }
    }
}

void Mht::cleanClusters(void)
{
    cout<<"begin clean"<<endl;
    vector<ClusterPtr>::iterator it;

    for(it=_clusters.begin();it!=_clusters.end();)
        if((*it)->isEmpty())
            it = completeRemoveCluster(it);
        else
            it++;

    cout<<"end clean"<<endl;
}

void Mht::removeEmptyClusters(void)
{
    ClusterPtr empty;

    if(_remove_empty_clusters==false)
        return;

    for(_hleaf=_hypothesisTree->begin_leaf();_hleaf!=_hypothesisTree->end_leaf();++_hleaf)
    {
        HypothesisPtr hypothesis = *_hleaf;

        if(hypothesis->_status==DEAD || hypothesis->_iteration!=iteration)
        {
            long cluster_id = hypothesis->_cluster;

            if(findCluster(_clusters,cluster_id)==empty)
            {
                cout<<"cluster: "<<cluster_id<<" marked for deletion"<<endl;

                HypothesisTree::iterator aux=_hleaf;

                while(_hypothesisTree->is_valid(_hypothesisTree->parent(aux)))
                    aux=_hypothesisTree->parent(aux);

                _hypothesisTree->erase(aux);

                //Reset the leaf due to the changes in the tree
                _hleaf=_hypothesisTree->begin();
            }
        }
    }
}

bool Mht::consistencyCheck(void)
{
// 			cout<<"\nconsistency check\n"<<endl;
    for(uint c=0;c<_clusters.size();c++)
    {
        ClusterPtr cluster = _clusters[c];

// 				cout<<*cluster<<endl;

        if(cluster->assigned_hypotheses.size()==0)
        {
            cout<<"major error! cluster without hypotheses"<<endl;
            cout<<*cluster<<endl;
        }

        assert(cluster->assigned_hypotheses.size()>0);

        for(uint h=0;h<cluster->assigned_hypotheses.size();h++)
        {
            HypothesisPtr hypothesis = cluster->assigned_hypotheses[h];

// 					cout<<"\t"<<*hypothesis<<endl;

// 					if(hypothesis->_cluster!=cluster->id)
// 					{
// 						cout<<"smp miss cluster in hypo: "<<*hypothesis<<endl;
//
// 					}

            assert(hypothesis->_cluster==cluster->id);

// 					if(hypothesis->_targets.size()==0)
// 					{
// 						cout<<"mer hypo: "<<*hypothesis<<endl;
// 					}

            assert(hypothesis->_targets.size()>0);

            for(uint t=0;t<hypothesis->_targets.size();t++)
            {
                TargetPtr target = hypothesis->_targets[t];
// 						cout<<"\t\t"<<*target<<endl;

                if(target->_hypothesis!=hypothesis->_uid)
                {
                    cout<<"major error"<<endl;
                    cout<<*hypothesis<<endl;
                    cout<<*target<<endl;
                }

                if(!is_finite(target->_xp) || !is_finite(target->_P))
                {
                    cout<<"major error, iteration "<<iteration<<endl;
                    cout<<*target<<endl;
                    cout<<*hypothesis<<endl;
                    cout<<*cluster<<endl;
                    cout<<target->_xp<<endl;
                }
// 						cout<<*hypothesis<<endl;
// 						cout<<*target<<" "<<target<<endl;

// 						cout<<"check 3"<<endl;
                assert(is_finite(target->_xp));
                assert(is_finite(target->_P));
// 						cout<<"pass 3"<<endl;

                assert(target->_cluster==cluster->id);
                assert(target->_hypothesis==hypothesis->_uid);
            }
        }
    }

    return true;
}

void Mht::breakClusters(vector<MeasurementPtr>& measurements)
{
// 	cout<<endl<<__FUNCTION__<<endl<<endl;

    vector<ClusterPtr>::iterator it_cluster;
    vector<ClusterPtr> new_clusters;

    //Check all clusters
    for(it_cluster=_clusters.begin();it_cluster!=_clusters.end();it_cluster++)
    {
        //Get a list of possible targets to break
        vector<TargetPtr> break_targets;

        //Get a pointer to the current cluster
        ClusterPtr cluster = *it_cluster;

        ldb()<<"checking:"<<endl;
        ldb()<<*cluster<<endl;

        //Hypotheses iterator
        vector<HypothesisPtr>::iterator it_hypothesis;

        //For all hypotheses in this cluster
        for(it_hypothesis=cluster->assigned_hypotheses.begin();it_hypothesis!=cluster->assigned_hypotheses.end();it_hypothesis++)
        {
            //Get a pointer to the current hypothesis
            HypothesisPtr hypothesis = *it_hypothesis;

            //Do assertions
            assert(hypothesis->_cluster==cluster->id);
            assert(hypothesis->_targets.size()>0);

            ldb()<<*hypothesis<<endl;

            //Do not remove more targets, if this hypothesis has too few targets (just one)
            if(hypothesis->_targets.size()<2)
                continue;

            //Target iterator
            vector<TargetPtr>::iterator it_target;

            //Go though all targets in this hypothesis
            for(it_target=hypothesis->_targets.begin();it_target!=hypothesis->_targets.end();it_target++)
            {
                //If this flag is still true by the end of the iteration this target is a candidate for separation
                bool separate_target = true;

                //Get a pointer to the current target
                TargetPtr target = *it_target;

                ldb()<<"check: "<<*target<<endl;

                //Do assertions
                assert(target!=NULL);
                assert(target->_cluster==cluster->id);
                assert(target->_hypothesis==hypothesis->_uid);

                //If this target was associated with more that one measurement, it must not be separated
                if(target->_assigned_measurements.size()>1)
                    separate_target=false;


                //Auxiliary target iterator
                vector<TargetPtr>::iterator it_target_aux;

// 				ldb()<<"stuff"<<endl;
// 				ldb()<<"Number of targets: "<<hypothesis->_targets.size()<<endl;

                //Check if this target was associated with a measurement also shared by one of its partners
                for(it_target_aux=hypothesis->_targets.begin();it_target_aux!=hypothesis->_targets.end();it_target_aux++)
                {
// 					ldb()<<"target: "<<**it_target_aux<<endl;

                    if(it_target_aux==it_target)//self reference
                    {
// 						ldb()<<"continue"<<endl;
                        continue;
                    }

                    //Get a pointer to the other target
                    TargetPtr other_target = *it_target_aux;

                    assert(other_target!=NULL);

// 					ldb()<<"before find"<<endl;
// 					ldb()<<"other target: "<<*other_target<<endl;
// 					ldb()<<"number of assingned measuremnts ot: "<<other_target->_assigned_measurements.size()<<endl;
// 					ldb()<<"number of assingned measuremnts t: "<<target->_assigned_measurements.size()<<endl;

                    //If this target was associated with a measurement, see if it shares that measurement with any other target
                    if(target->_assigned_measurements.size()>0)
                    {
                        if(find(other_target->_assigned_measurements.begin(),other_target->_assigned_measurements.end(),target->_assigned_measurements[0])!=other_target->_assigned_measurements.end())
                            separate_target=false;
                    }

// 					ldb()<<"midle"<<endl;
// 							cout<<"h2"<<endl;

                    //This measurement was associated with just one target, but it may be in the proximity of a non associated target
// 					if(other_target->_missed_associations>0)//This target was not associated with any measurement
// 					{
                    if(target->getDistanceToPredictedPosition(other_target)<_max_gating)
                        separate_target=false;
// 					}

// 					ldb()<<"end"<<endl;
                }

                ldb()<<"done run for targets"<<endl;

                if(separate_target==false)
                    continue;//continue to next target

// 				cout<<"hf"<<endl;

                ldb()<<"possible breaking: "<<*target<<endl;
                //Add the target to the list of targets to break
                break_targets.push_back(target);
            }

            ldb()<<"done checking hypotheses"<<endl;
        }

        ldb()<<"done checking: possible targets: "<<break_targets.size()<<endl;

        //No targets to break
        if(break_targets.size()==0)
        {
            ldb()<<"continuing"<<endl;
            if(it_cluster==_clusters.end())
            {
                ldb()<<"breaking"<<endl;
                break;
            }
            continue;
        }
        ldb()<<"creating list of valid targets"<<endl;

        vector<TargetPtr> valid_targets;

        //Now we have a list of targets to break for this cluster
        assert(cluster!=NULL);

// 				cout<<"cluster: "<<*cluster<<endl;
// 				cout<<"targets to break apart"<<endl;

// 				for(uint t=0;t<break_targets.size();t++)
// 					cout<<"\t"<<*break_targets[t]<<endl;
// 				cout<<"done"<<endl;


        //The break targets can correspond to a single measurement but taken form several hypothesis
        //For now ill choose just the most likely to be propagated

        //For each target obtain the aux1 variable, the corresponding hypothesis probability
        for(uint t=0;t<break_targets.size();t++)
        {
            HypothesisPtr hypothesis = getHypothesisPtr(break_targets[t]->_hypothesis,cluster);
            break_targets[t]->_aux1=hypothesis->_probability;
        }

// 		cout<<"probabilities obtained"<<endl;

        sort(break_targets.begin(),break_targets.end(),compareTargetsByAux1Descending);

// 		cout<<"sorting done"<<endl;

        //Now we have a ordered list of targets to break, the next task is to obtain a valid list
        //This list will contain no repeated targets, and only the highest probability target for each id

        for(uint t=0;t<break_targets.size();t++)
        {
            //Get the list of valid targets

            bool existing=false;

            for(uint v=0;v<valid_targets.size();v++)
            {
                if(break_targets[t]->_id==valid_targets[v]->_id)
                {
                    existing=true;
                    break;
                }

                if(break_targets[t]->_assigned_measurements.size()>0 && valid_targets[v]->_assigned_measurements.size()>0)
                    if(break_targets[t]->_assigned_measurements[0]->id==valid_targets[v]->_assigned_measurements[0]->id)
                    {
                        existing=true;//The measurements are overlapping, do not extract this target
                        break;
                    }
            }

            if(!existing)
                valid_targets.push_back(break_targets[t]);
        }

// 		cout<<"valid targets to remove: "<<valid_targets.size()<<endl;
// 		for(uint s=0;s<valid_targets.size();s++)
// 			cout<<"\t"<<*(valid_targets[s])<<endl;

        for(uint t=0;t<valid_targets.size();t++)
        {
            TargetPtr target = valid_targets[t];

// 			cout<<"doing target: "<<*target<<endl;

            bool remove=true;

// 			cout<<"check each hypothesis"<<endl;
            for(vector<HypothesisPtr>::iterator itc=cluster->assigned_hypotheses.begin();itc!=cluster->assigned_hypotheses.end();)
            {
                HypothesisPtr hypothesis = *itc;

                if(hypothesis->_targets.size()==1)//Last target of this hypothesis, cannot remove
                {
                    remove = false;
                    break;
                }

                uint number_of_targets=hypothesis->_targets.size();

                for(vector<TargetPtr>::iterator it=hypothesis->_targets.begin();it!=hypothesis->_targets.end();)
                {
                    if((*it)->_id==target->_id)//Remove if the id is overlapped
                    {
// 								cout<<"\tremoved target, id overlap: "<<**it<<endl;
                        it=hypothesis->_targets.erase(it);
                    }else if((*it)->_assigned_measurements.size()>0 && target->_assigned_measurements.size()>0)
                    {
                        if((*it)->_assigned_measurements[0]->id == target->_assigned_measurements[0]->id)//Remove if measurement is overlapped
                        {
// 								cout<<"\tremoved target, measurement overlap: "<<**it<<endl;
                            it=hypothesis->_targets.erase(it);
                        }else
                            it++;
                    }else
                        it++;
                }

// 						if(hypothesis->_targets.size()==0)
                if(hypothesis->_targets.size()!=number_of_targets-1)//This hypothesis has lost more that one target, so it is not compatible with the rest of the algorithm and will cause problems
                {
                    hypothesis->_status=DEAD;

                    itc = cluster->assigned_hypotheses.erase(itc);

                    HypothesisTree::iterator ith = find(_hypothesisTree->begin(),_hypothesisTree->end(),hypothesis);

                    recursiveHypothesisRemove(ith);
                }else
                    itc++;
            }

// 			cout<<"check hypotheses done"<<endl;

            if(!remove)//Do not remove if this target is marked as the last of the hypotheses
                continue;


            if(target->_assigned_measurements.size()>0)
            {
// 				cout<<"removing measurement association"<<endl;
                //Remove the cluster from the measurement
                removeLocal(target->_assigned_measurements[0]->assigned_clusters,cluster);

                //Remove the measurement form the cluster
                removeLocal(cluster->assigned_measurements,target->_assigned_measurements[0]);
            }

            if(cluster->assigned_hypotheses.size()==0)
            {
                ldb()<<"cluster without any hypothesis: "<<*cluster<<endl;
// 				it_cluster = completeRemoveCluster(it_cluster);
            }

// 			cout<<endl<<"moving the target:"<<endl;
// 			cout<<"\t"<<*target<<" prob: "<<target->_aux1<<endl;

            //Now we separate the target
            //Remove target and associated hypothesis from the current cluster
            HypothesisPtr new_hypothesis(new Hypothesis);

            new_hypothesis->_iteration=iteration-1;//This must have the date from the previous iteration, or else the assert() will catch the error further on
            new_hypothesis->_cluster=_cluster_id++;//
            new_hypothesis->_n_new=1;//One new target

            target->_cluster = new_hypothesis->_cluster;
            target->_hypothesis = new_hypothesis->_uid;

// 			cout<<"Creating cluster: "<<target->_cluster<<" from target: "<<target->_id<<endl;

            //Add target to new hypothesis
            new_hypothesis->_targets.push_back(target);//push the target to the new hypothesis

            getProbability(new_hypothesis);

            //Insert hypothesis in the tree as a root branch, effectively creating a new cluster
            _hypothesisTree->insert(_hypothesisTree->begin(),new_hypothesis);//insert root node	with the new target

            //And add a new cluster
            ClusterPtr new_cluster(new Cluster);

            new_cluster->id = new_hypothesis->_cluster;

            if(target->_assigned_measurements.size()>0)
                new_cluster->assigned_measurements.push_back(target->_assigned_measurements[0]);

            new_cluster->assigned_hypotheses.push_back(new_hypothesis);

            new_clusters.push_back(new_cluster);

            //... and add the new cluster reference
// 			if(target->_assigned_measurements.size()>0 && target->_assigned_measurements[0]==NULL)
// 				cout<<"\tNull assigned measurement"<<endl;

            if(target->_assigned_measurements.size()>0 && target->_assigned_measurements[0]!=NULL)
            {
                for(uint m=0;m<measurements.size();m++)
                {
                    if(measurements[m]->id==target->_assigned_measurements[0]->id)
                    {
                        measurements[m]->assigned_clusters.push_back(new_cluster);
                        break;
                    }
                }

// 				cout<<"\tadded the new cluster to the measurement: "<<target->_assigned_measurements[0]->id<<endl;
            }

// 			cout<<endl<<"new cluster added: "<<*new_cluster<<endl;
// 			cout<<"old cluster: "<<*cluster<<endl;
        }

        ldb()<<"finish with cluster"<<endl;
    }

    //Now add all new clusters to the main cluster list
    ldb()<<"addining: "<<new_clusters.size()<<" new clusters"<<endl;
    _clusters.insert(_clusters.end(),new_clusters.begin(),new_clusters.end());

    ldb()<<"end cluster break"<<endl;
}

HypothesisTree::iterator Mht::insertForcedParent(long cluster)
{
    //so we will create a new parent and change their place
    HypothesisPtr parent_hypothesis(new Hypothesis);

    parent_hypothesis->_iteration=-1;
    parent_hypothesis->_cluster=cluster;
    parent_hypothesis->_status=FORCED_PARENT;
    parent_hypothesis->_probability=1.;

    //insert root node
    return _hypothesisTree->insert(_hypothesisTree->begin(),parent_hypothesis);
}

vector<TargetPtr> Mht::createCopy(vector<TargetPtr>& original)
{
// 			cout<<"coping stuff"<<endl;

    vector<TargetPtr> new_targets;

    for(uint t=0;t<original.size();t++)
    {
        TargetPtr target(new Target);

        long uid=target->_uid;

        *target=*(original[t]);

        target->_uid=uid;
        target->_variant+="*";

        //This was used in the past but i think it is not needed
        //target->_id=target->_uid++;
// 				target->_id=target->_uid;
// 				target->_id=-1;

// 				cout<<"check 2"<<endl;
        assert(is_finite(target->_xp));
        assert(is_finite(target->_P));
// 				cout<<"pass 2"<<endl;

        new_targets.push_back(target);
    }

    return new_targets;
}

void Mht::clusterMerger(vector<MeasurementPtr>& measurements)
{
// 			cout<<"Doing merger"<<endl<<endl;

    //Now i must check if there are measurements with double assignments and merge those clusters

// 			bool changes=true;

// 			while(changes)
// 			{
// 				changes=false;

    vector<MeasurementPtr>::iterator it_m;
    for(it_m=measurements.begin();it_m!=measurements.end();it_m++)
    {
        MeasurementPtr measurement = *it_m;

        //Sort clusters
        sort(measurement->assigned_clusters.begin(),measurement->assigned_clusters.end(),compareClusters);


// 					cout<<endl<<"start: "<<*measurement<<endl;

// 					if(measurement->assigned_clusters.size()<2)
// 						cout<<"nothing to do"<<endl;

        //If this measurement was associated with less that 2 cluster
        if(measurement->assigned_clusters.size()<2)
            continue;


        //The cluster will be the one surviving
        ClusterPtr main_cluster = measurement->assigned_clusters[0];

        //If we are not removing clusters in this step
        if(main_cluster->isEmpty())
            continue;

// 					changes=true;

// 				for(uint i=0;i<measurement->assigned_clusters.size();i++)
// 					cout<<"m: "<<measurement->id<<" c: "<<measurement->assigned_clusters[i]->id<<endl;

// 					cout<<"main: "<<*main_cluster<<endl;

        //Get all hypotheses from this cluster (only current leafs)
        vector<HypothesisPtr> hypotheses_main = main_cluster->assigned_hypotheses;

        assert(hypotheses_main.size()!=0);

// 					cout<<endl<<"main cluster: "<<main_cluster->id<<endl<<endl;

        vector<ClusterPtr>::iterator it_c;
        for(it_c=measurement->assigned_clusters.begin()+1; it_c!=measurement->assigned_clusters.end();it_c++)
        {
            ClusterPtr secondary_cluster = *it_c;

// 					cout<<"\tsecondary: "<<*secondary_cluster<<endl;

// 					cout<<"at startc: "<<*secondary_cluster<<endl;
// 					cout<<"at start: "<<**it_c<<endl;

            //find all hypothesis with this cluster and move their targets, deleting it in the end
// 						vector<HypothesisPtr> hypotheses = secondary_cluster->assigned_hypotheses;

// 						cout<<"moving hypotheses from cluster:"<<*secondary_cluster<<endl;
// 						for(uint g=0;g<secondary_cluster->assigned_hypotheses.size();g++)
// 							cout<<"\t"<<*secondary_cluster->assigned_hypotheses[g]<<endl;
// 						cout<<"to cluster: "<<main_cluster->id<<endl;

            //For all hypotheses to delete

// 						sort(secondary_cluster->assigned_hypotheses.begin(),secondary_cluster->assigned_hypotheses.end(),compareHypothesesByProbabilityDescending);

// 					for(uint h=0;h<1;++h)
            for(uint h=0;h<secondary_cluster->assigned_hypotheses.size();++h)
            {
// 							HypothesisPtr hypothesis = hypotheses[h];

                //Copy targets from this hypothesis to all the main hypotheses, and do not add this hypothesis as a sibling to the main

// 						if(h==0)//Copy just from the most likely hypothesis
// 						{
                //Create copies of the objects in the deleted hypotheses and copy then to the main hypotheses
                for(uint g=0;g<hypotheses_main.size();++g)
                {
                    HypothesisPtr hypothesis_main = hypotheses_main[g];

                    //Create copies of the targets
                    vector<TargetPtr> new_targets = createCopy(secondary_cluster->assigned_hypotheses[h]->_targets);

                    //Insert the new copies in the main hypotheses
                    hypothesis_main->_targets.insert(hypothesis_main->_targets.begin(),new_targets.begin(),new_targets.end());

                    for(uint i=0;i<hypothesis_main->_targets.size();i++)
                    {
                        TargetPtr target = hypothesis_main->_targets[i];

                        target->_cluster = main_cluster->id;

                        assert(is_finite(target->_xp));
                        assert(is_finite(target->_P));
                    }
                }
// 						}

// 						cout<<"hypo: "<<*hypothesis<<endl;
// 						cout<<"moved to : "<<main_cluster->id<<endl;
                secondary_cluster->assigned_hypotheses[h]->_targets.clear();
                secondary_cluster->assigned_hypotheses[h]->_status=DEAD;
// 							secondary_cluster->assigned_hypotheses[h]->_cluster=main_cluster->id;
            }

            //Check all of the targets from the main hypotheses and copy only the most likely from each id
            for(uint mh=0;mh<hypotheses_main.size();mh++)
            {
                vector<TargetPtr> valid_targets;
                vector<TargetPtr> all_targets = hypotheses_main[mh]->_targets;

                //Get the probability for each main target
                for(uint t=0;t<all_targets.size();t++)
                    for(uint h=0;h<secondary_cluster->assigned_hypotheses.size();h++)
                        if(all_targets[t]->_hypothesis==secondary_cluster->assigned_hypotheses[h]->_uid)
                        {
                            all_targets[t]->_aux1=secondary_cluster->assigned_hypotheses[h]->_probability;
                            break;
                        }

                //Get the list of valid targets
                for(uint t=0;t<all_targets.size();t++)
                {
                    bool existing=false;

                    for(uint v=0;v<valid_targets.size();v++)
                    {
                        if(all_targets[t]->_id==valid_targets[v]->_id)
                        {
                            if(all_targets[t]->_aux1>valid_targets[v]->_aux1)
                                valid_targets[v]=all_targets[t];

                            existing=true;
                            break;
                        }
                    }

                    if(!existing)
                        valid_targets.push_back(all_targets[t]);
                }

                hypotheses_main[mh]->_targets=valid_targets;
// 						cout<<"smp cluster: "<<hypotheses_main[mh]->_cluster<<" fc: "<<main_cluster->id<<endl;

                for(uint t=0;t<hypotheses_main[mh]->_targets.size();t++)
                {
                    hypotheses_main[mh]->_targets[t]->_hypothesis=hypotheses_main[mh]->_uid;
                    hypotheses_main[mh]->_targets[t]->_cluster=hypotheses_main[mh]->_cluster;
                }
            }

// 						cout<<endl<<endl;

// 					cout<<"here"<<endl;
// 					cout<<"removingc: "<<*secondary_cluster<<endl;
// 					cout<<"removing: "<<**it_c<<endl;

            //Assign measurements from cl to first
            for(uint im=0;im<secondary_cluster->assigned_measurements.size();im++)
            {
                //Get working measurement
// 							long ma = secondary_cluster->assigned_measurements[im];//ma is the id of a measurement

                //find the measurement with the right id
                MeasurementPtr measurement_move = secondary_cluster->assigned_measurements[im];
                assert(measurement_move!=MeasurementPtr());

// 							cout<<"assigned measurements: "<<*measurement_move<<" to clulster: "<<secondary_cluster->id<<endl;

// 						if(measurement_move->

                //Erase cl from the measurement
// 							removeLocal(measurement_move->assigned_clusters,secondary_cluster);

                if(measurement_move == measurement)//There is a bug i think around here
                    continue;

                //Erase cl from the measurement
                removeLocal(measurement_move->assigned_clusters,secondary_cluster);

// 							cout<<"removed cluster: "<<secondary_cluster->id<<" from measurement: "<<*measurement_move<<endl;

                //If this measurement is not yet assigned to cluster f, add it
                if(find(main_cluster->assigned_measurements.begin(),main_cluster->assigned_measurements.end(),measurement_move)==main_cluster->assigned_measurements.end())//the measurement already belongs to the first
                {
// 								cout<<"adding measurement to f:"<<f<<endl;
                    main_cluster->assigned_measurements.push_back(measurement_move);
                }

// 							cout<<"added measurement: "<<*measurement_move<<" to cluster "<<main_cluster->id<<endl;

                //Add f to the assignment list of measurement ma
                if(find(measurement_move->assigned_clusters.begin(),measurement_move->assigned_clusters.end(),main_cluster)==measurement_move->assigned_clusters.end())
                    measurement_move->assigned_clusters.push_back(main_cluster);

// 						cout<<"added to cluster "<<main_cluster->id;
            }

            secondary_cluster->assigned_measurements.clear();
// 						secondary_cluster->assigned_hypotheses.clear();

// 						cout<<"finished with cluster: "<<*secondary_cluster<<endl;
// 					cout<<"here2"<<endl;

// 						cout<<"removing cluster: "<<secondary_cluster->id<<" from the cluster list"<<endl;
            //Remove cl from the cluster list, this cluster has now been removed from the measurement assigned clusters list

// 						cout<<"removingc: "<<*secondary_cluster<<endl;
// 						cout<<"removing: "<<**it_c<<endl;

// 					it_c = completeRemoveCluster(it_c);
// 					cout<<"removed"<<endl<<endl;

// 					cout<<"pointer: "<<**it_c<<endl;
// 					cout<<endl<<endl;
        }

        measurement->assigned_clusters.clear();
        measurement->assigned_clusters.push_back(main_cluster);

// 					for(uint i=0;i<measurements.size();i++)
// 						cout<<*measurements[i]<<endl;
    }

// 			for(uint i=0;i<_clusters.size();i++)
// 			{
// 				vector<HypothesisPtr> hypotheses = _clusters[i]->assigned_hypotheses;
    //Normalize the probabilities in this cluster
// 				normalize(hypotheses);
// 			}

// 			cout<<"Done merger"<<endl;
// 			}
}

TargetPtr Mht::associate(TargetPtr& target,MeasurementPtr& measurement)
{
    TargetPtr empty;

    //Check if this target is within gating distance
    if(target->getDistance(measurement)>_max_gating)
    {
        //Treat as miss association
        if(target->_missed_associations < _miss_association_threshold)
        {
            //Iterate over failed association
            TargetPtr new_target(new Target(target));
            new_target->_assigned_measurements.clear();

// 					cout<<"n7"<<endl;
// 					assert(is_finite(new_target->_xp));
// 					assert(is_finite(new_target->_P));
// 					cout<<"o7"<<endl;

            return new_target;
        }else
        {
            //Do not add this target to the new hypothesis
            return empty;
        }
    }else//Within max gating, good association
    {
        //Create new target
        TargetPtr new_target(new Target(target,measurement));

// 				cout<<*target<<endl;
// 				cout<<*measurement<<endl;

// 				cout<<"n8"<<endl;
// 				assert(is_finite(new_target->_xp));
// 				assert(is_finite(new_target->_P));
// 				cout<<"o8"<<endl;

        return new_target;
    }

    assert(NULL);
    return empty;//The program will never reach this line (by design)
}

bool Mht::normalize(vector<HypothesisPtr>& hypotheses,uint j)
{
    if(hypotheses.size()==0)
        return false;

    double total=hypotheses.size()>j?j:hypotheses.size();
    double sum=0;

    for(uint i=0;i<total;++i)
        sum+=hypotheses[i]->_probability;

    if(fpclassify(sum) != FP_ZERO)
        for(uint i=0;i<total;++i)
            hypotheses[i]->_probability/=sum;
    else
        for(uint i=0;i<total;++i)
            hypotheses[i]->_probability=1./total;

    return true;
}

void Mht::getProbability(HypothesisPtr hypothesis)
{
    if(hypothesis->_parent_uid!=-1)
    {
        HypothesisTree::iterator it;
        for(it=_hypothesisTree->begin();it!=_hypothesisTree->end();it++)
            if((*it)->_uid==hypothesis->_parent_uid)
                break;

        hypothesis->_probability=getProbability(hypothesis->_n_det,hypothesis->_n_occ,hypothesis->_n_del,hypothesis->_n_new,hypothesis->_n_fal,hypothesis->_prod,(*it)->_probability);
    }else
        hypothesis->_probability=getProbability(hypothesis->_n_det,hypothesis->_n_occ,hypothesis->_n_del,hypothesis->_n_new,hypothesis->_n_fal,hypothesis->_prod,1);
}

double Mht::getProbability(uint n_det, uint n_occ, uint n_del, uint n_new, uint n_fal, double prod, double previous)
{
    //Without normalization, it comes after
    //I'll need the number of previous targets N, number of detected targets Nd, the product of the bivariate
    //distribution of all associations, the probability of the previous hypothesis

    double p_det = 0.8;
    double p_occ = 0.01;
    double p_del = 0.2;

    double lambda_new=0.001;
    double lambda_fal=0.001;

    double p=1;
    p*=pow(p_det,n_det);
    p*=pow(p_occ,n_occ);
    p*=pow(p_del,n_del);
    p*=pow(lambda_new,n_new);
    p*=pow(lambda_fal,n_fal);
    p*=prod;
    p*=previous;

// 			cout<<"Calculating probability of hypothesis"<<endl;
// 			cout<< str(boost::format("Pdet: %1.5f | Pocc: %1.5f | Pdel: %1.5f | lnew: %1.5f | lfal: %1.5f") % p_det % p_occ % p_del % lambda_new % lambda_fal)<<endl;

// 			cout<< str(boost::format("Ndet: %7d | Nocc: %7d | Ndel: %7f | nnew: %7f | nfal: %7f") % n_det % n_occ % n_del % n_new % n_fal)<<endl;

// 			cout<<"Prod: "<<prod<<" | Prev: "<<previous<<endl;
// 			cout<<"pout: "<<p<<endl;

    return p;
}

HypothesisPtr Mht::getHypothesisPtr(long id,ClusterPtr& cluster)
{
    for(uint h=0;h<cluster->assigned_hypotheses.size();h++)
        if(cluster->assigned_hypotheses[h]->_uid==id)
            return cluster->assigned_hypotheses[h];
    return HypothesisPtr();
}

void Mht::updateTargetList(void)
{
    ldb()<<"updating targets"<<endl;
    _current_targets.clear();

    //Run through all clusters, and for each extract the targets from the most probable hypothesis
    for(uint i=0;i<_clusters.size();i++)
    {
        assert(_clusters[i]->assigned_hypotheses.size()!=0);

        //Sort the list, this should not be needed because the hypotheses should already be sorted
        sort(_clusters[i]->assigned_hypotheses.begin(),_clusters[i]->assigned_hypotheses.end(),compareHypothesesByProbabilityDescending);

        //Check if for some reason the hypothesis pointer is null
        assert(_clusters[i]->assigned_hypotheses[0]!=0);

// 				cout<<"cluster: "<<_clusters[i]->_id<<" number of hypotheses: "<<_clusters[i]->_id
        ldb()<<"cluster: "<<_clusters[i]->id<<" nMeasurements: "<<_clusters[i]->assigned_measurements.size()<<endl;

        for(uint h=0;h<_clusters[i]->assigned_hypotheses.size();h++)
        {
            ldb()<<"\thypothesis: "<<_clusters[i]->assigned_hypotheses[h]->_uid<<" p: "<<_clusters[i]->assigned_hypotheses[h]->_probability<<endl;
            for(uint t=0;t<_clusters[i]->assigned_hypotheses[h]->_targets.size();t++)
                ldb()<<"\t\ttarget: "<<_clusters[i]->assigned_hypotheses[h]->_targets[t]->_id<<" ("<<_clusters[i]->assigned_hypotheses[h]->_targets[t]->_uid<<")"<<endl;
        }
        ldb()<<endl;
        //Extract the targets from the first hypotheses and put them in the main list
        _current_targets.insert(_current_targets.begin(),_clusters[i]->assigned_hypotheses[0]->_targets.begin(),_clusters[i]->assigned_hypotheses[0]->_targets.end());
    }

    vector<TargetPtr> valid_targets;

    for(uint t=0;t<_current_targets.size();t++)
    {
        bool valid=true;

        TargetPtr target = _current_targets[t];

// 		if(target->_assigned_measurements.size()==0)
// 		{
// 			cout<<"No measurements on target: "<<*target<<endl;
// 			exit(0);
// 		}

// 				MeasurementPtr measurement = target->_assigned_measurements[0];

        for(uint vt=0;vt<valid_targets.size();vt++)
            if(valid_targets[vt]->_id==target->_id)
                valid=false;

        for(uint vt=0;vt<valid_targets.size();vt++)
            if(valid_targets[vt]->_uid==target->_uid)
                valid=false;

// 				if(measurement==NULL)
// 					valid=false;

        if(valid)
            valid_targets.push_back(target);
    }

    _current_targets=valid_targets;

    sort(_current_targets.begin(),_current_targets.end(),compareTargetsById);

    ldb()<<endl;
}

void Mht::checkCurrentTargets(vector<MeasurementPtr>& measurements)
{
    string report_name;

    //Total number of created targets

    ofstream report_total_targets;
    report_name = getReportName("_total");

    report_total_targets .open(report_name.c_str(),ios::app);

    ldb(0)<<"total report file: "<<report_name<<endl;

    report_total_targets <<"#"<<endl;
    report_total_targets <<"i "<<iteration<<endl;
    report_total_targets <<Target::_ntotal<<endl;

    report_total_targets.close();


    //Distance to correct measurement

    ofstream report_distance;
    report_name = getReportName("_dist");

    report_distance.open(report_name.c_str(),ios::app);

    ldb(0)<<"distance report file: "<<report_name<<endl;

    report_distance<<"#"<<endl;
    report_distance<<"i "<<iteration<<endl;

    for(uint t=0;t<_current_targets.size();t++)
    {
        TargetPtr target = _current_targets[t];

        bool found=false;

        report_distance<<target->_id;

        for(uint m=0;m<measurements.size();m++)
        {
            MeasurementPtr measurement = measurements[m];

            if(target->_exuid==measurement->id)//Found a matching measurement
            {
                double distance = target->getDistance(measurement,2);//Get euclidean distance

                report_distance<<" "<<distance<<endl;

                found=true;
                break;
            }
        }

        if(!found)
        {
            report_distance<<" -1"<<endl;
        }
    }

    report_distance.close();

    //Associations report
    //Do not count if the measurement was not present

    ofstream report;
    report_name = getReportName("");

    report.open(report_name.c_str(),ios::app);

    ldb(0)<<"data file: "<<fileName<<endl;
    ldb(0)<<"report file: "<<report_name<<endl;

    report<<"#"<<endl;
    report<<"i "<<iteration<<endl;

    for(uint t=0;t<_current_targets.size();t++)
    {
        TargetPtr target = _current_targets[t];

        report<<target->_id;

        if(target->_assigned_measurements.size()==0)
        {
            bool found = false;
            for(uint m=0;m<measurements.size();m++)
                if(measurements[m]->id==target->_exuid)
                {
                    found=true;
                    break;
                }

            if(found)
            {
                cout<<"Error!!, target: "<<target->_id<<" no assigned measurement"<<endl;
                report<<" nok"<<endl;
            }else
                report<<" ok"<<endl;//Mark targets not found in the measurement lists as ok

        }else if(target->_exuid != target->_assigned_measurements[0]->id)
        {
            cout<<"Error!!, target("<<target->_id<<"): "<<target->_exuid<<" with measurement: "<<target->_assigned_measurements[0]->id<<endl;
            report<<" nok"<<endl;
        }else
        {
            report<<" ok"<<endl;
        }

        assert(is_finite(target->_xp));
        assert(is_finite(target->_P));
    }

    report.close();
}