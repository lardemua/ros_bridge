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
\brief K-best Murtys' algorithm s
*/

// System Includes
#include <carla_ros_mtt/k_best.h>


Timer t;

int factorial(int x)
{
    return (x == 1 ? x : x * factorial(x - 1));
}

vector<vector<int> > convertToStdVector(MatrixXd&mat)
{
    vector<int> line;//define a working line
    vector<vector<int> > out_mat;//define the output matrix

    line.resize(mat.cols(),-1);//resize the line to have the same number of columns as the input matrix
    out_mat.resize(mat.rows(),line);//resize the matrix to have the same number of lines as the input matrix

    for(uint l=0;l<mat.rows();++l)
        for(uint c=0;c<mat.cols();++c)
            out_mat[l][c]=mat(l,c);

    return out_mat;
}

orderedPairPtr makeOrderedPair(int row,int col)
{
    orderedPairPtr op(new orderedPair);
    op->row=row;
    op->col=col;

    return op;
}

double munkers_wrapper(MatrixXd&cost_mat,vector<orderedPairPtr>&assignments)
{
    // Apply Munkres algorithm to matrix.
    Munkres m;
    double cost = m.solve(cost_mat,assignments);

    return cost;
}

ostream &operator<<(ostream &o,orderedPairPtr& p)
{
    o<<"("<<p->row+1<<", "<<p->col+1<<") ";
    return o;
}

ostream &operator<<(ostream &o,vector<orderedPairPtr>& op)
{
    if(op.size()==0)
        o<<"[]";

    for(uint i=0;i<op.size();++i)
    {
        o<<op[i];

        if(i<op.size()-1)
            o<<", ";
    }

    return o;
}

ostream &operator<<(ostream &o,vector<AssignmentsPtr>& assing)
{
    if(assing.size()==0)
        o<<"[]";
    for(uint i=0;i<assing.size();++i)
    {
        o<<"i: "<<i<<" cost: "<<assing[i]->cost<<" ";
        o<<assing[i]->pairs<<endl;
    }

    return o;
}


ostream &operator<<(ostream &o,NodePtr& n)
{
    o<<*n;
// 	o<<"cost: "<<n->cost<<endl;
// 	o<<"fixed: "<<n->fixed<<endl;
// 	o<<"excluded: "<<n->excluded<<endl;
// 	o<<"unspecific: "<<n->unspecific<<endl;

    return o;
}

ostream &operator<<(ostream &o,Node& n)
{
    o<<"cost: "<<n.cost<<endl;
    o<<"fixed: "<<n.fixed<<endl;
    o<<"excluded: "<<n.excluded<<endl;
    o<<"unspecific: "<<n.unspecific<<endl;

    return o;
}


ostream &operator<<(ostream &o,vector<NodePtr>& n)
{
    cout<<"*****************************************"<<endl;
    for(uint i=0;i<n.size();++i)
    {
        o<<"cost: "<<n[i]->cost<<endl;
        o<<"fixed: "<<n[i]->fixed<<endl;
        o<<"excluded: "<<n[i]->excluded<<endl;
        o<<"unspecific: "<<n[i]->unspecific<<endl;

        if(i<n.size()-1)
            cout<<endl;
    }
    cout<<"*****************************************"<<endl;
    return o;
}

vector<NodePtr>& operator+=(vector<NodePtr>& a,vector<NodePtr>& b)
{
    a.insert(a.begin(),b.begin(),b.end());
    return a;
}

int test(void)
{
    vector<AssignmentsPtr> assignments;

// 	MatrixXd cost_mat(10,10);
// 	MatrixXd cost_mat(2,2);
// 	MatrixXd cost_mat(4,4);
// 	MatrixXd cost_mat(3,2);
    MatrixXd cost_mat(5,2);
// 	MatrixXd cost_mat(2,5);
// 	MatrixXd cost_mat(3,3);

// 	cost_mat<<  8, 3, 3,
// 				4, 2, 55,
// 				57, 3, 5;
//
// 	cost_mat<<  0.1054, 1.6097, 2.3026, 0.1,
// 				1.6094, 1.2040, 0.3567, 0.5,
// 				0.5108, 0.3567, 1.6094, 0.3,
// 				0.5   , 0.1	  , 0.8   , 0.9;

// 	cost_mat<<  0.1054, 1.6097,
// 				1.6094, 1.2040,
// 				0.5108, 0.3567,
// 				0.5   , 0.1	  ;

// 	cost_mat<<  0.1, 1.6,
// 				1.6, 0.2,
// 				0.5, 0.3;

    cost_mat<<  0.1, 0.1,
            1.6, 0.4,
            0.5, 0.7,
            0.01, 2.,
            0.0, 0.54 ;


// 	cost_mat<<  0.1,
// 				1.6,
// 				0.5,
// 				0.01,
// 				0.0;

// 	cost_mat<<  1.1,	1.6,	0.5,	0.01,	0.0,
// 				0.1,	0.4,	0.7,	2.0,	0.54;

// 	cost_mat<<  0.1054, 1.6097, 2.3026,
// 				1.6094, 0.7040, 0.3567,
// 				0.5108, 0.3567, 0.8   ;
//

// 	cost_mat<< 	4.170220e-01, 4.191945e-01, 8.007446e-01, 9.834683e-02, 9.888611e-01, 1.936696e-02, 1.023344e-01, 9.034019e-01, 8.833061e-01, 1.147460e-01,
// 7.203245e-01, 6.852195e-01, 9.682616e-01, 4.211076e-01, 7.481657e-01, 6.788355e-01, 4.140560e-01, 1.374747e-01, 6.236722e-01, 9.494893e-01,
// 1.143748e-04, 2.044522e-01, 3.134242e-01, 9.578895e-01, 2.804440e-01, 2.116281e-01, 6.944002e-01, 1.392763e-01, 7.509424e-01, 4.499121e-01,
// 3.023326e-01, 8.781174e-01, 6.923226e-01, 5.331653e-01, 7.892793e-01, 2.655467e-01, 4.141793e-01, 8.073913e-01, 3.488983e-01, 5.783896e-01,
// 1.467559e-01, 2.738759e-02, 8.763892e-01, 6.918771e-01, 1.032260e-01, 4.915732e-01, 4.995346e-02, 3.976768e-01, 2.699279e-01, 4.081368e-01,
// 9.233859e-02, 6.704675e-01, 8.946067e-01, 3.155156e-01, 4.478935e-01, 5.336255e-02, 5.358964e-01, 1.653542e-01, 8.958862e-01, 2.370270e-01,
// 1.862602e-01, 4.173048e-01, 8.504421e-02, 6.865009e-01, 9.085955e-01, 5.741176e-01, 6.637946e-01, 9.275086e-01, 4.280912e-01, 9.033795e-01,
// 3.455607e-01, 5.586898e-01, 3.905478e-02, 8.346257e-01, 2.936141e-01, 1.467286e-01, 5.148891e-01, 3.477659e-01, 9.648400e-01, 5.736795e-01,
// 3.967675e-01, 1.403869e-01, 1.698304e-01, 1.828828e-02, 2.877753e-01, 5.893055e-01, 9.445948e-01, 7.508121e-01, 6.634415e-01, 2.870327e-03,
// 5.388167e-01, 1.981015e-01, 8.781425e-01, 7.501443e-01, 1.300286e-01, 6.997584e-01, 5.865550e-01, 7.259980e-01, 6.216957e-01, 6.171449e-01;


// 	cost_mat<< 	4.170220e-01, 4.191945e-01, 8.007446e-01, 9.834683e-02, 9.888611e-01, 1.936696e-02, 1.023344e-01, 9.034019e-01, 8.833061e-01, 1.147460e-01,
// 7.203245e-01, 6.852195e-01, 9.682616e-01, 4.211076e-01, 7.481657e-01, 6.788355e-01, 4.140560e-01, 1.374747e-01, 6.236722e-01, 9.494893e-01,
// 1.143748e-04, 2.044522e-01, 3.134242e-01, 9.578895e-01, 2.804440e-01, 2.116281e-01, 6.944002e-01, 1.392763e-01, 7.509424e-01, 4.499121e-01,
// 3.023326e-01, 8.781174e-01, 6.923226e-01, 5.331653e-01, 7.892793e-01, 2.655467e-01, 4.141793e-01, 8.073913e-01, 3.488983e-01, 5.783896e-01,
// 1.467559e-01, 2.738759e-02, 8.763892e-01, 6.918771e-01, 1.032260e-01, 4.915732e-01, 4.995346e-02, 3.976768e-01, 2.699279e-01, 4.081368e-01,
// 9.233859e-02, 6.704675e-01, 8.946067e-01, 3.155156e-01, 4.478935e-01, 5.336255e-02, 5.358964e-01, 1.653542e-01, 8.958862e-01, 2.370270e-01,
// 1.862602e-01, 4.173048e-01, 8.504421e-02, 6.865009e-01, 9.085955e-01, 5.741176e-01, 6.637946e-01, 9.275086e-01, 4.280912e-01, 9.033795e-01,
// 3.455607e-01, 5.586898e-01, 3.905478e-02, 8.346257e-01, 2.936141e-01, 1.467286e-01, 5.148891e-01, 3.477659e-01, 9.648400e-01, 5.736795e-01,
// 3.967675e-01, 1.403869e-01, 1.698304e-01, 1.828828e-02, 2.877753e-01, 5.893055e-01, 9.445948e-01, 7.508121e-01, 6.634415e-01, 2.870327e-03,
// 5.388167e-01, 1.981015e-01, 8.781425e-01, 7.501443e-01, 1.300286e-01, 6.997584e-01, 5.865550e-01, 7.259980e-01, 6.216957e-01, 6.171449e-01;

// 	int right_cols = 2;

// 	cost_mat.rightCols(right_cols)=MatrixXd::Constant(cost_mat.rows(),right_cols,-log(0));

    cout<<"Cost mat:"<<endl<<cost_mat<<endl<<endl;



// 	vector<orderedPairPtr> single_assignments;

    // Apply Munkres algorithm to matrix.
// 	Munkres m;
// 	double cost = m.solve(cost_mat,single_assignments);

// 	cout<<"cost: "<<cost<<endl;
// 	cout<<single_assignments<<endl;
// 	cout<<"FINAL"<<endl;

// 	return 0;

// 	for(uint c=1;c<cost_mat.cols();++c)


// 	cost_mat<< 	-1., 2.,
// 				2., 1.2;

// 	uint k=1088640;
    uint k=2000;

    t.tic();

    assignments = k_best_assignment(cost_mat,k);

    t.toc();

// 	cout<<"print assignments:"<<endl<<endl;
    cout<<assignments<<endl;

    cout<<"Time for code was "<< t.value() <<" seconds"<<endl;

// 	cout<<"done"<<endl;

    return 0;
}

//assignment
vector<AssignmentsPtr> k_best_assignment(MatrixXd& cost_mat,uint k)
{
    //Output variable
    vector<AssignmentsPtr> assignments_list;

    //Declare the assignments
    vector<orderedPairPtr> assignments;
    //Calculate the first best assignment
    double cost = munkers_wrapper(cost_mat,assignments);

// 	cout<<"here0"<<endl;

    //Declare an auxiliary node ptr
    NodePtr node_aux(new Node);

    //Put the best assignment in it
    node_aux->unspecific = assignments;
    node_aux->cost=cost;

    //Declare a node and add the calculated assignment as top layer
    vector<NodePtr> optimal_nodes;
    optimal_nodes.push_back(node_aux);

// 	cout<<"here2"<<endl;

// 	cout<<"optimal nodes:\n"<<optimal_nodes<<endl;

    //List of nodes to do the partitioning
    vector<NodePtr> nodes_list = partitionNode(*node_aux,cost_mat);

// 	cout<<"node list:\n"<<nodes_list<<endl;

    unsigned long max;

    uint r = cost_mat.rows();
    uint c = cost_mat.cols();

// 	cout<<"here1"<<endl;

    if(r<10 && c <10)
    {
        if(r==c)
        {
            max=factorial(r);

        }else
        {
// 			long c_max=1;
// 			long r_max=1;

            max=1;

            for(uint i=r;i>0;i--)
            {
                max*=i;
                if(max>k)
                    break;
            }

            max=r>c?factorial(r)/factorial(r-c):factorial(c)/factorial(c-r);
        }
    }else//the maximum number of hypothesis is too big to calculate and it is also not usefull
        max=100;

    if(k>max)
        k=max;

// 	cout<<"k"<<k<<endl;

    while(optimal_nodes.size()<k)
    {
        double min_cost=1e10;
        uint ind=-1;

        for(uint i=0;i<nodes_list.size();++i)
            if(nodes_list[i]->cost<min_cost)
            {
                min_cost=nodes_list[i]->cost;
                ind=i;
            }

        optimal_nodes.push_back(nodes_list[ind]);

        vector<NodePtr> nodes_tmp = partitionNode(*nodes_list[ind],cost_mat);

        nodes_list.erase(nodes_list.begin()+ind);
        nodes_list+=nodes_tmp;
    }

// 	cout<<"stuff"<<endl;

    for(uint ii=0;ii<optimal_nodes.size();++ii)
    {
        AssignmentsPtr assign(new Assignments);

        for(uint i=0;i<optimal_nodes[ii]->fixed.size();++i)
        {
            orderedPairPtr pair(new orderedPair);
            pair->row=optimal_nodes[ii]->fixed[i]->row;
            pair->col=optimal_nodes[ii]->fixed[i]->col;

            assign->pairs.push_back(pair);
        }

        for(uint i=0;i<optimal_nodes[ii]->unspecific.size();++i)
        {
            orderedPairPtr pair(new orderedPair);
            pair->row=optimal_nodes[ii]->unspecific[i]->row;
            pair->col=optimal_nodes[ii]->unspecific[i]->col;

            assign->pairs.push_back(pair);
        }

        assign->cost=optimal_nodes[ii]->cost;
        assignments_list.push_back(assign);
    }

// 	cout<<"stuff out"<<endl;

    return assignments_list;
}

vector<NodePtr> partitionNode(Node&node_in,MatrixXd&cost_mat)
{
    //Unspecific unspecific assignments
    vector<NodePtr> nodes_out;

    //Declare a temporary node
    NodePtr node_out_tmp(new Node);

    bool non_square_matrix=false;

    if(cost_mat.rows()!=cost_mat.cols())
        non_square_matrix=true;

// 	cout<<"Node in: "<<node_in<<endl;

// 	cout<<"iterate under the number of unspecific assignments"<<endl;

    if(!non_square_matrix)
    {
        //Iterate u times to permute help out assignments
        for(uint ii=0;ii<node_in.unspecific.size()-1;++ii)
        {
            // 		cout<<"iteration: "<<ii<<endl;
            //Keep fixed assignments
            node_out_tmp->fixed = node_in.fixed;

            // 		cout<<"tmp fixed: "<<node_out_tmp->fixed<<endl;

            //Keep excluded assignments
            node_out_tmp->excluded = node_in.excluded;

            // 		cout<<"tmp excluded: "<<node_out_tmp->excluded<<endl;

            //If there are unspecific assignments, permute and add to list of exclusions
            if(ii>0)
            {
                //Add that section to the fixed assignments
                node_out_tmp->fixed.insert(node_out_tmp->fixed.begin(),node_in.unspecific.begin(),node_in.unspecific.begin()+ii);
            }

            // 		cout<<"tmp fixed after uns: "<<node_out_tmp->fixed<<endl;

            //If there are unspecific assignment, permute and add to list of exclusions

            //Get the unspecific part of the input node
            //Add it has an excluded assignment
            node_out_tmp->excluded.push_back(node_in.unspecific[ii]);

            // 		cout<<"node_out_tmp->excluded: "<<node_out_tmp->excluded<<endl;

            //Add the rest of the unspecific
            node_out_tmp->unspecific.clear();

            node_out_tmp->unspecific.insert(node_out_tmp->unspecific.begin(),node_in.unspecific.begin()+ii+1,node_in.unspecific.end());

            // 		cout<<"node_out_tmp->unspecific: "<<node_out_tmp->unspecific<<endl;

// 			cout<<"node tmp"<<endl<<node_out_tmp<<endl;

            NodePtr node_out_tmp2 = calcNodeCost(*node_out_tmp,cost_mat,non_square_matrix);

            // 		cout<<"calc node cost: "<<node_out_tmp2<<endl;

            nodes_out.push_back(node_out_tmp2);
        }
    }else
    {
        //Iterate u times to permute help out assignments
        for(uint ii=0;ii<node_in.unspecific.size();++ii)
        {
            // 		cout<<"iteration: "<<ii<<endl;
            //Keep fixed assignments
            node_out_tmp->fixed = node_in.fixed;

            // 		cout<<"tmp fixed: "<<node_out_tmp->fixed<<endl;

            //Keep excluded assignments
            node_out_tmp->excluded = node_in.excluded;

            // 		cout<<"tmp excluded: "<<node_out_tmp->excluded<<endl;

            //If there are unspecific assignments, permute and add to list of exclusions
            if(ii>0)
            {
                //Add that section to the fixed assignments
                node_out_tmp->fixed.insert(node_out_tmp->fixed.begin(),node_in.unspecific.begin(),node_in.unspecific.begin()+ii);
            }

            // 		cout<<"tmp fixed after uns: "<<node_out_tmp->fixed<<endl;

            //If there are unspecific assignment, permute and add to list of exclusions

            //Get the unspecific part of the input node
            //Add it has an excluded assignment
            node_out_tmp->excluded.push_back(node_in.unspecific[ii]);

            // 		cout<<"node_out_tmp->excluded: "<<node_out_tmp->excluded<<endl;

            //Add the rest of the unspecific
            node_out_tmp->unspecific.clear();

            if(ii<node_in.unspecific.size()-1)
                node_out_tmp->unspecific.insert(node_out_tmp->unspecific.begin(),node_in.unspecific.begin()+ii+1,node_in.unspecific.end());

            // 		cout<<"node_out_tmp->unspecific: "<<node_out_tmp->unspecific<<endl;

// 			cout<<"node tmp"<<endl<<node_out_tmp<<endl;

            NodePtr node_out_tmp2 = calcNodeCost(*node_out_tmp,cost_mat,non_square_matrix);

            // 		cout<<"calc node cost: "<<node_out_tmp2<<endl;

            nodes_out.push_back(node_out_tmp2);
        }
    }


    if(nodes_out.size()==0)
    {
        NodePtr empty(new Node);
        empty->cost=1e12;

        nodes_out.push_back(empty);
    }

// 	cout<<"nodes out:\n"<<nodes_out<<endl;

    return nodes_out;
}

NodePtr calcNodeCost(Node&node_in,MatrixXd&cost_mat,bool non_square_matrix)
{
// 	cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<<endl;

    vector<orderedPairPtr> fixed = node_in.fixed;
    vector<orderedPairPtr> excluded = node_in.excluded;

    MatrixXd mask(cost_mat.rows(),cost_mat.cols());
    mask = MatrixXd::Zero(cost_mat.rows(),cost_mat.cols());

    double cost_sum = 0;
    double mask_num = 1e12;

// 	cout<<"calc node cost"<<endl;
// 	cout<<"fixed:"<<fixed<<endl;

// 	cout<<"mask:\n"<<mask<<endl;

    for(uint ii=0;ii<fixed.size();++ii)
    {
// 		cout<<"R:"<<fixed[ii]->row<<", C:"<<fixed[ii]->col<<endl;
        cost_sum += cost_mat(fixed[ii]->row,fixed[ii]->col);

// 		cout<<"Cost sum:"<<cost_sum<<endl;

        //Mask out all entries in same rows/cols as fixed assignments
// 		cout<<"r:"<<mask.row(fixed[ii]->row)<<endl;
// 		cout<<"rn:"<<MatrixXd::Constant(1,mask.rows(),mask_num)<<endl;

        mask.row(fixed[ii]->row) = MatrixXd::Constant(1,mask.cols(),mask_num);
        mask.col(fixed[ii]->col) = MatrixXd::Constant(mask.rows(),1,mask_num);
    }

// 	cout<<"mask after fixed:"<<mask<<endl;

    //Add excluded assignments to mask_num
    for(uint kk=0;kk<excluded.size();++kk)
        mask(excluded[kk]->row,excluded[kk]->col) = mask_num;

// 	cout<<"mask after excluded:"<<mask<<endl;

    //Get list of row, col where mask is 0
    vector<orderedPairPtr> zeros;

    for(uint r=0;r<mask.rows();++r)
        for(uint c=0;c<mask.cols();++c)
            if(mask(r,c)==0)
            {
                orderedPairPtr pair(new orderedPair);
                pair->row=r;
                pair->col=c;

                zeros.push_back(pair);
            }

// 	cout<<"mask after excluded:"<<mask<<endl;

    //Obtain a vector of rows and cols of zeros values the mask
    vector<int> rows=getRows(zeros);
    vector<int> cols=getCols(zeros);

    //Sort the rows and cols
    sort(rows.begin(),rows.end());
    sort(cols.begin(),cols.end());

    NodePtr node_out(new Node);

    //Check if the number of repeating values in the cols and rows is the same
    if(countRepeatingValues<int>(rows)==countRepeatingValues<int>(cols) && !non_square_matrix)
    {
        //Declare the assignments
        vector<orderedPairPtr> assignments;

        //Calculate optimal assignment using masked cost matrix
        mask+=cost_mat;

        double cost = munkers_wrapper(mask,assignments);

        for(uint f=0;f<fixed.size();++f)
            for(uint a=0;a<assignments.size();++a)
                if(assignments[a]->row==fixed[f]->row || assignments[a]->col==fixed[f]->col)
                {
                    cost-=mask(assignments[a]->row,assignments[a]->col);
                    assignments.erase(assignments.begin()+a);
                    a=a-1;
                }

        *node_out = node_in;

        //Copy unspecific assignments
        node_out->unspecific = assignments;

        //Add the sum of the costs from fixed assignments and the rest
        node_out->cost=cost_sum + cost;

    }else if(!non_square_matrix) //Degenerate state
    {
// 		cout<<"Degenerate case"<<endl;
        node_out->cost=1e12;
    }else
    {
        //This case happens in non square matrices

        //Declare the assignments
        vector<orderedPairPtr> assignments;

        //Calculate optimal assignment using masked cost matrix
        mask+=cost_mat;

        double cost = munkers_wrapper(mask,assignments);

        for(uint f=0;f<fixed.size();++f)
            for(uint a=0;a<assignments.size();++a)
                if(assignments[a]->row==fixed[f]->row || assignments[a]->col==fixed[f]->col)
                {
                    cost-=mask(assignments[a]->row,assignments[a]->col);
                    assignments.erase(assignments.begin()+a);
                    a=a-1;
                }

        *node_out = node_in;

        //Copy unspecific assignments
        node_out->unspecific = assignments;

        //Add the sum of the costs from fixed assignments and the rest
        node_out->cost = cost_sum + cost;

// 		cout<<"assignments:"<<assignments<<endl;
// 		cout<<"assignments.size():"<<assignments.size()<<endl;
// 		cout<<"cost mat:"<<cost_mat<<endl;

        uint total = assignments.size()+fixed.size();
        //Strange degenerate case, where we have too few assignments
        if(total < cost_mat.rows() && total <cost_mat.cols())
            node_out->cost=1e12;
    }

// 	cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    return node_out;
}

///Only works on sorted vectors
template<class type>
uint countRepeatingValues(vector<type> sorted_values)
{
    uint repetitions=0;

    if(sorted_values.size()>1)
        for(uint i=1;i<sorted_values.size();++i)
            if(sorted_values[i-1]==sorted_values[i])
                repetitions++;
    return repetitions;
}

vector<int> getRows(vector<orderedPairPtr>& pairs)
{
    vector<int> rows;

    for(uint s=0;s<pairs.size();++s)
        rows.push_back(pairs[s]->row);

    return rows;
}

vector<int> getCols(vector<orderedPairPtr>& pairs)
{
    vector<int> cols;

    for(uint s=0;s<pairs.size();++s)
        cols.push_back(pairs[s]->col);

    return cols;
}

bool compareOrdered_pair(orderedPairPtr& p1,orderedPairPtr& p2)
{
    return true;
