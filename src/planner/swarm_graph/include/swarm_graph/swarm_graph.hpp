#ifndef _SWARM_GRAPH_H_
#define _SWARM_GRAPH_H_

#include <memory>
#include <Eigen/Eigen>
#include <ros/ros.h>


class SwarmGraph{
    
private:
    std::vector<Eigen::Vector3d> nodes;  //Contains the pos of the vertices in swarm_graph
    std::vector<Eigen::Vector3d> nodes_des;  //Desired graph (permutation could change after assignment)
    std::vector<Eigen::Vector3d> nodes_des_init; //Initial desired graph (never change once set)

    std::vector<Eigen::Vector3d> agent_grad; //Contains the gradp of the swarm_graph

    bool have_desired;

    Eigen::MatrixXd A;   //Adjacency matrix
    Eigen::VectorXd D;   //Degree matrix
    Eigen::MatrixXd Lhat; //Symmetric Normed Laplacian

    Eigen::MatrixXd A_des;   //Desired adjacency matrix 
    Eigen::VectorXd D_des;   //Desired degree matrix
    Eigen::MatrixXd Lhat_des;  //Desired SNL 
    
    Eigen::MatrixXd DLhat; //Difference of SNL

public:

    SwarmGraph(); 
    ~SwarmGraph(){}

    //Update the nodes, feature matrices & desired matrices
    bool updateGraph( const std::vector<Eigen::Vector3d> &swarm); 

    //Set desired swarm nodes
    bool setDesiredForm( const std::vector<Eigen::Vector3d> &swarm_des );

    //Calculate the graph feature matices
    bool calcMatrices( const std::vector<Eigen::Vector3d> &swarm,
                       Eigen::MatrixXd &Adj, Eigen::VectorXd &Deg,
                       Eigen::MatrixXd &SNL);

    //E-norm distance
    double calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);                   

    //Calculate the squared F-Normed difference of SNL matrix
    bool calcFNorm2( double &cost ); 

    //Calculate the gradients over positions
    bool calcFGrad( Eigen::Vector3d &gradp, int idx );

    //Get the id_th gradient over position
    Eigen::Vector3d getGrad(int id);
    bool getGrad(std::vector<Eigen::Vector3d> &swarm_grad);

    //Helper functions
    std::vector<Eigen::Vector3d> getDesNodesInit() { return nodes_des_init; }
    std::vector<Eigen::Vector3d> getDesNodesCur() { return nodes_des; }

    typedef std::unique_ptr<SwarmGraph> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

#endif