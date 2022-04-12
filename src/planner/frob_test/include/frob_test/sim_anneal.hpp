#ifndef SIM_ANNEAL_HPP
#define SIM_ANNEAL_HPP

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <frob_test/swarm_graph.hpp>
#include <ctime>

class SimAnneal{

private: 

    Eigen::VectorXi as; //Current assignment
    Eigen::VectorXi best_as; //current best assignment
    Eigen::VectorXi last_as; //last best assignment

    Eigen::MatrixXd L_des;  //Desired Laplacian
    Eigen::MatrixXd L_init;  //Laplacian of current assignment

    int swarm_size; 
    bool have_assignment; //Flag: false if first assignment

    //Parameters for SA
    int markov_length;
    double decay_scale;
    double as_error;
    double T_init;
    double T_end;

public:
    SimAnneal();
    ~SimAnneal(){}

    void test( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr );

    //Generate a new assignment from solution space for next iteration
    void genNewSolution();

    //Swap the concerned assignment & matrices
    void swapVariables(int &id1, int &id2);

    //Assignment initialization
    void initAssignment();
    
    //Generate swap_id combinations for initial trial
    void genCombns(std::vector<std::pair<int, int> > &combns);

    void getCost(double &cost_sim, double &cost_as, double &total_cost); //Calculate current cost

    //Run the Simulated Annealing
    // void runSA( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr );
    void runSA( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr, bool &as_flag, std::vector<int> &assignment_vec );

    //Adjust the nodes permutation according to the given assignment, return a transformed one
    std::vector<Eigen::Vector3d> transformAssign(std::vector<Eigen::Vector3d> &swarm, const Eigen::VectorXi &as);
    
    //Calculate Laplacian matrix
    void calcLap(const std::vector<Eigen::Vector3d> &swarm, Eigen::MatrixXd &SNL);

    //Return E-norm
    double calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2);

    template<typename AnyType> //swap
    void swap( AnyType& a, AnyType& b )
    {
        AnyType Temp;
        Temp = a; a = b; b = Temp;
    }
    
};

#endif