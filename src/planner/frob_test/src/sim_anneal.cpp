#include <frob_test/sim_anneal.hpp>

SimAnneal::SimAnneal()
{
    //Init parameters
    markov_length = 150;
    decay_scale = 0.8;
    T_init = 0.02;
    T_end = 0.0005;
    as_error = 0.03;

    have_assignment = false;
}

void SimAnneal::calcLap( const std::vector<Eigen::Vector3d> &swarm,  Eigen::MatrixXd &SNL )
{
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero( swarm.size(), swarm.size() );
    Eigen::VectorXd Deg = Eigen::VectorXd::Zero( swarm.size() );
    SNL = Eigen::MatrixXd::Zero( swarm.size(), swarm.size() );

    //Adjacency and Degree
    for( int i = 0; i < swarm.size(); i++ ){
        for( int j = 0; j < swarm.size(); j++ ){
            Adj(i,j) = calcDist2( swarm[i], swarm[j] );
            Deg(i) += Adj(i,j);
        }
    }

    //Get SNL
    for( int i = 0; i < swarm.size(); i++ ){
        for( int j = 0; j < swarm.size(); j++ ){
            if( i == j){
                SNL(i,j) = 1;
            }else{
                SNL(i,j) = -Adj(i,j) * pow(Deg(i),-0.5) * pow(Deg(j),-0.5);
            }
        }
    }

}

void SimAnneal::getCost(double &cost_sim, double &cost_as, double &cost_total)
{   
    //Graph similarity part & assignment difference part
    cost_sim = (L_init - L_des).cwiseAbs2().sum();
    cost_as = 0;
    for( int i = 0; i < as.size(); i++ )
    {
        if(as(i) != last_as(i))
            cost_as += as_error;
    }
    cost_total = cost_as + cost_sim;
}


double SimAnneal::calcDist2( const Eigen::Vector3d &v1, const Eigen::Vector3d &v2 )
{
    return (v1-v2).cwiseAbs2().sum();
}


void SimAnneal::genNewSolution()
{
    //Generate a random swap
    int id1, id2;
    do {
        id1 = rand()%swarm_size;
        id2 = rand()%swarm_size;
    }while( id1 == id2 );
    
    //Swap
    swapVariables( id1, id2 );
}

void SimAnneal::swapVariables(int &id1, int &id2)
{
    //Swap the assignment vector
    swap( as(id1), as(id2) );

    //Swap the L_des matrix
    for( int i = 0; i < swarm_size; i++ )
    {
        if( i!=id1 && i!=id2 ){
            swap( L_des(id1,i), L_des(id2,i) );
            swap( L_des(i,id1), L_des(i,id2) );
        }
    }
}

void SimAnneal::genCombns(std::vector<std::pair<int, int> > &combns)
{   
    //Generate C_n^2 comb
    combns.clear();
    for(int i = 0; i < swarm_size; i++){
        for(int j = i + 1; j < swarm_size; j++)
            combns.push_back( std::make_pair(i,j) );
    }
}

std::vector<Eigen::Vector3d> SimAnneal::transformAssign( std::vector<Eigen::Vector3d> &swarm, const Eigen::VectorXi &assignment )
{
    std::vector<Eigen::Vector3d> swarm_new;
    for( int i = 0; i < swarm_size; i++ )
        swarm_new.push_back( swarm[assignment(i)] );
    return swarm_new;
}


void SimAnneal::initAssignment()
{   
    best_as = Eigen::VectorXi::Zero(swarm_size);
    for( int i = 0; i < swarm_size; i++ )
        best_as(i) = i; 
    last_as = best_as;
    have_assignment = true;
}


void SimAnneal::test( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr ){
    int iter_num = 0;
    srand(time(NULL));

    //Check if first time called
    if(!have_assignment){
        swarm_size = swarm_init.size();
        initAssignment();
    }
    
    //Make deep copies of the desired swarm nodes with initial permutation
    std::vector<Eigen::Vector3d> swarm_des_init = SG_ptr->getDesNodesInit();
    std::vector<Eigen::Vector3d> swarm_des = transformAssign(swarm_des_init, last_as);

    //Update the desired & initially-ordered laplacian
    calcLap(swarm_des, L_des);
    calcLap(swarm_init, L_init);

    as = last_as;

    std::vector<std::pair<int, int> > swap_combns;
    genCombns( swap_combns );
    double cost_total, cost_as, cost_sim;

    for( auto iter: swap_combns ){

        // //Swap, calculate metrics
        swapVariables( iter.first, iter.second );
        getCost(cost_sim, cost_as, cost_total);
        swapVariables( iter.second, iter.first );
    }
}

//Optimally Assign the nodes of desired formation. Manipulated variable: L_des & as

// void SimAnneal::runSA( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr )
void SimAnneal::runSA( const std::vector<Eigen::Vector3d> &swarm_init, SwarmGraph::Ptr &SG_ptr, bool &as_flag, std::vector<int> &assignment_vec )
{   
    //Check if assignment required
    double similarity_error;
    SG_ptr->calcFNorm2( similarity_error );
    if( similarity_error >= 0.18 ){

        as_flag = true;

        int iter_num = 0;
        srand(time(NULL));

        //Check if first time called
        if(!have_assignment){
            swarm_size = swarm_init.size();
            initAssignment();
        }
        
        //Make deep copies of the desired swarm nodes with initial permutation
        std::vector<Eigen::Vector3d> swarm_des_init = SG_ptr->getDesNodesInit();
        std::vector<Eigen::Vector3d> swarm_des = transformAssign(swarm_des_init, last_as);

        //Update the desired & initially-ordered laplacian
        calcLap(swarm_des, L_des);
        calcLap(swarm_init, L_init);
        
        //Init the cost variables
        double cost_sim_init, cost_as_init, cost_total_init;
        getCost(cost_sim_init, cost_as_init, cost_total_init);

        double cost_total = cost_total_init, cost_sim = cost_sim_init, cost_as = cost_as_init;
        double best_cost_total = cost_total, best_cost_sim = cost_sim, best_cost_as = cost_as;
        double delta_cost = 0;

        //Init the assignment
        Eigen::VectorXi init_as = last_as;
        as = init_as;

        //Flag if the solution is already acceptable
        bool is_low_cost = false;

        double rander;

        //Init the temperature
        double T = T_init;

        //Early check: swap the neighbor ID
        std::vector<std::pair<int, int> > swap_combns;
        genCombns( swap_combns );
        for( auto iter: swap_combns ){

            //Swap, calculate metrics
            swapVariables( iter.first, iter.second );
            getCost(cost_sim, cost_as, cost_total);
            
            if( cost_sim <= cost_sim_init * 0.25 ){
                
                std::cout << "Early Return. " << std::endl;
                best_as = as;
                last_as = best_as;
                
                //Reset the assignment_vec
                Eigen::VectorXi::Map( &assignment_vec[0], last_as.size() ) = last_as;

                //Generate new permutation of desired nodes using optimal assignment
                swarm_des = transformAssign( swarm_des_init, best_as );
                //Update the desired nodes in SwarmGraph
                SG_ptr->setDesiredForm( swarm_des );
                return;

            }else{
                //Swap the variables back
                swapVariables( iter.second, iter.first );
            }
        }
        

        //Simulated Anneal
        while( T > T_end )
        {
            for( int i = 0; i < markov_length; i++ )
            {
                genNewSolution();
                getCost(cost_sim, cost_as, cost_total);
                
                iter_num++;

                if( cost_sim <= cost_sim_init*0.25 && cost_as <= 0.4*swarm_size*as_error ){
                    // Save the assignment & cost
                    best_cost_total = cost_total;
                    best_cost_sim = cost_sim;
                    best_cost_as = cost_as;

                    best_as = as;
                    is_low_cost = true;
                    break;
                }

                delta_cost = cost_total - best_cost_total;

                if( delta_cost <= 0 ){
                    //Accept the solution
                    best_cost_total = cost_total;
                    best_cost_sim = cost_sim;
                    best_cost_as = cost_as;

                    best_as = as;
                }else{
                    // // Accept the solution with a temperature-based possibility
                    // rander = rand()%1000/(double)1000;
                    // if( rander < exp( -delta_cost / T ) ){
                    //     best_cost_total = cost_total;
                    //     best_cost_sim = cost_sim;
                    //     best_cost_as = cost_as;
                    //     best_as = as;
                    // }
                }
            }

            if(is_low_cost){
                break;
            }

            //Anneal the exploration
            T *= decay_scale;
        }

        if( best_cost_total <= cost_total_init ){
            
            //Save the best assignment
            last_as = best_as;
            
            //Reset the assignment_vec
            Eigen::VectorXi::Map( &assignment_vec[0], last_as.size() ) = last_as;

            //Generate new permutation of desired nodes using optimal assignment
            swarm_des = transformAssign( swarm_des_init, last_as );
            //Update the desired nodes in SwarmGraph
            SG_ptr->setDesiredForm( swarm_des );

            std::cout << "Init Assignment: " << init_as.transpose() << std::endl;
            std::cout << "Best Assignment: " << best_as.transpose() << std::endl;
            std::cout << "Init cost similarity: " << cost_sim_init << std::endl;
            std::cout << "Best cost similarity: " << best_cost_sim << std::endl;
            std::cout << "Init cost as_error: " << cost_as_init << std::endl;
            std::cout << "Best cost as_error: " << best_cost_as << std::endl;
            std::cout << "Iter_num: " << iter_num << std::endl;
        }else{
            std::cout << "Bad result." << std::endl;
            best_as = init_as;
        }

    }else{
        as_flag = false;
        Eigen::VectorXi::Map( &assignment_vec[0], last_as.size() ) = last_as;
        std::cout << "NO need for reAssignment. " << std::endl;
    }

}