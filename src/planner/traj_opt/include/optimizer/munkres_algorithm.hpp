#ifndef MUNKRES_ALGORITHM_HPP
#define MUNKRES_ALGORITHM_HPP

#include <algorithm>
#include <iostream>
#include <list>
#include <set>
#include <stack>
#include <stdexcept>
#include <vector>
#include <iterator>
#include <Eigen/Eigen>

class Munkres
{
    private:
        Eigen::MatrixXd C_;
        int n_row_;
        int n_col_;
        std::vector<bool> cover_row_;
        std::vector<bool> cover_col_;
        std::vector<std::vector<bool>> star_matrix_;
        std::vector<std::vector<bool>>  prime_matrix_; 
        
        double emin_;
        int row_z0_;
        int col_z0_;

        int step_;

        /* useful function*/
        bool check_validity(){
        // FUNCTION to check if given matrix is valid
            if(n_row_ != n_col_){
                std::cout << "Invalid Matrix." << std::endl;
                return false;
            }
            for (int row = 0; row < n_row_; row++) {
                for (int col = 0; col < n_col_; col++) {  
                    if(C_(row,col) < 0){
                        std::cout << "Invalid Matrix." << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }

        bool find_zero(int row, int col){
            if(C_(row,col) == 0){
                return true; 
            } else{
                return false;
            }
        }

        bool col_starred (int col){
            for (int row = 0; row < n_row_; row++) {
                if(star_matrix_[row][col])
                    return true;
            }
            return false;
        }

        bool row_starred (int row){
            for (int col = 0; col < n_col_; col++) {
                if(star_matrix_[row][col])
                    return true;
            }
            return false;
        }

        bool uncovered_zero (){
            for (int row = 0; row < n_row_; row++) {
                for (int col = 0; col < n_col_; col++) {  
                    if ((!cover_col_[col] && !cover_row_[row]) && find_zero(row, col))
                        return true;
                }
            }
            return false;
        }

        double smallest_element(){
            double min = DBL_MAX;
            for (int row = 0; row < n_row_; row++) {
                if (!cover_row_[row]) {
                    for (int col = 0; col < n_col_; col++) {
                        if (!cover_col_[col] && C_(row, col) < min)
                            min = C_(row, col);
                    }
                }
            }
            return min;
        }

        std::pair<int, int> get_star_zero (int col){
        /*FUNCTION to find a starred zero in a column*/
        for(int row =0; row < n_row_; row++) {
            if (star_matrix_[row][col] == 1) return std::make_pair(row,col);
        }
        return std::pair<int, int>();
        }

        int STEP_0(){
            /* For each row r in C, subtract its smallest element from every element in r
               For each column c in C, subtract its smallest element from every element in c*/
            for (int row = 0; row < n_row_; row++) {
                double row_min = DBL_MAX;
                for (int col = 0; col < n_col_; col++) {
                    if (C_(row, col) < row_min)
                        row_min = C_(row, col);     
                }
                for (int col = 0; col < n_col_; col++) {  
                    C_(row, col) -= row_min;
                }
            }

            /* For all zeros z_i in C, mark z_i with a star if there is no starred zero in its row or column*/
            for (int row = 0; row < n_row_; row++) {
                for (int col = 0; col < n_col_; col++) {
                    if (!row_starred(row)  && !col_starred(col) && find_zero(row,col))
                        star_matrix_[row][col] = true;
                }
            }
            return 1;
        }

        int STEP_1(){
            /* for Each column containing a starred zero, cover this column
               if n columns are covered then GOTO DONE*/
            for (int row = 0; row < n_row_; row++) { 
                for (int col = 0; col < n_col_; col++) {
                    if (col_starred(col))
                        cover_col_[col] = true;
                }
            }
            const bool covered = std::all_of(cover_col_.begin(), cover_col_.end(), [](bool v) {return v; });
            
            if(covered)
                return 5;
            
            return 2;
        }

        int STEP_2(){
            /* find a noncovered zero and prime it.  
            If there is no starred zero in the row containing this primed zero, Go to Step 3.  
            Otherwise, cover this row and uncover the column containing the starred zero. 
            Continue in this manner until there are no uncovered zeros left, Go to Step 2 
            Save the smallest uncovered value and Go to Step 4.*/

            if(uncovered_zero()) {
                for (int row = 0; row < n_row_; row++) {  
                    for (int col = 0; col < n_col_; col++) {
                        if ((!cover_col_[col] && !cover_row_[row]) && find_zero(row, col)) { //find uncovered zero
                            prime_matrix_[row][col] = true; //prime it
                            if (!row_starred(row)) { //if no starred zero in row
                                row_z0_ = row;
                                col_z0_ = col;

                                return 3;
                            } else { //if starred zero
                                cover_row_[row] = true; //cover row
                                for (int column = 0; column < n_col_; column++) {
                                    if (star_matrix_[row][column]) { //uncover col containing star zero
                                        cover_col_[column] = false;
                                        
                                        return 2;
                                    }
                                }
                            }
                        }
                    }
                }
            } else { //save the smallest uncovered element, e_min
                emin_ = smallest_element();
            }
        
            return 4;
        }

        int STEP_3(){
            /* Construct a series S of alternating primed and starred zeros as follows:
               Insert Z 0 into S
               while In the column of Z 0 exists a starred zero Z 1 do
               Insert Z 1 into S
               Replace Z 0 with the primed zero in the row of Z 1 . Insert Z 0 into S
               end while */
            
            std::pair<int ,int > Z0; //primed zero
            std::pair<int ,int > Z1; // starred zero
            Z0 = std::make_pair(row_z0_, col_z0_);
            std::vector<std::pair<int, int>> S;
            S.emplace_back(Z0);

            while(col_starred(Z0.second)){ //if column of Z0 has a starred zero Z1
                Z1 = get_star_zero(Z0.second);
                S.emplace_back(Z1); // insert Z1 to S

                for(int col =0; col < n_col_;  col++){
                    if(prime_matrix_[Z1.first][col]){ //Replace Z0 with the primed zero in the row of Z1 
                        Z0 = std::make_pair(Z1.first,col);
                        S.emplace_back(Z0); //Insert Z0 into S
                        break;
                    }
                }
            }

            /* Unstar each starred zero in S and replace all primes with stars.
               Erase all other primes and uncover every line in C GOTO STEP 1 */
            for(int index =0; index < S.size(); index=index+2){
                int z0_row = S[index].first;
                int z0_col = S[index].second;
                star_matrix_[z0_row][z0_col] = true; //replace all primes with stars 
            
                if(index != S.size() -1) {
                    int z1_row = S[index + 1].first;
                    int z1_col = S[index + 1].second;
                    star_matrix_[z1_row][z1_col] = false; //Unstar each starred zero in S
                }
            }

            for(int i =0; i < n_row_; i++)
                for(int j =0; j < n_row_; j++){
                    prime_matrix_[i][j] = false; //Erase all other primes
                }
            
            //uncover every line in C
            for(auto row:cover_row_){
                row = false;
            }

            for(auto col:cover_col_){
                col= false;
            }
            return 1;
        }

        int STEP_4(){
            /* Add emin to every element in covered rows
               subtract it from every element in uncovered columns. GOTO STEP 2*/

            for (int row = 0; row < n_row_; row++) {
                if (cover_row_[row]) {
                    for (int col = 0; col < n_col_; col++) {
                        C_(row, col) = C_(row,col) + emin_;
                    }
                }
            }

            for (int col = 0; col < n_col_; col++) {
                if (!cover_col_[col]) {
                    for (int row = 0; row < n_row_; row++) {
                        C_(row, col) = C_(row,col) - emin_;
                    }
                }
            }

            return 2;
        }

        void dubug(){
            std::cout << "-------debug ------" << std::endl;
            std::cout << "step_ : " << step_ << std::endl;
            std::cout << "C_ : " << std::endl;
            for (int row=0; row<n_row_ ; row++){
                for (int col=0; col<n_col_; col++){
                    std::cout << C_(row, col) << " " ;
                }
                std::cout << std::endl;
            }

            std::cout << "star_matrix_ : " << std::endl;
            for (int row=0; row<n_row_ ; row++){
                for (int col=0; col<n_col_; col++){
                    std::cout << star_matrix_[row][col] << " " ;
                }
                std::cout << std::endl;
            }

            std::cout << "emin_ : " << emin_ << std::endl;

        }

    public:
        Munkres(){};
        ~Munkres(){};

        // API
        void run_munkres_algorithm(Eigen::MatrixXd C){
            // initial
            C_              = C;
            n_row_          = C.rows();
            n_col_          = C.cols();
            cover_row_      = std::vector<bool>(n_row_, false);
            cover_col_      = std::vector<bool>(n_col_, false);
            star_matrix_    = std::vector<std::vector<bool>>(n_row_, std::vector<bool>(n_col_, false));
            prime_matrix_   = std::vector<std::vector<bool>>(n_row_, std::vector<bool>(n_col_, false));
            emin_           = DBL_MAX;
            step_           = 0;

            if(!check_validity())
                return;
            
            bool show_debug_output = false;      // change manually
            
            if (show_debug_output){
                std::cout << "[run_munkres_algorithm] --------------------------------------------" << std::endl;
                dubug();
            }
            
            step_ = STEP_0();
            
            bool DONE = false;
            while (!DONE) {

                if (show_debug_output)
                    dubug();
                
                switch (step_) {
                    case 1:
                        step_ = STEP_1();
                        break;
                    case 2:
                        step_ = STEP_2();
                        break;
                    case 3:
                        step_ = STEP_3();
                        break;
                    case 4:
                        step_ = STEP_4();
                        break;
                    default:
                        DONE = true;
                        break;
                }
            }
        }

        Eigen::VectorXi output(){
            Eigen::VectorXi assignment(n_row_);
            for (int row=0; row<n_row_ ;row++)
                for (int col=0; col<n_col_; col++){
                    if (star_matrix_[row][col])
                        assignment(row) = col;
                }
            return assignment;
        }
};

#endif //MUNKRES_ALGORITHM_HPP