#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#pragma once

class qp_solver {
public:
    void qp_init(int numTasks, int DOFsize,std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi);
    void qp_setWeightMatrices(std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi);
    void qp_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian);
    void qp_castDGHC2QPHessian(std::vector<Eigen::MatrixXd>& alljacobian, std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi,Eigen::SparseMatrix<double>& hessianMatrix);
    void qp_castDGHC2QPGradient(std::vector<Eigen::MatrixXd>& alljacobian,std::vector<Eigen::VectorXd>& x_dot_d,Eigen::VectorXd& gradient);
    void qp_setInEqualityConstrain();
    void qp_setEqualityConstraint();
    void qp_solve_problem();
    
private:
    int Nt = 0;
    int Dof = 0;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    std::vector<Eigen::MatrixXd> Qr;
    std::vector<Eigen::MatrixXd> Qi;
    std::vector<Eigen::MatrixXd> jacobian;
    std::vector<Eigen::VectorXd> x_dot_d;
    
      
};