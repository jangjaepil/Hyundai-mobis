#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#pragma once

class qp_solver {
public:
    bool qp_init(Eigen::VectorXd& init_q,std::vector<Eigen::VectorXd>&init_x_dot_d,
                std::vector<Eigen::MatrixXd>&init_jacobians,std::vector<Eigen::MatrixXd>&init_projections,
                unsigned int numTasks, unsigned int DOFsize,Eigen::VectorXd& tasksize);
    void qp_setcurrent_q(Eigen::VectorXd& q);
    void qp_setWeightMatrices(Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi);
    void qp_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian);
    void qp_setProjectionMatrices(std::vector<Eigen::MatrixXd>& allProjections);
    void qp_castDGHC2QPHessian(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi,Eigen::SparseMatrix<double>& hessianMatrix);
    void qp_castDGHC2QPGradient(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,Eigen::VectorXd& gradient);
    void qp_setLinearConstraint(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,std::vector<Eigen::MatrixXd>& allProjections,std::vector<Eigen::MatrixXd>& alljacobian,Eigen::SparseMatrix<double>& constraintMatrix);
    void qp_setRef(std::vector<Eigen::VectorXd>& allx_dot_d);
    void qp_setConstraintVectors(double& dt, unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,std::vector<Eigen::VectorXd>& allx_dot_d,Eigen::VectorXd& current_q,Eigen::VectorXd& lbq,Eigen::VectorXd& ubq,Eigen::VectorXd& q_lower_limit,Eigen::VectorXd& q_upper_limit,Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound);
    bool qp_solve_problem(std::vector<Eigen::MatrixXd>&allProjections);
    bool qp_updateAllConstraint(std::vector<Eigen::MatrixXd>&allProjections,std::vector<Eigen::MatrixXd>&update_jacobian,std::vector<Eigen::VectorXd>& update_x_dot_d,Eigen::VectorXd& update_q);
    Eigen::VectorXd getProjectedJointVel();
private:
    unsigned int Nt = 0;
    unsigned int Dof = 0;
    double dt = 1;

    OsqpEigen::Solver solver;
    Eigen::VectorXd QPSolution;
    Eigen::VectorXd ctr;
    Eigen::VectorXd Ts;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lbq_dot;
    Eigen::VectorXd ubq_dot;
    Eigen::VectorXd current_q;
    Eigen::VectorXd q_lower_limit;
    Eigen::VectorXd q_upper_limit;
    Eigen::MatrixXd Qr;
    Eigen::MatrixXd Qi;
    std::vector<Eigen::MatrixXd> jacobians;
    std::vector<Eigen::MatrixXd> Projections;
    std::vector<Eigen::VectorXd> x_dot_d;
    
      
};