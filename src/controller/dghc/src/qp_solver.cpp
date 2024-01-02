#include "osqp_solver.hpp"
void qp_solver::qp_init(int numTasks, int DOFsize,Eigen::VectorXd tasksize,Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi)
{
    Nt = numTasks;
    Dof = DOFsize;
    Ts = tasksize; 
    
    qp_setWeightMatrices(Qr,Qi);
    qp_castDGHC2QPHessian(Nt,Dof,Ts,Qr,Qi,hessian);
    qp_castDGHC2QPGradient(gradient);
    // qp_setInEqualityConstrain();
    // qp_setEqualityConstraint();


    
}
void qp_solver::qp_setWeightMatrices(Eigen::MatrixXd& allQr,Eigen::MatrixXd& allQi)
{ 
    this-> Qr = allQr;
    this-> Qi = allQi;
    
}
void qp_solver::qp_castDGHC2QPHessian(int& Nt, int& Dof,Eigen::VectorXd& Ts,Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi,Eigen::SparseMatrix<double>& hessianMatrix)
{
    hessianMatrix.resize(Nt*(Ts.sum()+Dof),Nt*(Ts.sum()+Dof));
    for(int i = 0;i<Nt*(Ts.sum()+Dof);i++)
    {   
        if(i<(Nt*Ts.sum()))
        { 

            hessianMatrix.insert(i,i) = Qi(i,i);
        }
        else if(i<(Nt*(Ts.sum()+Dof)))
        {
            hessianMatrix.insert(i,i) = Qr(i,i);
        }
    }
}
void qp_solver::qp_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian)
{
    this->jacobian = alljacobian;
}
void qp_solver::qp_castDGHC2QPGradient(Eigen::VectorXd& gradient)
{
    gradient = Eigen::VectorXd::Zero(Dof*Nt+Ts.sum());    
}
