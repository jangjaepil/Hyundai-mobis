#include "osqp_solver.hpp"
void qp_solver::qp_init(int numTasks, int DOFsize,Eigen::VectorXd tasksize,Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi)
{
    Nt = numTasks;
    Dof = DOFsize;
    Ts = tasksize; 
    
    qp_setWeightMatrices(Qr,Qi);
    qp_castDGHC2QPHessian(Nt,Dof,Ts,Qr,Qi,hessian);
    qp_castDGHC2QPGradient(gradient);
    qp_setLinearConstraint(Projections,jacobian,Nt,Dof,linearMatrix);
    qp_setConstraintVectors();


    
}
void qp_solver::qp_setWeightMatrices(Eigen::MatrixXd& allQr,Eigen::MatrixXd& allQi)
{ 
    this-> Qr = allQr;
    this-> Qi = allQi;
    
}
void qp_solver::qp_castDGHC2QPHessian(int& Nt, int& Dof,Eigen::VectorXd& Ts,Eigen::MatrixXd& Qr,Eigen::MatrixXd& Qi,Eigen::SparseMatrix<double>& hessianMatrix)
{
    hessianMatrix.resize(Nt*(Ts.sum()+Dof),Nt*(Ts.sum()+Dof));
    hessianMatrix.setZero();
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
void qp_solver::qp_setProjectionMatrices(std::vector<Eigen::MatrixXd>& allProjections)
{
    this->Projections = allProjections;
}
void qp_solver::qp_castDGHC2QPGradient(Eigen::VectorXd& gradient)
{
    gradient = Eigen::VectorXd::Zero(Dof*Nt+Ts.sum());    
}
void qp_solver::qp_setLinearConstraint(std::vector<Eigen::MatrixXd>& allProjections,std::vector<Eigen::MatrixXd>& alljacobian,int& Nt, int& Dof,Eigen::SparseMatrix<double>& constraintMatrix)
{
    constraintMatrix.resize(Ts.sum()+Dof,Ts.sum()+Nt*Dof);
    constraintMatrix.setZero();

    Eigen::MatrixXd constraintmatrix = Eigen::MatrixXd::Zero(Ts.sum()+Dof,Ts.sum()+Nt*Dof);
    constraintmatrix.block(0,0,Ts.sum(),Ts.sum()) = Eigen::MatrixXd::Identity(Ts.sum(),Ts.sum());
    for(int i = 0;i<Nt;i++)
    {   
        if(i==0)
        {
            constraintmatrix.block(i,Ts.sum()+i*Dof,Ts(i),Dof) = alljacobian[i];    
        }
        else
        {
            constraintmatrix.block(Ts(i-1)*i,Ts.sum()+i*Dof,Ts(i),Dof) = alljacobian[i];
        }
        constraintmatrix.block(Ts.sum(),Ts.sum()+i*Dof,Dof,Dof) = allProjections[i];
    }

    constraintMatrix = constraintmatrix.sparseView();


}
void qp_solver::qp_setConstraintVectors()
{

}