#include "osqp_solver.hpp"
void qp_solver::qp_init(int numTasks, int DOFsize,std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi)
{
    Nt = numTasks;
    Dof = DOFsize;
    
    hessian.resize(Dof*Nt,Dof*Nt);
    gradient = Eigen::VectorXd::Zero(Dof*Nt);

    qp_setWeightMatrices(Qr,Qi);
    qp_castDGHC2QPHessian(jacobian,Qr,Qi,hessian);
    qp_castDGHC2QPGradient(jacobian,x_dot_d,gradient);
    // qp_setInEqualityConstrain();
    // qp_setEqualityConstraint();


    
}
void qp_solver::qp_setWeightMatrices(std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi)
{ 
    this-> Qr = Qr;
    this-> Qi = Qi;
    
}
void qp_solver::qp_castDGHC2QPHessian(std::vector<Eigen::MatrixXd>& alljacobian, std::vector<Eigen::MatrixXd>& Qr,std::vector<Eigen::MatrixXd>& Qi,Eigen::SparseMatrix<double>& hessianMatrix)
{
    Eigen::MatrixXd hessian;
    hessian.resize(Dof*Nt,Dof*Nt);
    for(int i = 0;i<Nt;i++)
    {
        hessian.block(i*Dof,i*Dof,Dof,Dof) = (alljacobian[i].transpose()*Qi[i]*alljacobian[i]) + Qr[i];  
    }

   
    hessianMatrix = hessian.sparseView();
    hessianMatrix.makeCompressed();
    std::cout<<"HessianMatrix: "<<std::endl<<hessianMatrix<<std::endl;

}
void qp_solver::qp_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian)
{
    this->jacobian = alljacobian;
}
void qp_solver::qp_castDGHC2QPGradient(std::vector<Eigen::MatrixXd>& alljacobian,std::vector<Eigen::VectorXd>& x_dot_d,Eigen::VectorXd& gradient)
{
    for(int i =0 ;i<Nt;i++)
    {
        gradient.block(i*Dof,0,Dof,1) = -2*alljacobian[i]*x_dot_d[i];
    }
}
