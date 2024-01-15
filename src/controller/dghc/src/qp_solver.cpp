#include "osqp_solver.hpp"
bool qp_solver::qp_init(Eigen::VectorXd& init_q,std::vector<Eigen::VectorXd>&init_x_dot_d,
                        std::vector<Eigen::MatrixXd>&init_jacobians,std::vector<Eigen::MatrixXd>&init_projections,
                        unsigned int numTasks, unsigned int DOFsize,Eigen::VectorXd& tasksize)
{
    //////////////////// set variables //////////////////////////////////////////////////
    Nt = numTasks;
    Dof = DOFsize;
    Ts = tasksize; 
    lbq = Eigen::VectorXd::Zero(Dof);
    ubq = Eigen::VectorXd::Zero(Dof);
    lbq << -30,-30,-30,-30,-30,-30,-30,-30,0,0,0,0,-10,-10,-10,-10,-10,-10;  //
    ubq << 30,30,30,30,30,30,30,30,0,0,0,0,10,10,10,10,10,10;
    ctr = Eigen::VectorXd::Zero(Dof);
    qp_setcurrent_q(init_q);
    qp_setRef(init_x_dot_d);
    qp_setJacobianMatrices(init_jacobians);
    qp_setProjectionMatrices(init_projections);
    qp_setWeightMatrices(Qr,Qi);
    std::cout<<"set variables"<<std::endl;
  
    //////////////////// cast qp problem ///////////////////////////////////////////////
    qp_castDGHC2QPHessian(Nt,Dof,Ts,Qr,Qi,hessian);
    std::cout<<"set hessian"<< std::endl;
    qp_castDGHC2QPGradient(Nt,Dof,Ts,gradient);
    std::cout<<"set gradient"<<std::endl;
    qp_setLinearConstraint(Nt,Dof,Ts,init_projections,init_jacobians,linearMatrix);
    std::cout<<"set constraint matrix"<<std::endl;
    qp_setConstraintVectors(dt,Nt,Dof,Ts,init_x_dot_d,init_q,lbq,ubq,lowerBound,upperBound);
    std::cout<<"set constraint vector"<<std::endl;
    std::cout<<"cast to qp problem done"<<std::endl;
    //////////////////// initialize qp problem ///////////////////////////////////////////
    solver.settings()->setWarmStart(true);
    
    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(Ts.sum()+Nt*Dof);
    solver.data()->setNumberOfConstraints(Ts.sum()+Dof);
    if (!solver.data()->setHessianMatrix(hessian))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;
    std::cout<<"initialize qp problem"<<std::endl;
    // instantiate the solver
    if (!solver.initSolver())
        return 1;
    std::cout<<"instantiate the solver"<<std::endl;
    return 0;
}
void qp_solver::qp_setcurrent_q(Eigen::VectorXd& q)
{
    this->current_q = q;
}
void qp_solver::qp_setWeightMatrices(Eigen::MatrixXd& allQr,Eigen::MatrixXd& allQi)
{ 
    this-> Qr = allQr;
    this-> Qi = allQi;
    
}
void qp_solver::qp_castDGHC2QPHessian(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,Eigen::MatrixXd& Qr,
                                    Eigen::MatrixXd& Qi,Eigen::SparseMatrix<double>& hessianMatrix)
{
    hessianMatrix.resize(Ts.sum()+Nt*Dof,Ts.sum()+Nt*Dof);
    hessianMatrix.setZero();
    
    for(int i = 0;i<Ts.sum()+ Nt*Dof;i++)
    {   
        if(i<Ts.sum())
        { 

            hessianMatrix.insert(i,i) = Qi(i,i);
        }
        else if(i<Ts.sum()+Nt*Dof)
        {
            hessianMatrix.insert(i,i) = Qr(i-int(Ts.sum()),i-int(Ts.sum()));
        }
    }
}
void qp_solver::qp_setJacobianMatrices(std::vector<Eigen::MatrixXd>& alljacobian)
{
    this->jacobians = alljacobian;
}
void qp_solver::qp_setProjectionMatrices(std::vector<Eigen::MatrixXd>& allProjections)
{
    this->Projections = allProjections;
}
void qp_solver::qp_castDGHC2QPGradient(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,Eigen::VectorXd& gradient)
{
    gradient = Eigen::VectorXd::Zero(Dof*Nt+Ts.sum());    
}
void qp_solver::qp_setLinearConstraint(unsigned  int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,
                                        std::vector<Eigen::MatrixXd>& allProjections,std::vector<Eigen::MatrixXd>& alljacobian,
                                        Eigen::SparseMatrix<double>& constraintMatrix)
{
  

    Eigen::MatrixXd constraintmatrix = Eigen::MatrixXd::Zero(Ts.sum()+Dof,Ts.sum()+Nt*Dof);
    constraintmatrix.block(0,0,Ts.sum(),Ts.sum()) = Eigen::MatrixXd::Identity(Ts.sum(),Ts.sum());
    
    int offset = 0;
    
    for(unsigned int i = 0;i<Nt;i++)
    {   
        
            constraintmatrix.block(offset,Ts.sum()+i*Dof,Ts(i),Dof) = alljacobian[i];    
            offset = offset + Ts(i);
                
            constraintmatrix.block(Ts.sum(),Ts.sum()+i*Dof,Dof,Dof) = allProjections[i];
            
    }

    constraintMatrix = constraintmatrix.sparseView();
}
void qp_solver::qp_setRef(std::vector<Eigen::VectorXd>& allx_dot_d)
{
    this->x_dot_d = allx_dot_d;
}
void qp_solver::qp_setConstraintVectors(double& dt, unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,
                                        std::vector<Eigen::VectorXd>& allx_dot_d,Eigen::VectorXd& current_q,
                                        Eigen::VectorXd& lbq,Eigen::VectorXd& ubq,Eigen::VectorXd& lowerBound, 
                                        Eigen::VectorXd& upperBound)
{
    lowerBound = Eigen::VectorXd::Zero(Ts.sum()+Dof);
    upperBound = Eigen::VectorXd::Zero(Ts.sum()+Dof);
    int offset = 0;
    for(unsigned int i = 0;i<Nt;i++)
    {   
        lowerBound.block(offset,0,Ts(i),1) = allx_dot_d[i];
        upperBound.block(offset,0,Ts(i),1) = allx_dot_d[i];
        offset = offset + Ts(i);
    }
        
        lowerBound.block(offset,0,Dof,1) = lbq;
        upperBound.block(offset,0,Dof,1) = ubq;        
}

bool qp_solver::qp_solve_problem(std::vector<Eigen::MatrixXd>&allProjections)
{
    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
       return 0;
    // get the controller input
    QPSolution = solver.getSolution();
    
    ctr.setZero();
    for(unsigned int i = 0;i<Nt;i++)
    {
        ctr = ctr + allProjections[i]*QPSolution.block(Ts.sum()+Dof*i,0,Dof,1);
    }

    return 1;
}

bool qp_solver::qp_updateAllConstraint(std::vector<Eigen::MatrixXd>&update_Projections,
                                        std::vector<Eigen::MatrixXd>&update_jacobian,std::vector<Eigen::VectorXd>& update_x_dot_d,Eigen::VectorXd& update_q)
{
    qp_setcurrent_q(update_q);
    qp_setJacobianMatrices(update_jacobian);
    qp_setProjectionMatrices(update_Projections);
    qp_setRef(update_x_dot_d);
    qp_setLinearConstraint(Nt,Dof,Ts,Projections,jacobians,linearMatrix);
    qp_setConstraintVectors(dt,Nt,Dof,Ts,x_dot_d,current_q,lbq,ubq,lowerBound,upperBound);
    
    if (!solver.updateBounds(lowerBound, upperBound))
            return 1;

    if(!solver.updateLinearConstraintsMatrix(linearMatrix))
            return 1;

    return 0;

}

Eigen::VectorXd qp_solver::getProjectedJointVel()
{
    return ctr;
}