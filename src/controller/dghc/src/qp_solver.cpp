#include "osqp_solver.hpp"
bool qp_solver::qp_init(Eigen::VectorXd& init_q,std::vector<Eigen::VectorXd>&init_x_dot_d,
                        std::vector<Eigen::MatrixXd>&init_jacobians,std::vector<Eigen::MatrixXd>&init_projections,
                        unsigned int numTasks, unsigned int DOFsize,Eigen::VectorXd& tasksize)
{
    //////////////////// set variables //////////////////////////////////////////////////
    Nt = numTasks;
    Dof = DOFsize;
    Ts = tasksize; 
    lbq_dot = Eigen::VectorXd::Zero(Dof);
    ubq_dot = Eigen::VectorXd::Zero(Dof);
    q_upper_limit = Eigen::VectorXd::Zero(Dof-8);
    q_lower_limit = Eigen::VectorXd::Zero(Dof-8);
    lbq_dot << -30,-30,-30,-30,-30,-30,-30,-30,-0.5,-0.5,-0.5,-0.5,-M_PI,-M_PI,-M_PI,-M_PI,-M_PI,-M_PI;  // velocity limit
    ubq_dot << 30,30,30,30,30,30,30,30,0.5,0.5,0.5,0.5,M_PI,M_PI,M_PI,M_PI,M_PI,M_PI;
    q_upper_limit<<0,0,0,0,2*M_PI,2*M_PI,2*M_PI,2*M_PI,2*M_PI,2*M_PI; // joint limit
    q_lower_limit<<-0.15,-0.15,-0.15,-0.15,-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI,-2*M_PI;
    ctr = Eigen::VectorXd::Zero(Dof);
    
    qp_setcurrent_q(init_q);
    std::cout<<"set init q"<<std::endl;
    qp_setRef(init_x_dot_d);
    std::cout<<"set init ref"<<std::endl;
    qp_setJacobianMatrices(init_jacobians);
    std::cout<<"set jacobians"<<std::endl;
    qp_setProjectionMatrices(init_projections);
    std::cout<<"set projections"<<std::endl;
    qp_setWeightMatrices(Qr,Qi);
    std::cout<<"set qr and qi"<<std::endl;
    std::cout<<"set variables"<<std::endl;
  
    //////////////////// cast qp problem ///////////////////////////////////////////////
    qp_castDGHC2QPHessian(Nt,Dof,Ts,Qr,Qi,hessian);
    std::cout<<"set hessian"<< std::endl;
    //std::cout<<hessian<<std::endl;
    qp_castDGHC2QPGradient(Nt,Dof,Ts,gradient);
    std::cout<<"set gradient"<<std::endl;
    //std::cout<<gradient<<std::endl;
    qp_setLinearConstraint(Nt,Dof,Ts,init_projections,init_jacobians,linearMatrix);
    std::cout<<"set constraint matrix"<<std::endl;
    std::cout<<linearMatrix<<std::endl;
    qp_setConstraintVectors(Nt,Dof,Ts,init_x_dot_d,init_q,lbq_dot,ubq_dot,q_lower_limit,q_upper_limit,lowerBound,upperBound);
    std::cout<<"set constraint vector"<<std::endl;
    std::cout<<upperBound.transpose()<<std::endl;
    std::cout<<lowerBound.transpose()<<std::endl;
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
            // if(Ts[i] ==1)
            // {
            //     for(unsigned int j =0;j<Nt;j++)
            //     {
            //         constraintmatrix.block(Ts.sum()+Dof,Ts.sum()+j*Dof,1,Dof) = alljacobian[i]*allProjections[j];
            //     }
            // } 
    }

    constraintMatrix = constraintmatrix.sparseView();
}
void qp_solver::qp_setRef(std::vector<Eigen::VectorXd>& allx_dot_d)
{
    this->x_dot_d = allx_dot_d;
}
void qp_solver::qp_setConstraintVectors(unsigned int& Nt, unsigned int& Dof,Eigen::VectorXd& Ts,
                                        std::vector<Eigen::VectorXd>& allx_dot_d,Eigen::VectorXd& current_q,
                                        Eigen::VectorXd& lbq_dot,Eigen::VectorXd& ubq_dot,Eigen::VectorXd& q_lower_limit,
                                        Eigen::VectorXd& q_upper_limit,Eigen::VectorXd& lowerBound, 
                                        Eigen::VectorXd& upperBound)
{

    lowerBound = Eigen::VectorXd::Zero(Ts.sum()+Dof);
    upperBound = Eigen::VectorXd::Zero(Ts.sum()+Dof);
    Eigen::VectorXd LB = Eigen::VectorXd::Zero(Dof);
    Eigen::VectorXd UB = Eigen::VectorXd::Zero(Dof);
    // std::cout<<"ref_lbq_dot: "<<std::endl<<lbq_dot<<std::endl;
    // std::cout<<"ref_ubq_dot: "<<std::endl<<ubq_dot<<std::endl;
    int offset = 0;
    double obs_vel = 0;
    for(unsigned int i = 0;i<Nt;i++)
    {   
        if(Ts(i) == 1)
        {   
            lowerBound(offset,0) =  allx_dot_d[i].norm();
            upperBound(offset,0) =  allx_dot_d[i].norm();  
            obs_vel = allx_dot_d[i].norm();
            offset = offset + Ts(i);
        }
        else
        {
            lowerBound.block(offset,0,Ts(i),1) = allx_dot_d[i];
            upperBound.block(offset,0,Ts(i),1) = allx_dot_d[i];
            offset = offset + Ts(i);
        }
    }
    
    for(unsigned int i = 0 ; i<Dof;i++)
    {
        if(i>=8 && i<Dof)
        {
            if(current_q(i)>=q_upper_limit(i-8))
            {
                // std::cout<<"current q : "<<i<<"th "<<current_q(i)<<std::endl;
                // std::cout<<"q_upper_limit : "<<i-8<<"th "<<q_upper_limit(i-8)<<std::endl;
                LB(i) = lbq_dot(i);
                UB(i) = 0;
            }
            else if(current_q(i)<= q_lower_limit(i-8))
            {   
                // std::cout<<"current q : "<<i<<"th "<<current_q(i)<<std::endl;
                // std::cout<<"q_lower_limit : "<<i-8<<"th "<<q_lower_limit(i-8)<<std::endl;
                UB(i) = ubq_dot(i);
                LB(i) = 0;
            }
            else
            {
                UB(i) = ubq_dot(i);
                LB(i) = lbq_dot(i);
                // std::cout<<i<<"th joint is safe"<<std::endl;
            }
        }
        else
        {
            UB(i) = ubq_dot(i);
            LB(i) = lbq_dot(i);
        }
    }

   
    // std::cout<<"lbq_dot: "<<std::endl<<LB.transpose()<<std::endl;
    // std::cout<<"ubq_dot: "<<std::endl<<UB.transpose()<<std::endl;
    // std::cout<<"current_q: "<<std::endl<<current_q.transpose()<<std::endl;
    
    lowerBound.block(offset,0,Dof,1) = LB;
    upperBound.block(offset,0,Dof,1) = UB;
    // if(obs_vel>=0.008)
    // {
    //     lowerBound(offset+Dof,0) = 0;
    //     upperBound(offset+Dof,0) = OsqpEigen::INFTY;        
    // }
    // else
    // {
    //     lowerBound(offset+Dof,0) = -OsqpEigen::INFTY;
    //     upperBound(offset+Dof,0) = OsqpEigen::INFTY;             
    // }
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
    qp_setConstraintVectors(Nt,Dof,Ts,x_dot_d,current_q,lbq_dot,ubq_dot,q_lower_limit,q_upper_limit,lowerBound,upperBound);
    
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