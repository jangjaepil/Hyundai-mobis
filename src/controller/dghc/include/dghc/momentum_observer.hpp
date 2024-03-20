#ifndef MOMENTUM_OBSERVER_H
#define MOMENTUM_OBSERVER_H

#include <mobis_admittance/admittance_control.hpp>

class MomentumObserver {
public:
  MomentumObserver(Eigen::VectorXd& k);
  Eigen::VectorXd getExternalTorque(Eigen::MatrixXd& inertia, Eigen::MatrixXd& coriolis,  Eigen::VectorXd& gravity_comp, 
                                                    Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::VectorXd& tau, double dt);
  void settings(Eigen::VectorXd& k);

private:
  Eigen::VectorXd sum, r;
  Eigen::VectorXd p, beta, torque, tprev;
  Eigen::VectorXd ko, ko_rat;
  bool isRun;
};

MomentumObserver::MomentumObserver(Eigen::VectorXd& k)
  : sum(Eigen::VectorXd::Zero(6))
  , r(Eigen::VectorXd::Zero(6))
  , p(Eigen::VectorXd::Zero(6))
  , beta(Eigen::VectorXd::Zero(6))
  , torque(Eigen::VectorXd::Zero(6))
  , tprev(Eigen::VectorXd::Zero(6))
  , ko(k)
  , ko_rat(Eigen::VectorXd::Zero(6))
  , isRun(false)
{ 
  settings(k);
}

Eigen::VectorXd MomentumObserver::getExternalTorque(Eigen::MatrixXd& inertia, Eigen::MatrixXd& coriolis, Eigen::VectorXd& gravity_comp, 
                                                    Eigen::VectorXd& q, Eigen::VectorXd& qd, Eigen::VectorXd& tau, double dt)
{
  p = inertia * qd;
  beta = gravity_comp - coriolis.transpose() * qd;
  torque = tau;
 
  if(isRun) {
    torque += r - beta;
    sum += 0.5 * dt * (torque + tprev);
    // sum += dt * (torque + tprev);
  } else {
    torque -= beta;
    r.setZero();
    sum = p;
    isRun = true;
  }
  tprev = torque;

  p -= sum;

  for(int i = 0; i < 6; i++) {
    r(i) = ko(i) * (p(i));
    torque(i) = ko_rat(i) * r(i);
  }
  
  return torque;
}

void MomentumObserver::settings(Eigen::VectorXd& k)
{
  ko = k;
  for(int i = 0; i < 6; i++) ko_rat(i) = k(i)/(1+k(i));
}

#endif // MOMENTUM_OBSERVER_H
