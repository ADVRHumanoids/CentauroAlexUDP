#include <CentauroAlexUDP/CentauroArmForceEstimation.h>



centauro::ForceEstimation::ForceEstimation(const XBot::ModelInterface& model):
    _model(model),
    _logger(XBot::MatLogger::getLogger("/tmp/centauro_arm_force_estimation_log")),
    _force(Eigen::Vector3d::Zero()),
    _res(Eigen::VectorXd::Zero(N_J)),
    _tau_offset(Eigen::VectorXd::Zero(N_J)),
    _J(6, model.getJointNum()),
    _ee_name("arm2_8")
{
    compute();
}


void centauro::ForceEstimation::compute()
{

    _model.getJointEffort(_tau);
    _model.computeNonlinearTerm(_nl);
    
    Eigen::Matrix3d w_R_ee;
    _model.getOrientation(_ee_name, w_R_ee);
    _model.getJacobian(_ee_name, _J);
    int start_dof = _model.getDofIndex(_model.chain("right_arm").getJointName(0));
    auto Jt = _J.topRows<3>().middleCols<N_J>(start_dof).transpose();
    
    _svd.compute(Jt, Eigen::ComputeFullU|Eigen::ComputeFullV);
    
    auto tau_residual = (_nl - _tau).segment<N_J>(start_dof);
    auto tau_residual_offset_comp = tau_residual - _tau_offset;
    Eigen::Vector3d f_est = _svd.solve(tau_residual_offset_comp);
    
    _logger->add("f_est", f_est);
    _logger->add("tau_residual", tau_residual);
    
    _force = f_est;
    _res = tau_residual;
}


void centauro::ForceEstimation::set_offset(const Eigen::VectorXd& tau_offset)
{
    _tau_offset = tau_offset;
}

Eigen::Vector3d centauro::ForceEstimation::getForce(Eigen::VectorXd* tau_res) const
{
    if(tau_res) *tau_res = _res;
    return _force;
}

centauro::ForceEstimation::~ForceEstimation()
{
    _logger->flush();
}



