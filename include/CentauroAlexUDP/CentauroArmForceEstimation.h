#ifndef __MPL_CENTAURO_FORCE_EST_H__
#define __MPL_CENTAURO_FORCE_EST_H__

#include <XBotInterface/RobotInterface.h>
#include <vector>

namespace centauro {

    
    class ForceEstimation 
    {
        
    public:
        
        typedef std::shared_ptr<ForceEstimation> Ptr;
        
        static const int N_J = 4;
        
        /**
         * @brief Contruct the force estimation class.
         * 
         * @param model Model that is kept updated with the robot state
         */
        ForceEstimation(const XBot::ModelInterface& model);
        
        /**
         * @brief Call this function to compute an estimate based on the current model state.
         */
        void compute();
        
        
        /**
         * @brief Call this function to set a joint-space torque offset based on model information.
         */
        void set_offset(const Eigen::VectorXd& tau_offset);
        
        Eigen::Vector3d getForce(Eigen::VectorXd * tau_res = nullptr) const;
        
        ~ForceEstimation();
        
    private:
        
        const XBot::ModelInterface& _model;
        XBot::MatLogger::Ptr _logger;
        
        std::string _ee_name;
        Eigen::Vector3d _force;
        Eigen::VectorXd _res;
        
        Eigen::JacobiSVD<Eigen::Matrix<double, N_J, 3>> _svd;
        
        Eigen::VectorXd _tau, _nl, _tau_offset;
        Eigen::MatrixXd _J;
        
    };
    
}


#endif