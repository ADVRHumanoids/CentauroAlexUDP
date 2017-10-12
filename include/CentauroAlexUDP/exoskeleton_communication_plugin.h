/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __CENTAURO_BONN_REVIEW_PLUGINS_EXOSKELETON_COMMUNICATION_H__
#define __CENTAURO_BONN_REVIEW_PLUGINS_EXOSKELETON_COMMUNICATION_H__


#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XBotPipes.h>

#include <CentauroAlexUDP/packet/master2slave.h>
#include <CentauroAlexUDP/packet/slave2master.h>

namespace demo {

    class ExoskeletonCommunicationPlugin : public XBot::XBotControlPlugin {
      
    public:
        
        virtual bool init_control_plugin(std::string path_to_config_file, 
                                         XBot::SharedMemory::Ptr shared_memory, 
                                         XBot::RobotInterface::Ptr robot);
        virtual void control_loop(double time, double period);
        virtual bool close();
            
    protected:
        
    private:
        
        void updateReferences();
        void initPacket();
        
        inline void rotationEigenToKDL(const Eigen::Matrix3d& eigen_rotation, KDL::Rotation& kdl_rotation) const;
        inline void rotationKDLToEigen(const KDL::Rotation& kdl_rotation, Eigen::Matrix3d& eigen_rotation) const;
        
        void ft_transform_DEPRECATED( Eigen::Vector3d& f_to_transform );
        
        XBot::RobotInterface::Ptr _robot;
        
        double _cutoff_freq;
        double _sampling_time;
        
        Eigen::Vector3d _position_left_ee, _position_right_ee;
        Eigen::Vector3d _position_left_ee_filtered, _position_right_ee_filtered;
        Eigen::Vector3d _position_left_ee_q, _position_right_ee_q;
        Eigen::Vector3d _position_left_ee_qq, _position_right_ee_qq;
        
        Eigen::Affine3d _left_ee_offset, _right_ee_offset;
        
        XBot::XDDP_pipe _exoskeleton_pipe;
        XBot::XDDP_pipe _robot_pipe;
        
        CentauroUDP::packet::master2slave _exoskeleton_pipe_packet;
        CentauroUDP::packet::slave2master _robot_pipe_packet;
        
        XBot::SharedObject<Eigen::Affine3d> _T_left_ee, 
                                            _T_right_ee, 
                                            _T_left_elb, 
                                            _T_right_elb;

        
    };
    
inline void demo::ExoskeletonCommunicationPlugin::rotationEigenToKDL(const Eigen::Matrix3d& eigen_rotation, KDL::Rotation& kdl_rotation) const
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            kdl_rotation.data[3*i+j] = eigen_rotation(i,j); 
        }
    }
}

inline void demo::ExoskeletonCommunicationPlugin::rotationKDLToEigen(const KDL::Rotation& kdl_rotation, Eigen::Matrix3d& eigen_rotation) const
{
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            eigen_rotation(i,j) = kdl_rotation.data[3*i+j]; 
        }
    }
}
    
}
#endif