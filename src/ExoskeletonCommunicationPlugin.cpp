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

#include <CentauroAlexUDP/exoskeleton_communication_plugin.h>

#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

SHLIBPP_DEFINE_SHARED_SUBCLASS(ExoskeletonCommunicationPlugin_factory, demo::ExoskeletonCommunicationPlugin, XBot::XBotControlPlugin);

namespace demo {
 
bool ExoskeletonCommunicationPlugin::init_control_plugin(std::string path_to_config_file, 
                                                         XBot::SharedMemory::Ptr shared_memory, 
                                                         XBot::RobotInterface::Ptr robot)
{
    std::cout << "Initializing ExoskeletonCommunicationPlugin..." << std::endl;
    // exoskeleton pipe
    _exoskeleton_pipe.init("exoskeleton_pipe");
    // robot pipe
    _robot_pipe.init("robot_pipe");
    // init pkt
    initPacket();
    
    // Set offsets
    _left_ee_offset.setIdentity();
    _right_ee_offset.setIdentity();
    
    _left_ee_offset.translation()  << -0.144119, 0.142902, -0.063278;
    _right_ee_offset.translation() << -0.144119, -0.142902, -0.063278;
    
    _left_ee_offset.linear()(0,0) = 1.340974;
    _left_ee_offset.linear()(1,1) = 0.995771;
    _left_ee_offset.linear()(2,2) = 0.873206;
    
    _right_ee_offset.linear()(0,0) = 1.340974;
    _right_ee_offset.linear()(1,1) = 0.995771;
    _right_ee_offset.linear()(2,2) = 0.873206;
    
    
    // Advertise shared objects
    _T_left_ee = shared_memory->advertise<Eigen::Affine3d>("w_T_left_ee");
    _T_right_ee = shared_memory->advertise<Eigen::Affine3d>("w_T_right_ee");
    _T_left_elb = shared_memory->advertise<Eigen::Affine3d>("w_T_left_elb");
    _T_right_elb = shared_memory->advertise<Eigen::Affine3d>("w_T_right_elb");
    
    // Allocate shared objects
    _T_left_ee.reset(new Eigen::Affine3d);
    _T_right_ee.reset(new Eigen::Affine3d);
    _T_left_elb.reset(new Eigen::Affine3d);
    _T_right_elb.reset(new Eigen::Affine3d);
    
    _T_left_ee->setIdentity();
    _T_right_ee->setIdentity();
    _T_left_elb->setIdentity();
    _T_right_elb->setIdentity();
    
    _cutoff_freq = 20;
    _sampling_time = 0.001;
    
    robot->sense();
    robot->model().getPose(robot->chain("left_arm").getTipLinkName(), robot->chain("torso").getTipLinkName(), *_T_left_ee);
    robot->model().getPose(robot->chain("right_arm").getTipLinkName(), robot->chain("torso").getTipLinkName(), *_T_right_ee);
    
    // Hard coded offsets...
    KDL::Rotation left_orientation_offset_kdl, right_orientation_offset_kdl;
    Eigen::Matrix3d left_orientation_offset, right_orientation_offset;
    
//     right_orientation_offset_kdl.DoRotX(-3.1415/1.9);
//     left_orientation_offset_kdl.DoRotX(2.1/3.0*3.1415);
//     right_orientation_offset_kdl.DoRotX(0.0);
//     left_orientation_offset_kdl.DoRotX(0.0);
    
    
//     rotationKDLToEigen(left_orientation_offset_kdl, left_orientation_offset);
//     rotationKDLToEigen(right_orientation_offset_kdl, right_orientation_offset);
    
    _T_left_ee->linear() <<  0,  0, -1,
                             1,  0,  0,
                             0, -1,  0;
                             
    
                             
    _T_right_ee->linear() <<  0,  0, -1,
                             -1,  0,  0,
                              0,  1,  0;
                              
     _T_left_ee->linear() = _T_left_ee->linear();
     _T_right_ee->linear() = _T_right_ee->linear();
                             
    _position_left_ee = _position_left_ee_filtered = _position_left_ee_q = _T_left_ee->translation();
    _position_right_ee = _position_right_ee_filtered = _position_right_ee_q = _T_right_ee->translation();
    
    _robot = robot;
    
    std::cout << "Initialized ExoskeletonCommunicationPlugin..." << std::endl;
    
    return true;
}


void ExoskeletonCommunicationPlugin::control_loop(double time, double period)
{
    // filter
// //     double pi = 3.1415;
// //     double b0 = 2*pi*_cutoff_freq*_sampling_time/(1 + 2*pi*_cutoff_freq*_sampling_time);
// //     double a1 = 1/(1 + 2*pi*_cutoff_freq*_sampling_time);
//     b0 = 1;
//     a1 = 0;
    
    // Save previous sample
    _position_left_ee_q = _position_left_ee;
    _position_right_ee_q = _position_right_ee;
    
    // Read from pipes and fill _position_left_ee/_position_right_ee
    _exoskeleton_pipe.xddp_read<CentauroUDP::packet::master2slave>(_exoskeleton_pipe_packet);
    updateReferences();
    
// //     _position_left_ee_filtered = b0*_position_left_ee + a1*_position_left_ee_q;
// //     _position_right_ee_filtered = b0*_position_right_ee + a1*_position_right_ee_q;
    
    // Write to shared memory
    _T_left_ee->translation()  = _left_ee_offset.linear()*_T_left_ee->translation() + _left_ee_offset.translation();
    _T_right_ee->translation() = _right_ee_offset.linear()*_T_right_ee->translation() + _right_ee_offset.translation();
    
//     _T_left_ee->translation() += Eigen::Vector3d(0,0,1) * 0.3 * std::sin(time - get_first_loop_time());
//     _T_right_ee->translation() += Eigen::Vector3d(0,0,1) * 0.3 * std::sin(time - get_first_loop_time());
    
    
    // sense to update the FT values
    _robot->sense();
    // get FT values
    const XBot::ForceTorqueSensor& ft = *_robot->getForceTorque().at("ft_arm1");
    // transform it
    Eigen::Vector3d f;
    ft.getForce(f);
    
//     DPRINTF("RAW f: %f %f %f", f(0), f(1), f(2));
    
//     ft_transform_DEPRECATED(f);
    
    // fill the robot pipe pkt
    _robot_pipe_packet.l_force_x = f(0);
    _robot_pipe_packet.l_force_y = f(1);
    _robot_pipe_packet.l_force_z = f(2);
    // send it to the XDDP robot pipe
    _robot_pipe.xddp_write<CentauroUDP::packet::slave2master>(_robot_pipe_packet);
    
}

bool ExoskeletonCommunicationPlugin::close()
{
    return true;
}

void demo::ExoskeletonCommunicationPlugin::updateReferences()
{

    _T_left_ee->translation().x() = _exoskeleton_pipe_packet.l_position_x;
    _T_left_ee->translation().y() = _exoskeleton_pipe_packet.l_position_y;
    _T_left_ee->translation().z() = _exoskeleton_pipe_packet.l_position_z;
    _T_left_ee->linear() = Eigen::Map<Eigen::Matrix3f>(&_exoskeleton_pipe_packet.l_rotation[0]).cast<double>();
    
 
    _T_right_ee->translation().x() = _exoskeleton_pipe_packet.r_position_x;
    _T_right_ee->translation().y() = _exoskeleton_pipe_packet.r_position_y;
    _T_right_ee->translation().z() = _exoskeleton_pipe_packet.r_position_z;
    _T_right_ee->linear() = Eigen::Map<Eigen::Matrix3f>(&_exoskeleton_pipe_packet.r_rotation[0]).cast<double>();
    
//     std::cout << _T_right_ee->matrix() << std::endl;
    
    
}

void demo::ExoskeletonCommunicationPlugin::initPacket()
{
    _exoskeleton_pipe_packet.l_position_x = 0.3;
    _exoskeleton_pipe_packet.l_position_y = 0.3;
    _exoskeleton_pipe_packet.l_position_z = -0.3;
    
    _exoskeleton_pipe_packet.r_position_x = 0.3;
    _exoskeleton_pipe_packet.r_position_y = -0.3;
    _exoskeleton_pipe_packet.r_position_z = -0.3;
    
    memset((void*) &_robot_pipe_packet, 0, sizeof(_robot_pipe_packet));
}

void demo::ExoskeletonCommunicationPlugin::ft_transform_DEPRECATED(Eigen::Vector3d& f_to_transform)
{
    KDL::Vector f_kdl;
    Eigen::Matrix3d w_R_left;
    
    _robot->model().getOrientation((*_robot).chain("left_arm").getTipLinkName(), w_R_left);
    
    KDL::Rotation L_R_S = KDL::Rotation::RotZ(0.5236*2)*KDL::Rotation::RotX(3.1415);

    tf::vectorEigenToKDL(f_to_transform, f_kdl);
    
    f_kdl = L_R_S * f_kdl;
    
//     DPRINTF("f_kdl: %f %f %f\n", f_kdl.x(), f_kdl.y(), f_kdl.z());
    
    tf::vectorKDLToEigen(f_kdl, f_to_transform);
    
    f_to_transform = w_R_left*f_to_transform;
    
    DPRINTF("WORLD: F = : %f %f %f\n", f_to_transform(0), f_to_transform(1), f_to_transform(2));
}




}
