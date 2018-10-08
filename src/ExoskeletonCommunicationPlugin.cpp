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
#include <geometry_msgs/WrenchStamped.h>
#include <XBotInterface/Utils.h>

#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

#define N_FILTER_SAMPLE 5000


REGISTER_XBOT_PLUGIN(AlexCommunication, demo::ExoskeletonCommunicationPlugin)


namespace demo {

bool ExoskeletonCommunicationPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    // exoskeleton pipe
    _exoskeleton_pipe.init("exoskeleton_pipe");
    // robot pipe
    _robot_pipe.init("robot_pipe");
    // init pkt
    initPacket();

    // get robot
    _robot = handle->getRobotInterface();

    auto _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());

    YAML::Node yaml_file = YAML::LoadFile(handle->getPathToConfigFile());
    XBot::Cartesian::ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"], _model);

    // NOTE it should be     _ci = std::make_shared<OpenSotImpl>(_model, ik_problem);
//     std::string impl_name = yaml_file["CartesianInterface"]["solver"].as<std::string>();
    _ci = std::make_shared<XBot::Cartesian::CartesianInterfaceImpl>(_model, ik_problem);
    // TEST trying Reflexxes HACK assuming 1 ms period
    _ci->enableOtg(0.001);
    for(std::string task : _ci->getTaskList()) {
        _ci->setVelocityLimits(task, 0.2, 0.1);
        _ci->setAccelerationLimits(task, 1.0, 0.5);
    }

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
    _T_left_ee = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_left_ee");
    _T_right_ee = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_right_ee");
    _T_left_elb = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_left_elb");
    _T_right_elb = handle->getSharedMemory()->getSharedObject<Eigen::Affine3d>("w_T_right_elb");

    _T_left_ee.set(Eigen::Affine3d::Identity());
    _T_right_ee.set(Eigen::Affine3d::Identity());
    _T_left_elb.set(Eigen::Affine3d::Identity());
    _T_right_elb.set(Eigen::Affine3d::Identity());


    l_ft_filter.setOmega(2 * M_PI * 1); // 1 Hz
    l_ft_filter.setDamping(1.0);
    l_ft_filter.setTimeStep(0.001);

    r_ft_filter.setOmega(2 * M_PI * 1); // 1 Hz
    r_ft_filter.setDamping(1.0);
    r_ft_filter.setTimeStep(0.001);

    _robot->sense();

    _robot->model().getPose(_robot->chain("left_arm").getTipLinkName(), _robot->chain("torso").getTipLinkName(), _aux);
    _T_left_ee.set(_aux);

    _robot->model().getPose(_robot->chain("right_arm").getTipLinkName(), _robot->chain("torso").getTipLinkName(), _aux);
    _T_right_ee.set(_aux);


    _aux.translation() = _T_right_ee.get().translation();

    _aux.linear() <<   0,  0, -1,
                       1,  0,  0,
                       0, -1,  0;

    _T_right_ee.set(_aux);


    _aux.translation() = _T_left_ee.get().translation();

    _aux.linear() <<  0,  0, -1,
                     -1,  0,  0,
                      0,  1,  0;

    _T_left_ee.set(_aux);


    _position_left_ee = _position_left_ee_filtered = _position_left_ee_q = _T_left_ee.get().translation();
    _position_right_ee = _position_right_ee_filtered = _position_right_ee_q = _T_right_ee.get().translation();

    _l_ft_offset.setZero();
    _r_ft_offset.setZero();

    // HACK HAND
    _left_hand = _robot->getHand(20);
    
    // force estimation
    _force_est = std::make_shared<centauro::ForceEstimation>(_robot->model());
    _force_est->getForce(&_tau_offset_right);
    _force_est->getForce(&_tau_res_right);
    
    _pub_rt = handle->getRosHandle()->advertise<geometry_msgs::WrenchStamped>("/force_estimation", 3);

    _logger = XBot::MatLogger::getLogger("/tmp/AlexLogger");


    return true;
}


void ExoskeletonCommunicationPlugin::control_loop(double time, double period)
{
    
    // estimate force based on joint torque
    _force_est->compute();
    geometry_msgs::WrenchStamped f_msg;
    f_msg.header.stamp = ros::Time(XBot::get_time_ns(CLOCK_REALTIME) * 1e-9);
    
    f_msg.wrench.force.x = _force_est->getForce().x();
    f_msg.wrench.force.y = _force_est->getForce().y();
    f_msg.wrench.force.z = _force_est->getForce().z();
    
    _pub_rt->pushToQueue(f_msg);

    // Save previous sample
    _position_left_ee_q = _position_left_ee;
    _position_right_ee_q = _position_right_ee;

    // Read from pipes and fill _position_left_ee/_position_right_ee
    _exoskeleton_pipe.xddp_read<CentauroUDP::packet::master2slave>(_exoskeleton_pipe_packet);
    updateReferences();

    _aux = _T_left_ee.get();
    _aux.translation() = _left_ee_offset.linear() * _aux.translation() + _left_ee_offset.translation();
    _ci->setPoseReference("arm1_8", _aux);

     _aux = _T_right_ee.get();
    _aux.translation() = _right_ee_offset.linear() * _aux.translation() + _right_ee_offset.translation();
    _ci->setPoseReference("arm2_8", _aux);
    
    if(!_ci->update(time, period))
    {
        XBot::Logger::error("CartesianInterface: unable to solve \n");
        return;
    }
    
    // NOTE the genius
    Eigen::Affine3d ref_otg;
   
    _ci->getPoseReference("arm1_8", ref_otg);
    _T_left_ee.set(ref_otg);
   
    _ci->getPoseReference("arm2_8", ref_otg);
    _T_right_ee.set(ref_otg);

    if(_left_hand) {
        _left_hand->grasp(_exoskeleton_pipe_packet.l_handle_trigger * 0.75);
    }

    // sense to update the FT values
    _robot->sense();

    // get left FT values
    XBot::ForceTorqueSensor::ConstPtr ft;
    if(_robot->getForceTorque().count("ft_arm1")) {
        ft = _robot->getForceTorque().at("ft_arm1");
    }

    // transform it
    Eigen::Vector6d w = getWorldWrench(ft) - _l_ft_offset;

    //Logger::info(Logger::Severity::HIGH, "TRANSFORMED Left f: %f %f %f", w(0), w(1), w(2));

    // fill the robot pipe pkt
    _robot_pipe_packet.l_force_x = w(0);
    _robot_pipe_packet.l_force_y = w(1);
    _robot_pipe_packet.l_force_z = w(2);
    _robot_pipe_packet.l_torque_x = w(3);
    _robot_pipe_packet.l_torque_y = w(4);
    _robot_pipe_packet.l_torque_z = w(5);

    // get right FT values
    if(_robot->getForceTorque().count("ft_arm2")) {
        ft = _robot->getForceTorque().at("ft_arm2");
    }

    w = getWorldWrench(ft) - _r_ft_offset;

    //Logger::info(Logger::Severity::HIGH, "TRASFORMED Right f: %f %f %f", w(0), w(1), w(2));

    // fill the robot pipe pkt
    // NOTE we use Arturo's estimation
    
    const double max_force = 40.0;
    Eigen::Vector3d f_sat = _force_est->getForce().array().min(max_force).max(-max_force);
    
    _robot_pipe_packet.r_force_x = f_sat.x();
    _robot_pipe_packet.r_force_y = f_sat.y();
    _robot_pipe_packet.r_force_z = f_sat.z();
    _robot_pipe_packet.r_torque_x = 0.0;
    _robot_pipe_packet.r_torque_y = 0.0;
    _robot_pipe_packet.r_torque_z = 0.0;
    


    // send it to the XDDP robot pipe
    _robot_pipe.xddp_write<CentauroUDP::packet::slave2master>(_robot_pipe_packet);

    if( current_command.str() == "reset_ft") {
    	_reset_ft = true;
    }
    
    if( current_command.str() == "filter_OFF") {
    	for(std::string task : _ci->getTaskList()) {
            _ci->setVelocityLimits(task, 0.5, 0.5);
            _ci->setAccelerationLimits(task, 1.0, 1.0);
        }
    }
    
    if( current_command.str() == "filter_ON") {
    	for(std::string task : _ci->getTaskList()) {
            _ci->setVelocityLimits(task, 0.2, 0.1);
            _ci->setAccelerationLimits(task, 1.0, 0.5);
	}
    }

    if( _reset_ft ) {
	    resetFT();
    }

    _logger->add("l_ft_offset", _l_ft_offset);
    _logger->add("r_ft_offset", _r_ft_offset);

    _logger->add("l_handle_trigger", _exoskeleton_pipe_packet.l_handle_trigger);

}

void ExoskeletonCommunicationPlugin::resetFT() {
    
    
    
    Eigen::Vector6d l_wrench, r_wrench;
    l_wrench.setZero();
    r_wrench.setZero();

    XBot::ForceTorqueSensor::ConstPtr ft;
    if(_robot->getForceTorque().count("ft_arm1")) {
        ft = _robot->getForceTorque().at("ft_arm1");
	ft->getWrench(l_wrench);
    }

    if(_robot->getForceTorque().count("ft_arm2")) {
        ft = _robot->getForceTorque().at("ft_arm2");
        ft->getWrench(r_wrench);
    }

    // first iteration
    if( _reset_ft_count == 0 ) {

        l_ft_filter.reset(l_wrench);
        r_ft_filter.reset(r_wrench);
        _tau_offset_right.setZero();

    }

    // increment filter sample counter
    _reset_ft_count++;

    _l_ft_offset = l_ft_filter.process(l_wrench);
    _r_ft_offset = r_ft_filter.process(r_wrench);
    _force_est->getForce(&_tau_res_right);
    _tau_offset_right += _tau_res_right;
    


    // check for the trh
    if( _reset_ft_count == N_FILTER_SAMPLE ) {
    	_reset_ft = false;
        _reset_ft_count = 0;
        _tau_offset_right /= (N_FILTER_SAMPLE + 1);
        _force_est->set_offset(_tau_offset_right);
    }
}

bool ExoskeletonCommunicationPlugin::close()
{
    //_logger->flush();
    return true;
}

void demo::ExoskeletonCommunicationPlugin::updateReferences()
{

    _aux = _T_left_ee.get();

    _aux.translation().x() = _exoskeleton_pipe_packet.l_position_x;
    _aux.translation().y() = _exoskeleton_pipe_packet.l_position_y;
    _aux.translation().z() = _exoskeleton_pipe_packet.l_position_z;
    _aux.linear() = Eigen::Map<Eigen::Matrix3f>(&_exoskeleton_pipe_packet.l_rotation[0]).cast<double>();

    _T_left_ee.set(_aux);


    _aux = _T_right_ee.get();

    _aux.translation().x() = _exoskeleton_pipe_packet.r_position_x;
    _aux.translation().y() = _exoskeleton_pipe_packet.r_position_y;
    _aux.translation().z() = _exoskeleton_pipe_packet.r_position_z;
    _aux.linear() = Eigen::Map<Eigen::Matrix3f>(&_exoskeleton_pipe_packet.r_rotation[0]).cast<double>();

    _T_right_ee.set(_aux);



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

Eigen::Vector6d demo::ExoskeletonCommunicationPlugin::getWorldWrench(XBot::ForceTorqueSensor::ConstPtr ft)
{
    Eigen::Vector6d ft_wrench;

    if( !ft ) {
        return 0 * ft_wrench;
    }

    ft->getWrench(ft_wrench);

    Eigen::Matrix3d W_R_ft_frame;
    _robot->model().getOrientation(ft->getSensorName(), W_R_ft_frame);


    return XBot::Utils::GetAdjointFromRotation(W_R_ft_frame) * ft_wrench;

}




}
