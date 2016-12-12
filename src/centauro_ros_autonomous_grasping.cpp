#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <CentauroUDP/pipes.h>


void callback ( geometry_msgs::TransformStamped::ConstPtr msg, CentauroUDP::XDDP_pipe& pipe){
    
    Eigen::Affine3d pose;
    
    tf::transformMsgToEigen(msg->transform, pose);
    
    pipe.xddp_write(pose);
    
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centauro_ros_autonomous_grasping_subscriber");
    ros::NodeHandle nh;
    
    CentauroUDP::XDDP_pipe ros_autonomous_grasping_pipe;
    ros_autonomous_grasping_pipe.init("ros_autonomous_grasping_pipe");
    
    
    Eigen::Affine3d ee_pose;
    ee_pose.setIdentity();
    
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>("/centauro_upperbody/ee_pose", 
                                                                        1,
                                                                        boost::bind(callback, _1, ros_autonomous_grasping_pipe)
                                                                       );
    ros::spin();
    
    return 0;
}
