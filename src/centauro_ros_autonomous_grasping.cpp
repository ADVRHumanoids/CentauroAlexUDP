#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <CentauroUDP/pipes.h>


void callback ( geometry_msgs::TransformStamped::ConstPtr msg, int fd){

    Eigen::Affine3d pose;

    tf::transformMsgToEigen(msg->transform, pose);

    int bytes = write(fd, (void *)(&pose), sizeof(Eigen::Affine3d));




}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "optitrack_to_xddp");
    ros::NodeHandle nh;

    int optitrack_fd = open((pipe_prefix+std::string("optitrack_pipe")).c_str(), O_WRONLY);


    Eigen::Affine3d ee_pose;
    ee_pose.setIdentity();

    ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>("Operator/TransformedRee",
                                                                        1,
                                                                        boost::bind(callback, _1, optitrack_fd)
                                                                       );
    ros::spin();

    return 0;
}
