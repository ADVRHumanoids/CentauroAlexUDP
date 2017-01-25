#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/bind.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <CentauroUDP/pipes.h>
#include <tf2_ros/transform_listener.h>



int main(int argc, char** argv)
{

    std::cout << "optitrack_to_xddp initialized!" << std::endl;


    ros::init(argc, argv, "optitrack_to_xddp");
    ros::NodeHandle nh;
    int optitrack_fd;
    bool open_ok = false;
    
    while( !open_ok ) {
        optitrack_fd = open((pipe_prefix+std::string("optitrack_pipe")).c_str(), O_WRONLY);
        std::cout << "waiting for XDDP pipes to be initialized by the RT plugin..."<< std::endl;
        if(optitrack_fd > 0) open_ok = true;
        
        sleep(1);
    }
    std::cout << "optitrack_fd: " << optitrack_fd << std::endl;
    


    Eigen::Affine3d ee_pose;
    ee_pose.setIdentity();



    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::TransformStamped transform;

    ros::Rate loop_rate(100);

    while(ros::ok()){

        try{

            std::cout << "Before lookupTransform" << std::endl;

            transform = tf_buffer.lookupTransform("world", "Operator/TransformedRee", ros::Time(0));

            Eigen::Affine3d pose;
            pose.setIdentity();


            tf::transformMsgToEigen(transform.transform, pose);

            std::cout << "Ref from optitrack : " << pose.translation().transpose() << " --> " << transform.header.stamp.toNSec() << std::endl;

            int bytes = write(optitrack_fd, (void *)(&pose), sizeof(pose));
        }
        catch(tf2::TransformException e){
            std::cout << "Exception: " << e.what() << std::endl;
        }

        loop_rate.sleep();

    }


    ros::spin();

    return 0;
}
