#include<ros/ros.h>
#include<geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centauro_ros_autonomous_grasping");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe<geometry_msgs::TransformStamped>("");
    return 0;
}
