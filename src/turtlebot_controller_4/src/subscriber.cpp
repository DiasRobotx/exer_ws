#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>

ros::Publisher velocity_publisher;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
    ROS_INFO("Turtle position: x=%f, y=%f", msg->x, msg->y);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 1.0;
    velocity_publisher.publish(vel_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_subscriber");
    ros::NodeHandle nh;

    // Clear the background using the /clear service
    ros::ServiceClient clear_client = nh.serviceClient<std_srvs::Empty>("/clear");
    std_srvs::Empty srv;
    clear_client.call(srv);

    // Spawn a new turtle at position (5.0, 5.0) with orientation 0.0
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 5.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "new_turtle";
    spawn_client.call(spawn_srv);

    // Subscribe to the pose of the original turtle
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 10, turtleCallback);
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    ros::spin();

    return 0;
}

