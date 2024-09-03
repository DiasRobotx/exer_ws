#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>

ros::Publisher velocity_publisher;

void moveTurtle(float linear_speed, float angular_speed, float duration)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = linear_speed;
    vel_msg.angular.z = angular_speed;

    ros::Rate loop_rate(10);
    int ticks = int(duration * 10);

    for (int i = 0; i < ticks; i++)
    {
        velocity_publisher.publish(vel_msg);
        loop_rate.sleep();
    }
}

void moveInSquare()
{
    for (int i = 0; i < 4; i++)
    {
        moveTurtle(1.0, 0.0, 2.0);  // Move forward
        moveTurtle(0.0, 1.57, 1.0); // Rotate 90 degrees (approx)
    }
}

void moveInTriangle()
{
    for (int i = 0; i < 3; i++)
    {
        moveTurtle(1.0, 0.0, 2.0);  // Move forward
        moveTurtle(0.0, 2.09, 1.5); // Rotate 120 degrees (approx)
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_subscriber");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    // Use service to spawn a new turtle at the center
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.5;
    spawn_srv.request.y = 5.5;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle_YOURNAME";
    spawn_client.call(spawn_srv);

    // Move the turtle along a square trajectory
    moveInSquare();

    // Move the turtle along a triangular trajectory
    moveInTriangle();

    return 0;
}

