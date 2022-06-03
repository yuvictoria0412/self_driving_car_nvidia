#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"

#define PI acos(-1)

const int dx[8] = {1, 0, 0, 0, -1, 0, 0, 0};
const int dy[8] = {0, 0, 1, 0, 0, 0, -1, 0};

int i;
void pose_callback(const turtlesim::Pose &msg) {
    ROS_INFO_STREAM("x: " << msg.x << ", y: " << msg.y << ", theta: " << msg.theta << ", i: " << i);
	switch (i) {
		case 0:
			if (msg.x > 7) i++;
			break;
		case 2:
			if (msg.y > 7) i++;
			break;
		case 4:
			if (msg.x < 5) i++;
			break;
		case 6:
			if (msg.y < 5) i++;
			break;
		default:
			if (i == 7) i = 0;
			else i++;
			break;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "draw_rec");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, pose_callback);
    ros::Rate rate(10);
	ROS_INFO_STREAM("PI: " << PI);
    while (ros::ok()) {
    	geometry_msgs::Twist vel;
		vel.linear.x = dx[i];
		vel.linear.y = dy[i];
		pub.publish(vel);
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
