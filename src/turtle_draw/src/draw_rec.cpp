#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"

#define PI acos(-1)
#define err 0.007
//const int dx[8] = {1, 0, 0, 0, -1, 0, 0, 0};
//const int dy[8] = {0, 0, 1, 0, 0, 0, -1, 0};

int i;
int theta;
int count;
float ori_theta[4] = {0, PI/2, -PI, -PI/2};
void pose_callback(const turtlesim::Pose &msg) {
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
			if (i < 5) {
				if (msg.theta < 0 && msg.theta > -1.57) {
					theta = 1;
					break;
				}
				else if (msg.theta < 0 && msg.theta < -1.57) {
					theta = -1, count++;
					break;
				}
			}
			else {
				if (msg.theta > 0 && msg.theta > 1.57) {
					theta = 1;
					break;
				}
				else if (msg.theta > 0 && msg.theta < 1.57) {
					theta = -1, count++;
					break;
				}
			}
			if (msg.theta >= ori_theta[i/2] + PI/2 - err && msg.theta <= ori_theta[i/2] + PI/2) {
				 theta = 0, i++;
			}
			else if (msg.theta < ori_theta[i/2] + PI/2) {
				theta = 1;
			}
			else {
				theta = -1;
				count++;
			}
			if (count > 10) theta = 0, i++, count = 0;
			if (i == 8) i = 0;
			break;
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "draw_rec");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, pose_callback);
    while(!sub.getNumPublishers() && ros::ok()){
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, pose_callback);
	ROS_INFO("no subscribe");
    }
    ros::Rate rate(1);
    ROS_INFO_STREAM("START");
    while (ros::ok()) {
    	geometry_msgs::Twist vel;
		if (i % 2 == 0) {
			vel.linear.x = 1;
			vel.linear.y = 0;
		}
		else {
			vel.angular.z = theta;
		}
		pub.publish(vel);
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}
