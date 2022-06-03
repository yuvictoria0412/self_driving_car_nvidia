#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"

#define pi M_PI
#define kp 3
#define ka 8
#define kb -1.5
#define adj 0.1

float g_x, g_y, g_theta;
float n_x, n_y, n_theta, n_theta_pipi;
float e_x, e_y, e_theta, e_theta_pipi;
double p, a, a_pipi, b, b_pipi;
bool isgoal=true;

void pose_callback(const geometry_msgs::Twist &msg) {
	n_x=msg.linear.x;
	n_y=msg.linear.y;
	n_theta=msg.angular.z;
	if(n_theta>pi)
		n_theta_pipi=n_theta-2*pi;	
	else if(n_theta<-pi)
		n_theta_pipi=n_theta+2*pi;
	//ROS_INFO("x=%f, y=%f, th=%f",n_x,n_y,n_theta);

}

void goal_callback(const geometry_msgs::PoseStamped &msg) {
	g_x=msg.pose.position.x;
	g_y=msg.pose.position.y;
	g_theta=atan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y),1-2*(msg.pose.orientation.z*msg.pose.orientation.z+msg.pose.orientation.y*msg.pose.orientation.y));
	
	ROS_INFO("xg=%f, yg=%f, thg=%f",g_x,g_y,g_theta);

}
double calculate_pab(float g_x, float g_y, float g_theta, float n_x, float n_y, float n_theta){
	e_x=g_x-n_x;
	e_y=g_y-n_y;
	e_theta=g_theta-n_theta_pipi;
	p=sqrt(e_x*e_x+e_y*e_y);
	a=-n_theta+atan2(e_y,e_x);
	while(a>pi)
		a-=2*pi;	
	while(a<-pi)
		a+=2*pi;
	b=e_theta;
	while(b>pi)
		b-=2*pi;	
	while(b<-pi)
		b+=2*pi;
	//ROS_INFO("p=%f, a=%f, b=%f",p,a,b);
	return p, a, b;
}

bool isGoal(float g_x, float g_y, float g_theta, float n_x, float n_y, float n_theta){	
	if(abs(g_x-n_x)<0.1 && abs(g_y-n_y)<0.1 && abs(g_theta-n_theta_pipi)<1){
		ROS_INFO("g_x-n_x=%f, g_y-n_y=%f, g_theta-n_theta=%f",abs(g_x-n_x),abs(g_y-n_y),abs(g_theta-n_theta));
		isgoal=true;
	}
	else
		isgoal=false;	
	return isgoal;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pab_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe("/robot_pose", 1000, pose_callback);
    ros::Subscriber sub2 = nh.subscribe("/move_base_simple/goal", 1000, goal_callback);
    
    ros::Rate rate(100);
    ROS_INFO_STREAM("START");

    while (ros::ok()) {
	isGoal(g_x, g_y, g_theta, n_x, n_y, n_theta);
	ROS_INFO("arrive: %d",isgoal);
	geometry_msgs::Twist vel;
	if(isgoal==false){
		calculate_pab(g_x, g_y, g_theta, n_x, n_y, n_theta);
		vel.linear.x=(kp*p)*adj;
		vel.angular.z=(ka*a+kb*b)*adj;
		pub.publish(vel);
		ROS_INFO("v= %f, w= %f", vel.linear.x, vel.angular.z);
	}
	else if(isgoal==true){
		vel.linear.x=0;
		vel.angular.z=0;
		pub.publish(vel);	
	}
	ros::spinOnce();
	rate.sleep();
    }
    return 0;
}

