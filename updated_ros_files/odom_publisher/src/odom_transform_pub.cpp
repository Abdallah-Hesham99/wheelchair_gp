#include <ros/ros.h>
#include<iostream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2.h>
#include <geometry_msgs/TransformStamped.h>
#include<ros/console.h>

#include <nav_msgs/Odometry.h>
//#include <std_msgs/Int16.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
 double right_ticks_diff=0, left_ticks_diff=0;
long ppr = 20*32; 
double wheel_diameter=0.3;
double center_dist = 0.61;
double distance_per_tick = (3.14*wheel_diameter)/ppr;
double x,y,z,theta=0;
double vx=0,vy=0,w=0;
double old_time;

//ros::Time new_time, old_time; 
nav_msgs::Odometry odom;
geometry_msgs::Twist spd_msg;
ros::Publisher odom_pub;
ros::Publisher spd_pub;


void get_right_wheel_ticks(const std_msgs::Int16 right_ticks_msg)
{
right_ticks_diff = right_ticks_msg.data;
ROS_DEBUG("i RECIEVED RIGHT");

}

void get_left_wheel_ticks(const std_msgs::Int16 left_ticks_msg)
{
ROS_DEBUG("i RECIEVED LEFT");
left_ticks_diff = left_ticks_msg.data;


}


void cal_transforms()
{
static tf2_ros::TransformBroadcaster odom_bc;
geometry_msgs:: TransformStamped odom_trans;

tf2::Quaternion odom_quat;

	double new_time = ros::Time::now().toSec(); 
	ros::Time cur_time;
	cur_time =ros::Time::now();
	
	std::cout<<" right_diff "<<right_ticks_diff<<" left tick diff "<<left_ticks_diff<<"\n";
	
	
	/*if (right_ticks_diff>100000)
{right_ticks_diff=(4294967296-right_ticks_diff);
right_ticks_diff=0-right_ticks_diff;
}
	
	
	if (left_ticks_diff>100000)
{left_ticks_diff=(4294967296-left_ticks_diff);
left_ticks_diff=0-left_ticks_diff;
}*/
	
	
	
	
	
	
	double sr = distance_per_tick * right_ticks_diff ;
	double sl = distance_per_tick * left_ticks_diff ;
	double mean = (sl+sr)/2.0;
	double dx = mean*cos(theta);
	double dy = mean*sin(theta);
	double dtheta = (sr-sl)/center_dist;
	
	std::cout<<" sr "<<sr<<" sl "<<sl<<" mean "<<mean<<"\n";
	
	std::cout<<" dx "<<dx<<" dy "<<dy<<" dtheta "<<dtheta<<"\n";
	x += dx;
	y += dy;
	theta += dtheta;
	
	if(theta > 6.28)
	theta -= 6.28;
	else if (theta<-6.28)
	theta +=6.28;
	
	
	
	vx = dx/(new_time-old_time);
	vy = dy/(new_time-old_time);
	w = dtheta/(new_time-old_time);
	
	//geometry_msgs::Quaternion odom_quat = tf2_ros::createQuaternionMsgFromYaw(theta)
	//geometry_msgs::Quaternion

	odom_quat.setRPY(0.0,0.0,theta);
	odom_trans.header.stamp = cur_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
//	odom_trans.transform.rotation = odom_quat;
	odom_trans.transform.rotation.x =odom_quat.x();
	odom_trans.transform.rotation.y =odom_quat.y();	
	odom_trans.transform.rotation.z =odom_quat.z();
	odom_trans.transform.rotation.w =odom_quat.w();	
	
	std::cout<<"the quat z is "<<odom_quat.z()<<"\n"<<"dt is"<<(new_time-old_time)<<"\n";
	odom_bc.sendTransform(odom_trans);

	odom.header.stamp = cur_time;
	odom.header.frame_id = "odom";
	odom.pose.pose.position.x =x;
	odom.pose.pose.position.y=y;
	odom.pose.pose.position.z=0.0;
//	odom.pose.pose.orientation = odom_quat;
	odom.pose.pose.orientation.x=odom_quat.x();
	odom.pose.pose.orientation.y=odom_quat.y();
	odom.pose.pose.orientation.z=odom_quat.z();
	odom.pose.pose.orientation.w=odom_quat.w();
	//twist msg 
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y=vy;
	odom.twist.twist.angular.z = w;

	old_time = new_time;
	
	spd_msg.linear.x = vy;
	spd_msg.angular.z = w;
	
}






int main(int argc, char** argv)
{

ros::init(argc,argv,"Hardware_Odometry_publisher");

ros:: NodeHandle nd;

ros::Subscriber rsub = nd.subscribe("wheel_encoders/right_ticks",1000,get_right_wheel_ticks);
ros::Subscriber lsub = nd.subscribe("wheel_encoders/left_ticks",1000,get_left_wheel_ticks);
ros::Publisher odom_pub = nd.advertise<nav_msgs::Odometry>("odom",50);
ros::Publisher spd_pub = nd.advertise<geometry_msgs::Twist>("/control/current_speed",50);
ros::Rate r(10);

old_time = ros::Time::now().toSec();
while(ros::ok())
{

cal_transforms();
ros::spinOnce();
	odom_pub.publish(odom);
	ROS_DEBUG("i AM PUBLISHING");
r.sleep();


}
return 0;


}



