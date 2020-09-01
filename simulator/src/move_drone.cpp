#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>


void drone_odom_callback(const nav_msgs::Odometry &msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped odom_trans;

	odom_trans.header.stamp = msg.header.stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "drone_link";

    odom_trans.transform.translation.x = msg.pose.pose.position.x;
    odom_trans.transform.translation.y = msg.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.5;

    tf2::Quaternion q_initial;
    q_initial.setRPY(M_PI/2,0,M_PI/2);

    tf2::Quaternion q;
    tf2::fromMsg(msg.pose.pose.orientation,q);
    odom_trans.transform.rotation = tf2::toMsg(q_initial);

    br.sendTransform(odom_trans);

}




int main(int argc,char **argv){
	ros::init(argc,argv,"move_drone");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	// ros::Subscriber drone_odom_sub = n.subscribe("/drone/odom",10,&drone_odom_callback);


	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		geometry_msgs::Twist twist_cmd;

		twist_cmd.linear.x = 0;
		twist_cmd.linear.y = 0;
		twist_cmd.linear.z = 0;

		twist_cmd.angular.x = 0;
		twist_cmd.angular.y = 0;
		twist_cmd.angular.z = 0;

		pub.publish(twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
