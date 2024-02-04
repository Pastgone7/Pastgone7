#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
//历史数据
double last_rf = 0;
double last_lf = 0;
double last_lr = 0;
double last_rr = 0;

//历史时间
ros::Time last_time;

//历史里程
double x = 0;
double y = 0;
double th = 0;

tf2_ros::TransformBroadcaster *odom_broadcaster;
ros::Publisher odom_pub;

double v1 = 0;
double v2 = 0;
double v3 = 0;
double v4 = 0;

void stateCB1(std_msgs::Float64 msg){
    v1 = msg.data;
}
void stateCB2(std_msgs::Float64 msg){
    v2 = msg.data;
}
void stateCB3(std_msgs::Float64 msg){
    v3 = msg.data;
}
void stateCB4(std_msgs::Float64 msg){
    v4 = msg.data;
}
void calodom(){
    double dx;
    double dy;
    double dth;

    
    double dt = (ros::Time::now() - last_time).toSec();
    last_time = ros::Time::now();
    if(dt > 2){
         //时间异常
         return;
    }
    
    double r = 0.06;
    double w = 0.2;
    double l = 0.2;
    
    dx = r*0.25*(v1+v2+v3+v4);
    dy = r*0.25*(-v3+v4+v2-v1);
    dth = r*(-v3+v4-v2+v1)/((l+w)*4);
    
    x += dx*cos(th)*dt - dy*sin(th)*dt;
    y += dx*sin(th)*dt + dy*cos(th)*dt;
    th += dth*dt;
    
    //将角度变为四元数
    tf2::Quaternion q;
    q.setRPY(0,0,th);
    
    nav_msgs::Odometry odom;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = q.getW();
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = dy;
    odom.twist.twist.angular.z = dth;
    odom_pub.publish(odom);
    
    printf("%.3f,%.3f,%.3f\r\n",x,y,th);
    //发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.frame_id = "odom";
    odom_trans.header.stamp  = ros::Time::now();
    odom_trans.transform.rotation.w = q.getW();
    odom_trans.transform.rotation.x = q.getX();
    odom_trans.transform.rotation.y = q.getY();
    odom_trans.transform.rotation.z = q.getZ();
    
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    
    odom_broadcaster->sendTransform(odom_trans);
}
int main(int argc, char **argv){
	ros::init(argc,argv,"mecanum_odom");
    ros::NodeHandle nh;
    //发布里程信息
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom",10);
    //ros::Subscriber joint_sub = nh.subscribe("/joint_states",1,stateCB);
    //获取关节返回信息
    ros::Subscriber joint_sub1 = nh.subscribe("/hero/right_front_j/command",1,stateCB1);
    ros::Subscriber joint_sub2 = nh.subscribe("/hero/left_front_j/command",1,stateCB2);
    ros::Subscriber joint_sub3 = nh.subscribe("/hero/left_back_j/command",1,stateCB3);
    ros::Subscriber joint_sub4 = nh.subscribe("/hero/right_back_j/command",1,stateCB4);
    
    odom_broadcaster = new tf2_ros::TransformBroadcaster();
    
    last_time = ros::Time::now();
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        //运动学正解
        calodom();
        rate.sleep();
    }

	return 0;
}
