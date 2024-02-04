#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
//轴速度发布者
ros::Publisher frjoint_pub;
ros::Publisher fljoint_pub;
ros::Publisher rrjoint_pub;
ros::Publisher rljoint_pub;
//获取速度时的时间
ros::Time get_vel_time;
//获取的速度
geometry_msgs::Twist vel;
//获得速度内容
void velCB(const geometry_msgs::TwistConstPtr msg){
    vel = *msg;
    get_vel_time = ros::Time::now();
}
int main(int argc, char** argv){
	ros::init(argc,argv,"mecanum_controller");
    ros::NodeHandle nh;
   
        //左右轴距
        double w = 0.2;
        //前后轴距
        double l = 0.2;
        //轮半径
        double r = 0.06;
     
        ros::param::get("/wheelbase",w);
        ros::param::get("/trackwidth",l);
       // printf("%.3f,%3f",w,l);
        
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel",1,velCB);
    
    frjoint_pub = nh.advertise<std_msgs::Float64>("/hero/right_front_j/command",1000);
    fljoint_pub = nh.advertise<std_msgs::Float64>("/hero/left_front_j/command",1000);
    rrjoint_pub = nh.advertise<std_msgs::Float64>("/hero/right_back_j/command",1000);
    rljoint_pub = nh.advertise<std_msgs::Float64>("/hero/left_back_j/command",1000);
    
    get_vel_time = ros::Time::now();
    
    //在循环中进行运动学逆解
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        
        std_msgs::Float64 v1,v2,v3,v4;
        // if(ros::Time::now()-get_vel_time > ros::Duration(1)){
        //     //大于1秒没收到新的速度指令，则给四个轴发送0速度
        //     v1.data = 0;
        //     v2.data = 0;
        //     v3.data = 0;
        //     v4.data = 0;
        //     frjoint_pub.publish(v1);
        //     fljoint_pub.publish(v2);
        //     rljoint_pub.publish(v3);
        //     rrjoint_pub.publish(v4);
        // printf("%.3f,%.3f,%.3f,%.3f\r\n",v1.data,v2.data,v3.data,v4.data);
            
        //     //结束本次循环
        //     continue;
        // }
         //进行运动学逆解
        
        v1.data = (vel.linear.x - vel.linear.y + vel.angular.z*(w+l))/r; //d
        v2.data = (vel.linear.x + vel.linear.y - vel.angular.z*(w+l))/r; //c
        v3.data = (vel.linear.x - vel.linear.y - vel.angular.z*(w+l))/r; //a
        v4.data = (vel.linear.x + vel.linear.y + vel.angular.z*(w+l))/r; //b
        
        printf("%.3f,%.3f,%.3f,%.3f\r\n",v1.data,v2.data,v3.data,v4.data);
        frjoint_pub.publish(v1);
        fljoint_pub.publish(v2);
        rljoint_pub.publish(v3);
        rrjoint_pub.publish(v4);      
        
        rate.sleep();
    }
	return 0;

}
