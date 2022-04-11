#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include"turtlesim/Pose.h"
#include<iostream>
#include<cmath>
#include<string>
#include<random>

using namespace std;


struct destination_position{
	float x;
	float y;
	float angle;
};

struct position{
	float x;
	float y;
	float z;
	float theta;
};
 
struct vel{
	float x;
	float y;
	float z;
	float angular;
};

position current_position;//defining the structure to store the current position data of the turtle

vel current_velocity;//defining the structure to store the current velocity data of the turtle

turtlesim::Pose Position;//it will read the exact position of the turtle and publish on the desired topic 

turtlesim::Pose noisy_pose;//it will read the  position of the turtle mixed with gaussian noise and publish on the desired topic


void turtleX_pose(const turtlesim::PoseConstPtr &pose)
{
	// reading the current position data of the turtle as an input from the turtlesim/Pose.msg
	current_position.x=pose->x;
	current_position.y=pose->y;
	current_position.z=0;
	current_position.theta=pose->theta;
        
        //reading the current position data to be published on another topic
        Position.x=pose->x;
        Position.y=pose->y;
        Position.theta=pose->theta;
        Position.linear_velocity=pose->linear_velocity;
        Position.angular_velocity=pose->angular_velocity;

        //reading data of the current position of the turtle in order to mix it with the gaussian noise
        noisy_pose.x=pose->x;
        noisy_pose.y=pose->y;
        noisy_pose.theta=pose->theta;
        noisy_pose.linear_velocity=pose->linear_velocity;
        noisy_pose.angular_velocity=pose->angular_velocity;

	//reading the current velocity data of the turtle as an input from the turtlesim/Pose.msg
	current_velocity.x=(pose->linear_velocity)*(cos(pose->theta));
	current_velocity.y=(pose->linear_velocity)*(sin(pose->theta));
	current_velocity.z=0;
	current_velocity.angular=pose->angular_velocity;


	ROS_INFO("%f,%f,%f",pose->x,pose->y,pose->theta);
	
 
	
}

float kp=1.5,kp_theta=1.5;
float kd=2,kd_theta=2;
float ki=0,ki_theta=0;
float t_sampling=0.5;




int main(int argc, char** argv)
{

	//initializing the node 
	ros::init(argc, argv, "turtle_circle");

	//declaring a nodeHandle for running the node 
	ros::NodeHandle n;

    ros::Subscriber turtle_sub=n.subscribe("/turtle1/pose",100,turtleX_pose);
        
        //defining a publisher
        ros::Publisher turtle_pub=n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);
        ros::Rate loop_rate(10);
        
        //defining a publisher to publish the pose of the turtle
        ros::Publisher turtle_pub_pose=n.advertise<turtlesim::Pose>("/rt_real_pose",100);

        //defining a publisher to publish the pose of the turtle mixed with random gaussian noise
        ros::Publisher turtle_pub_noisy_pose=n.advertise<turtlesim::Pose>("rt_noisy_pose",100);

	//defining the structure to read the user desired position and orientation of the turtle
	destination_position Pose;
    
        geometry_msgs::Twist velocity;

	//taking user input for the desired location of the turtle
	cout<<"Enter the position, where you want the turtle to move from the present position"<<endl;
	cout<<"Enter the x_coordinate:";
	cin>>Pose.x;
	cout<<"Enter the y_coordinate:";
	cin>>Pose.y;
	
	Pose.angle=0.0;

	//defining the corresponding data type for error calculation
	float err_pose_x,err_pose_y,err_pose_z;
	float err_vel_x,err_vel_y,err_vel_z;
	float err_theta,err_angular_vel;

	float current_acc_x,current_acc_y,current_acc_angle;
	float commanded_acc_x,commanded_acc_y,commanded_acc_angle;
	float pre_vel_x,pre_vel_y,pre_vel_angular;
        float time_taken=0;
        float radius,speed,t,omega;

        pre_vel_x=0;
	pre_vel_y=0;
	pre_vel_angular=0;
        
        

       
while(ros::ok())
{
       
	ros::spinOnce();
               


		//finding the error in the position
		err_pose_x=Pose.x-(current_position.x);
		err_pose_y=Pose.y-(current_position.y);
		

		// finding the error in velocity
		err_vel_x=0-(current_velocity.x);
		err_vel_y=0-(current_velocity.y);
		

		//finding the error in orientation
		err_theta=Pose.angle-(current_position.theta);
		err_angular_vel=0 -(current_velocity.angular);

		//determining the commanded acc value using the PID law
		commanded_acc_x= ((current_velocity.x-pre_vel_x)/t_sampling)+ (kp*err_pose_x) + (kd*err_vel_x);
		commanded_acc_y= ((current_velocity.y-pre_vel_y)/t_sampling) + (kp*err_pose_y) + (kd*err_vel_y);
		commanded_acc_angle= ((current_velocity.angular-pre_vel_angular)/t_sampling) + (kp_theta*err_theta) + (kd_theta*err_angular_vel);
                
  if (abs(err_pose_x)>0.05&&abs(err_pose_y)>0.05)
  {
		velocity.linear.x=commanded_acc_x*t_sampling;
		velocity.linear.y=commanded_acc_y*t_sampling;

		velocity.angular.x=0;
		velocity.angular.y=0;
		velocity.angular.z=commanded_acc_angle*t_sampling;
     if(abs(err_theta)<0.02)
       {velocity.angular.z=0;}
  }
  else if(abs(err_pose_x)<0.05)
 {              velocity.linear.x= 0; 
		velocity.linear.y=commanded_acc_y*t_sampling;

		velocity.angular.x=0;
		velocity.angular.y=0;
		velocity.angular.z=commanded_acc_angle*t_sampling;
    if(abs(err_theta)<0.02)
       {velocity.angular.z=0;}
    if(abs(err_pose_y)<0.05)
     {          velocity.linear.x=0; 
		velocity.linear.y=0; 

		velocity.angular.x=0;
		velocity.angular.y=0;
		velocity.angular.z=0; 
                cout<<"reached at the start position of the circle"<<endl;
      break;
     }
 
  }
  else if(abs(err_pose_y)<0.05)
 {              velocity.linear.x=commanded_acc_x*t_sampling;
		velocity.linear.y=0; 

		velocity.angular.x=0;
		velocity.angular.y=0;
		velocity.angular.z=commanded_acc_angle*t_sampling;
    if(abs(err_theta)<0.02)
       {velocity.angular.z=0;}
    if(abs(err_pose_x)<0.05)
     {          velocity.linear.x=0; 
		velocity.linear.y=0; 

		velocity.angular.x=0;
		velocity.angular.y=0;
		velocity.angular.z=0; 
                cout<<"reached at the start position of the circle"<<endl;
      break;
     }
 
  }
                             
	        
	        
                ROS_INFO("%f,%f,%f,%f",velocity.linear.x,velocity.linear.y,velocity.linear.z,current_position.x);
		ROS_INFO("%f,%f,%f,%f",velocity.angular.x,velocity.angular.y,velocity.angular.z,time_taken);
		turtle_pub.publish(velocity);
		loop_rate.sleep();
                time_taken+=0.1;
  
}
time_taken=0;

cout<<"what should be the radius of the circle in which you want the turtle to move around:";
cin>>radius;
cout<<"what speed you would like the turtle to move with in the circular path:";
cin>>speed;
t=0;
omega=speed/radius;

//creating a gaussian noise to add it to the pose data that we get from the the topic turtlesim/Pose

double mean=0.5;
double std_deviation=10.0;
default_random_engine gaussian_noise;
normal_distribution<double> distribution(mean,std_deviation);
 

//here we introduce a loop to draw a circle at the desired location
 while(ros::ok())
    {ros::spinOnce();
        t+=0.11;

        velocity.linear.x=radius*omega*(cos(omega*t));
        velocity.linear.y=-radius*omega*(sin(omega*t));
        velocity.linear.z=0;

        velocity.angular.x=0;
        velocity.angular.y=0;
        velocity.angular.z=0;

        //adding the gaussian noise with the normal pose data 
        noisy_pose.x=noisy_pose.x + distribution(gaussian_noise);
        noisy_pose.y=noisy_pose.y + distribution(gaussian_noise);
        noisy_pose.theta=noisy_pose.theta + distribution(gaussian_noise);
        noisy_pose.linear_velocity=noisy_pose.linear_velocity + distribution(gaussian_noise);
        noisy_pose.angular_velocity=noisy_pose.angular_velocity + distribution(gaussian_noise);
        

      ROS_INFO("%f,%f,%f,%f,%f",velocity.linear.x,velocity.linear.y,velocity.linear.z,velocity.angular.z,t);
       turtle_pub.publish(velocity);
       if(int(t)%5==0)
        {
         turtle_pub_pose.publish(Position);
         turtle_pub_noisy_pose.publish(noisy_pose);
        }
       loop_rate.sleep();
    }

  return 0;
}




