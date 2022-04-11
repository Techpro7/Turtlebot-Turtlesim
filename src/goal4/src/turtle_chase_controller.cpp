#include"ros/ros.h"
#include"geometry_msgs/Twist.h"
#include"turtlesim/Pose.h"
#include<iostream>
#include<cmath>
#include<string>


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

turtlesim::Pose des_pose2;//it will read the position of the turtle 1 to take it as an input of destination position of turtle 2

turtlesim::Pose curr_pose2;


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

       	//reading the current velocity data of the turtle as an input from the turtlesim/Pose.msg
	current_velocity.x=(pose->linear_velocity)*(cos(pose->theta));
	current_velocity.y=(pose->linear_velocity)*(sin(pose->theta));
	current_velocity.z=0;
	current_velocity.angular=pose->angular_velocity;

       //reading the current position of the turtle1 as the destination position of turtle2
        des_pose2.x=pose->x;
        des_pose2.y=pose->y;
        des_pose2.theta=pose->theta;
        des_pose2.linear_velocity=pose->linear_velocity;
        des_pose2.angular_velocity=pose->angular_velocity;

	ROS_INFO("%f,%f,%f",pose->x,pose->y,pose->theta);
	
 
	
}

void turtle2_pose(const turtlesim::PoseConstPtr &pose)
{
      //here we taking the input of the current position of the turtle 2
      curr_pose2.x=pose->x;
      curr_pose2.y=pose->y;
      curr_pose2.theta=pose->theta;
      curr_pose2.linear_velocity=pose->linear_velocity;
      curr_pose2.angular_velocity=pose->angular_velocity;
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
        
        //here we are subscribing to get the pose of the second turtle
        ros::Subscriber turtle2_sub=n.subscribe("/turtle2/pose",100,turtle2_pose);

        
        //defining a publisher
        ros::Publisher turtle_pub=n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);
        ros::Rate loop_rate(10);
        
        //defining a publisher to publish the pose of the turtle
        ros::Publisher turtle_pub_pose=n.advertise<turtlesim::Pose>("/rt_real_pose",100);

        //defining  a publisher to publish the pose in order to spawn turtle2 in the simulator
        ros::Publisher turtle2_pub=n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",100);

      	//defining the structure to read the user desired position and orientation of the turtle
	destination_position Pose;
    
        geometry_msgs::Twist velocity;
        geometry_msgs::Twist velocity2;

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

float err_posex2,err_posey2,err_pose_theta2;
float err_velx2,err_vely2,err_angular2;
float kp_2=1.5;
float kd_2=2.0;
float kp_2_theta=1.5;
float kd_2_theta=2.0;
float comm_acc_x2, comm_acc_y2, comm_acc_angular2;



//here we introduce a loop to draw a circle at the desired location
 while(ros::ok())
    {ros::spinOnce();
        t+=0.1;

        //deriving the error in position of the turtle2
        err_posex2=des_pose2.x-curr_pose2.x;
        err_posey2=des_pose2.y-curr_pose2.y;

        //deriving the error in velocity of the turtle2
        err_velx2=((des_pose2.linear_velocity*cos(des_pose2.theta)) -(curr_pose2.linear_velocity*cos(curr_pose2.theta)));
        err_vely2=((des_pose2.linear_velocity*sin(des_pose2.theta)) -(curr_pose2.linear_velocity*sin(curr_pose2.theta)));

        //deriving the error in the angular velocity of the turtle 2
        err_angular2=des_pose2.angular_velocity - curr_pose2.angular_velocity;

        //applying the PID law to implement the controller
        comm_acc_x2= ((curr_pose2.linear_velocity*cos(curr_pose2.theta))/t_sampling) + (kp_2*err_posex2) + (kd_2*err_velx2);
        comm_acc_y2= ((curr_pose2.linear_velocity*sin(curr_pose2.theta))/t_sampling) + (kp_2*err_posey2) + (kd_2*err_vely2);
        comm_acc_angular2= (curr_pose2.angular_velocity/t_sampling) + (kp_2_theta*err_pose_theta2) + (kd_2_theta*err_angular2);
        
  if (abs(err_posex2)>1.5&&abs(err_posey2)>1.5)
  {
		velocity2.linear.x=comm_acc_x2*t_sampling;
		velocity2.linear.y=comm_acc_y2*t_sampling;

		velocity2.angular.x=0;
		velocity2.angular.y=0;
		velocity2.angular.z=comm_acc_angular2*t_sampling;
     if(abs(err_pose_theta2)<0.02)
       {velocity2.angular.z=0;}
  }
  else if(abs(err_posex2)<1.5)
 {              velocity2.linear.x= 0; 
		velocity2.linear.y=comm_acc_y2*t_sampling;

		velocity2.angular.x=0;
		velocity2.angular.y=0;
		velocity2.angular.z=comm_acc_angular2*t_sampling;
    if(abs(err_pose_theta2)<0.02)
       {velocity2.angular.z=0;}
    if(abs(err_posey2)<1.5)
     {          velocity2.linear.x=0; 
		velocity2.linear.y=0; 

		velocity2.angular.x=0;
		velocity2.angular.y=0;
		velocity2.angular.z=0; 
                cout<<"chase complete"<<endl;
      break;
     }
 
  }
  else if(abs(err_posey2)<1.5)
 {              velocity2.linear.x=comm_acc_x2*t_sampling;
		velocity2.linear.y=0; 

		velocity2.angular.x=0;
		velocity2.angular.y=0;
		velocity2.angular.z=comm_acc_angular2*t_sampling;
    if(abs(err_pose_theta2)<0.02)
       {velocity2.angular.z=0;}
    if(abs(err_posex2)<1.5)
     {          velocity2.linear.x=0; 
		velocity2.linear.y=0; 

		velocity2.angular.x=0;
		velocity2.angular.y=0;
		velocity2.angular.z=0; 
                cout<<"chase complete"<<endl;
      break;
     }
 }
 


        velocity.linear.x=radius*omega*(cos(omega*t));
        velocity.linear.y=-radius*omega*(sin(omega*t));
        velocity.linear.z=0;

        velocity.angular.x=0;
        velocity.angular.y=0;
        velocity.angular.z=0;    

      ROS_INFO("%f,%f,%f,%f,%f",velocity.linear.x,velocity.linear.y,velocity.linear.z,velocity.angular.z,t);
       turtle_pub.publish(velocity);
       if(int(t)%5==0)
        {
         turtle_pub_pose.publish(Position);
         }
        if(t>=5.0)
        {
         turtle2_pub.publish(velocity2);
        }
       loop_rate.sleep();
    }

  return 0;
}


