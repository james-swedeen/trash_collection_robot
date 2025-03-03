
#include <senior_project/arm_controller.hpp>
#include <cmath>
#include <stdio.h>
#include<rrt_search/edge_generators/rx200_arm_edge_generator.hpp>
//So I can use degrees instead of constantly thinking in radians
#define TO_RAD (M_PI/180)

int main(int argc, char **argv)
{
	//Eigen Matrix Call
	Eigen::Matrix<double,1,5,Eigen::RowMajor> angles;
	//Input from command line
	double input[5];

	ros::init(argc, argv, "Main");
	//handle publishing movement commands
	ros::NodeHandle n;

	//handle service call to object
	ros::NodeHandle obj;

	//publish to joint group commands
	ros::Publisher main_pub = n.advertise<interbotix_xs_sdk::JointGroupCommand>("/robot/commands/joint_group",5);
	ros::Publisher main_grip_pub = n.advertise<interbotix_xs_sdk::JointSingleCommand>("/robot/commands/joint_single",5);

	ros::Rate rate(0.25);

	interbotix_xs_sdk::JointGroupCommand message;
	interbotix_xs_sdk::JointSingleCommand grip_message;

	int x = 0;
	bool is_grip = false;

	while(ros::ok()){

      rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(-0.15, -0.15, 0.1, 0, 0, angles);
			message = mover(angles);
			main_pub.publish(message);
			ros::spinOnce();
			rate.sleep();

      rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(-0.25, -0.25, 0.05, 0, 0, angles);
			message = mover(angles);
			main_pub.publish(message);
			ros::spinOnce();
			rate.sleep();

      rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(-0.2, 0.27, 0.25, 0, 0, angles);
			message = mover(angles);
			main_pub.publish(message);
			ros::spinOnce();
			rate.sleep();

      rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(0.25, -0.125, 0.05, 0, 0, angles);
			message = mover(angles);
			main_pub.publish(message);
			ros::spinOnce();
			rate.sleep();

      rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor>::jointStates(0.25, 0.1, 0.25, 0, 0, angles);
			message = mover(angles);
			main_pub.publish(message);
			ros::spinOnce();
			rate.sleep();

/*		printf("Enter 1 to control arm, 2 to control gripper: ");
		scanf("%d",&x);
		if(x == 1){
			printf("Enter Waist: ");
			scanf("%lf",&input[0]);
			printf("Enter Shoulder: ");
			scanf("%lf",&input[1]);
			printf("Enter Elbow: ");
			scanf("%lf",&input[2]);
			printf("Enter Wrist Angle: ");
			scanf("%lf",&input[3]);
			printf("Enter Wrist Rotate: ");
			scanf("%lf",&input[4]);

			for(int i = 0; i < 5; i++){
				angles[i] = input[i] * TO_RAD;
			}*/

	/*	}else if(x==2){
			is_grip = !is_grip;
			grip_message = grip(is_grip);
			main_grip_pub.publish(grip_message);
		}else
			return 0;*/
	}

	return 0;
}
