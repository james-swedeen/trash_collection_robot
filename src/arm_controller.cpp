
#include<trash_collection_robot/arm_controller.hpp>

interbotix_xs_msgs::JointGroupCommand mover(const Eigen::Matrix<double, 5, 1>& angles)
{
	interbotix_xs_msgs::JointGroupCommand message;

	message.name = "arm";
	message.cmd.resize(5);
	message.cmd[0] = angles[0];	//Waist
	message.cmd[1] = angles[1];	//Shoulder
	message.cmd[2] = angles[2];	//Elbow
	message.cmd[3] = angles[3];	//Wrist Angle
	message.cmd[4] = angles[4];	//Wrist_Rotate

	return message;
}


interbotix_xs_msgs::JointSingleCommand grip(bool open_it){

	interbotix_xs_msgs::JointSingleCommand message;
	message.name = "gripper";

	if(open_it)
		message.cmd = 250;
	else
		message.cmd = -300;


	return message;

}
