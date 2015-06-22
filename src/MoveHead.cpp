#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/HeadPanCommand.h"
#include "baxter_core_msgs/HeadState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstring>

double left_s0, right_s0, head;

void callbackHead(baxter_core_msgs::HeadState msg1)
{
	head = msg1.pan;
	ROS_INFO("head: %f\n", head);
}

void callbackArm(sensor_msgs::JointState msg2)
{
	int right_s0Index = -999, left_s0Index = -999;
	
	for(int i = 0; i < msg2.name.size(); i++)
	{
		if(strcmp(msg2.name[i].c_str(), "left_s0") == 0)
		{
			left_s0Index = i;
			break;
		}
	}

	if(left_s0Index == -999)
	{
		ROS_ERROR("left_s0 not found.\n");
		exit(1);
	}

	for(int i = 0; i < msg2.name.size(); i++)
	{
		if(strcmp(msg2.name[i].c_str(), "right_s0") == 0)
		{
			right_s0Index = i;
			break;
		}
	}
	
	if(right_s0Index == -999)
	{
		ROS_ERROR("right_s0 not found.\n");
		exit(1);
	}
	
	left_s0 = msg2.position[left_s0Index];
	right_s0 = msg2.position[right_s0Index];
	ROS_INFO("left_s0: %f \tright_s0:%f\n", left_s0, right_s0);
}

double getArmPos(double headPos)
{
	double armPos;
	
	if(headPos >= 0)
		armPos = headPos - 0.8;
	else
		armPos = headPos + 0.8;

	return armPos;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "head_arm_sync");

	if(argc != 2)
	{
		ROS_ERROR("Incorrect usage. Try 'move_head <head_position>'.\n");
		exit(1);
	}
	
	ros::NodeHandle n;
	ros::Publisher headPose_pub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan", 1000);
	ros::Publisher armPoseLeft_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);
	ros::Publisher armPoseRight_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
	ros::Subscriber headState = n.subscribe<baxter_core_msgs::HeadState>("/robot/head/head_state", 1000, callbackHead);
	ros::Subscriber armState = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, callbackArm);
	ros::Rate loop_rate(10);

	std_msgs::String leftName, rightName;
	leftName.data = "left_s0";
	rightName.data = "right_s0";
	double headCommand, armCommand;
	int state = 0;
	baxter_core_msgs::HeadPanCommand moveHead;
	baxter_core_msgs::JointCommand moveArm;

	headCommand = atof(argv[1]);

	//Set a position for the head to move
	moveHead.target = headCommand;
	moveHead.speed = 10;
	ROS_INFO("moveHead.target = %f\n", moveHead.target);

	moveArm.mode = 1;
        moveArm.names.push_back(rightName.data);
        moveArm.command.push_back(0);

	while(ros::ok())
	{
		if(state == 0)
		{
			if(head <= (moveHead.target - 0.1) || head >= (moveHead.target + 0.1))
				headPose_pub.publish(moveHead);
			else
				state = 1;
		}
		else if(state == 1)
		{
			armCommand = getArmPos(head);
			moveArm.command[0] = armCommand;
			if(head >= 0)
			{
				if(right_s0 <= armCommand - 0.1 || right_s0 >= armCommand + 0.1)
				{
					moveArm.names[0] = rightName.data;
					armPoseRight_pub.publish(moveArm);
				}
				else
					state = 2;
			}
			else
			{
				if(left_s0 <= armCommand - 0.1 || left_s0 >= armCommand + 0.1)
				{
					moveArm.names[0] = leftName.data;
                                        armPoseLeft_pub.publish(moveArm);
				}
				else
					state = 2;
			}
		}
		else
			break;
		
		moveArm.command[0] = armCommand;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

