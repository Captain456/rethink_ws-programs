#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/JointCommand.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "control_msgs/JointControllerState.h"
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "unistd.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_wave");
	
	if(argc != 2)
	{
		ROS_ERROR("Incorrect usage. Try calling 'wave <left/right>'.\n");
		exit(1);
	}

	ros::NodeHandle n;
	ros::Publisher armPose_pub;
	ros::Subscriber e0cur, e1cur, s0cur, s1cur, w0cur, w1cur, w2cur;
	ros::Rate loop_rate(10);
	double positions[7];
	std_msgs::String names[7];

	if(strcmp(argv[1], "left") == 0)
	{
		armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);

		//e0cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_e0_controller/state", 1000, callbacke0);
		//e1cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_e1_controller/state", 1000, callbacke1);
		//s0cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_s0_controller/state", 1000, callbacks0);
		//s1cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_s1_controller/state", 1000, callbacks1);
		//w0cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_w0_controller/state", 1000, callbackw0);
		//w1cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_w1_controller/state", 1000, callbackw1);
		//w2cur = n.subscribe<control_msgs::JointControllerState>("/robot/left_joint_position_controller/joints/left_w2_controller/state", 1000, callbackw2);
		names[0].data = "left_e0";
		names[1].data = "left_e1";
		names[2].data = "left_s0";
		names[3].data = "left_s1";
		names[4].data = "left_w0";
		names[5].data = "left_w1";
		names[6].data = "left_w2";
	}
	else if(strcmp(argv[1], "right") == 0)
	{
		armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);

                //e0cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_e0_controller/state", 1000, callbacke0);
                //e1cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_e1_controller/state", 1000, callbacke1);
                //s0cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_s0_controller/state", 1000, callbacks0);
                //s1cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_s1_controller/state", 1000, callbacks1);
                //w0cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_w0_controller/state", 1000, callbackw0);
                //w1cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_w1_controller/state", 1000, callbackw1);
                //w2cur = n.subscribe<control_msgs::JointControllerState>("/robot/right_joint_position_controller/joints/right_w2_controller/state", 1000, callbackw2);
		
		names[0].data = "right_e0";
		names[1].data = "right_e1";
		names[2].data = "right_s0";
		names[3].data = "right_s1";
		names[4].data = "right_w0";
		names[5].data = "right_w1";
		names[6].data = "right_w2";
	}
	else
	{
		ROS_ERROR("Invalid limb. Try 'left' or 'right'.\n");
		exit(1);
	}

	positions[0] = -3.028;
	positions[1] = (M_PI/2);
	positions[2] = 0;
	positions[3] = 0;
	positions[4] = -3.059;
        positions[5] = 0;
	positions[6] = 0;

	baxter_core_msgs::JointCommand wavePose;
	for(int i = 0; i < 6; i++)
	{
		wavePose.names.push_back(names[i].data);
		wavePose.command.push_back(positions[i]);
	}
	wavePose.mode = 1;

	while(ros::ok())
	{	
		armPose_pub.publish(wavePose);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
