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

int jointIndex;
double jointPos;

void callback(sensor_msgs::JointState msg)
{
	jointPos = msg.position[jointIndex];
	ROS_INFO("%f\n", jointPos);
	return;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_wave_motion");

	if(argc != 2)
	{
		ROS_ERROR("Incorrect usage. Try calling 'wave_motion <left/right>'.\n");
                exit(1);
        }

	ros::NodeHandle n;
	ros::Publisher armPose_pub;
	ros::Subscriber state = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, callback);
	ros::Rate loop_rate(10);
	double positions[7];
	std_msgs::String names[7];
	int waveState = 0;
	
	if(strcmp(argv[1], "left") == 0)
        {
                armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);

		names[0].data = "left_w1";
		jointIndex = 7;
	}
        else if(strcmp(argv[1], "right") == 0)
        {
                armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
		names[0].data = "right_w1";
		jointIndex = 14;

	}
	else
	{
		ROS_ERROR("Invalid limb. Try 'left' or 'right'.\n");
                exit(1);
        }

	positions[0] = 1;

	baxter_core_msgs::JointCommand waveHandPose, waveHandPose2;
	for(int i = 0; i < 6; i++)
        {
                waveHandPose.names.push_back(names[i].data);
                waveHandPose.command.push_back(positions[i]);
        }
        waveHandPose.mode = 1;

	positions[0] = -1;
	for(int i = 0; i < 6; i++)
        {
                waveHandPose2.names.push_back(names[i].data);
                waveHandPose2.command.push_back(positions[i]);
        }
        waveHandPose2.mode = 1;

        while(ros::ok())
        {
		if(waveState == 0)
		{
			if(jointPos <= 0.95)
			{
				ROS_INFO("Positive!\n");
				armPose_pub.publish(waveHandPose);
			}
			else
			{
				ROS_INFO("Changed state to 1.\n");
				waveState = 1;
			}
		}
		else
		{
			if(jointPos >= -0.95)
			{
				ROS_INFO("Negative!\n");
				armPose_pub.publish(waveHandPose2);
			}
			else
			{
				ROS_INFO("Changed state to 0.\n");
				waveState = 0;
			}
		}
			
                ros::spinOnce();
		loop_rate.sleep();
        }


	return 0;
}
