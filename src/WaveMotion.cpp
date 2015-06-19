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
	ros::Rate loop_rate(10);
	double positions[7];
	std_msgs::String names[7];
	
	if(strcmp(argv[1], "left") == 0)
        {
                armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);
		
		names[0].data = "left_w1";
	}
        else if(strcmp(argv[1], "right") == 0)
        {
                armPose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);

		names[0].data = "right_w1";
	}
	else
	{
		ROS_ERROR("Invalid limb. Try 'left' or 'right'.\n");
                exit(1);
        }

	positions[0] = -1;

	baxter_core_msgs::JointCommand waveHandPose;
	for(int i = 0; i < 6; i++)
        {
                waveHandPose.names.push_back(names[i].data);
                waveHandPose.command.push_back(positions[i]);
        }
        waveHandPose.mode = 1;

        while(ros::ok())
        {
                armPose_pub.publish(waveHandPose);
                ros::spinOnce();
                loop_rate.sleep();
        }


	return 0;
}
