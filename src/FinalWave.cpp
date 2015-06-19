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

double e0, e1, s0, s1, w0, w1, w2;
int index_space;

void callback(sensor_msgs::JointState msg)
{
	/*int left_e0Index = -999;
	
	for(int i = 0; i < msg.name.size(); i++)
	{
		if(strcmp(msg.name[i].c_str(), "left_e0") == 0)
		{
			left_e0Index = i;
			break;
		}
	}
	if(left_e0Index == -999)
	{
		ROS_ERROR("left_e0 not found.\n");
		exit(1);
	}
	e0 = msg.position[left_e0Index + index_space];
	e1 = msg.position[left_e0Index + index_space + 1];
	s0 = msg.position[left_e0Index + index_space + 2];
	s1 = msg.position[left_e0Index + index_space + 3];
	w0 = msg.position[left_e0Index + index_space + 4];
	w1 = msg.position[left_e0Index + index_space + 5];
	w2 = msg.position[left_e0Index + index_space + 6];*/
	e0 = msg.position[2 + index_space];
        e1 = msg.position[3 + index_space];
        s0 = msg.position[4 + index_space];
        s1 = msg.position[5 + index_space];
        w0 = msg.position[6 + index_space];
        w1 = msg.position[7 + index_space];
        w2 = msg.position[8 + index_space];
	ROS_INFO("%f, %f, %f, %f, %f, %f, %f\n", e0, e1, s0, s1, w0, w1, w2);
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
		index_space = 0;

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
		index_space = 7;		

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

	//Move the arm into a waving position
	positions[0] = -3.028;
        positions[1] = (M_PI/2);
        positions[2] = 0;
        positions[3] = 0;
        positions[4] = -3.059;
        positions[5] = 0;
        positions[6] = (M_PI/2);

        baxter_core_msgs::JointCommand wavePose, waveMove1, waveMove2;
        for(int i = 0; i < 7; i++)
        {
                wavePose.names.push_back(names[i].data);
                wavePose.command.push_back(positions[i]);
        }
        wavePose.mode = 1; //Set it in position mode

	//Move the hand in positive-radian direction
	positions[5] = 3;
	waveMove1.mode = 2; //Set it in velocity mode
	waveMove1.names.push_back(names[5].data);
	waveMove1.command.push_back(positions[5]);
	
	//Move the hand in negative-radian direction
	positions[5] = -3;
	waveMove2.mode = 2; //Set it in velocity mode
	waveMove2.names.push_back(names[5].data);
	waveMove2.command.push_back(positions[5]);

	while(ros::ok())
	{
		if(waveState == 0)
		{
			if(e0 >= -3.015 || e1 <= ((M_PI/2) - 0.1) || e1 >= ((M_PI/2) + 0.1) || s0 <= -0.1 || s0 >= 0.1 || s1 <= -0.1 || s1 >= 0.1 || w0 >= -3.045 || w1 <= -0.1 || w1 >= 0.1 || w2 <= ((M_PI/2) - 0.1) || w2 >= ((M_PI/2) + 0.1))
			{
				armPose_pub.publish(wavePose);
			}
			else
			{
				waveState = 1;
			}
		}
		else if(waveState == 1)
		{
			if(w1 <= 0.5)
			{
				armPose_pub.publish(waveMove1);
			}
			else
			{
				waveState = 2;
			}
		}
		else
		{
			if(w1 >= -0.5)
			{
				armPose_pub.publish(waveMove2);
			}
			else
			{
				waveState = 1;
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
