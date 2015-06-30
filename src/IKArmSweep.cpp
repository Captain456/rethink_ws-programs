#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/JointCommand.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstring>

double e0, e1, s0, s1, w0, w1, w2;
int index_space;

void callback(sensor_msgs::JointState msg)
{
        int left_e0Index = -999;

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
        w2 = msg.position[left_e0Index + index_space + 6];
        //ROS_INFO("%f, %f, %f, %f, %f, %f, %f\n", e0, e1, s0, s1, w0, w1, w2);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sweeper");

	if(argc != 8)
	{
                ROS_INFO("Incorrect usage. Try 'move_hand2 <left/right> <x> <y> <z> <roll> <pitch> <yaw>'.\n");
                exit(1);
        }

	ros::NodeHandle n;
        ros::Publisher pose_pub;
	ros::Subscriber state = n.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1000, callback);
        ros::ServiceClient client;
	ros::Rate loop_rate(10);
	double positions[7];
        std_msgs::String names[7];
        int waveState = 0;

	if(strcmp(argv[1], "left") == 0)
        {
                pose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);
                client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
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
                pose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
                client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
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
                ROS_ERROR("%s You did not enter a valid limb. Try 'left' or 'right'.\n", argv[1]);
                exit(1);
        }

	baxter_core_msgs::SolvePositionIK srv;
        double posx, posy, posz, orix, oriy, oriz, oriw, heading, attitude, bank;

        posx = atof(argv[2]);
        posy = atof(argv[3]);
        posz = atof(argv[4]);
        bank = atof(argv[5]);
        heading = atof(argv[6]);
        attitude = atof(argv[7]);

        double mathc1 = cos(heading);
        double maths1 = sin(heading);
        double mathc2 = cos(attitude);
        double maths2 = sin(attitude);
        double mathc3 = cos(bank);
        double maths3 = sin(bank);
	oriw = sqrt(1.0 + mathc1 * mathc2 + mathc1*mathc3 - maths1 * maths2 * maths3 + mathc2*mathc3) / 2.0;
        double oriw4 = (4.0 * oriw);
        orix = (mathc2 * maths3 + mathc1 * maths3 + maths1 * maths2 * mathc3) / oriw4;
        oriy = (maths1 * mathc2 + maths1 * mathc3 + mathc1 * maths2 * maths3) / oriw4;
        oriz = (-maths1 * maths3 + mathc1 * maths2 * mathc3 +maths2) / oriw4;

        ROS_INFO("%f, %f, %f, %f, %f, %f, %f\n", posx, posy, posz, orix, oriy, oriz, oriw);

	geometry_msgs::PoseStamped myPose;

        myPose.header.stamp = ros::Time::now();
        myPose.header.frame_id = "base";
        myPose.pose.position.x = posx;
        myPose.pose.position.y = posy;
        myPose.pose.position.z = posz;
        myPose.pose.orientation.x = orix;
        myPose.pose.orientation.y = oriy;
        myPose.pose.orientation.z = oriz;
        myPose.pose.orientation.w = oriw;

	srv.request.pose_stamp.push_back(myPose);
        if(client.call(srv))
        {
                ROS_INFO("%d", srv.response.isValid[0]);
                if(srv.response.isValid[0])
                {
                        baxter_core_msgs::JointCommand finalPose;
                        for(int i = 0; i < srv.response.joints[0].name.size(); i++)
                        {
                                finalPose.names.push_back(srv.response.joints[0].name[i]);
				ROS_INFO("Name: %s\n", srv.response.joints[0].name[i].c_str());
                                finalPose.command.push_back(srv.response.joints[0].position[i]);
                        }
                        finalPose.mode = 1;

                        while(ros::ok)
                        {	
				if(e0 <= finalPose.command[2] - 0.1 || e0 >= finalPose.command[2] + 0.1 || e1 <= finalPose.command[3] - 0.1 || e1 >= finalPose.command[3] + 0.1 || s0 <= finalPose.command[0] - 0.1 || s0 >= finalPose.command[0] + 0.1 || s1 <= finalPose.command[1] - 0.1 || s1 >= finalPose.command[1] + 0.1 || w0 <= finalPose.command[4] - 0.1 || w0 >= finalPose.command[4] + 0.1 || w1 <= finalPose.command[5] - 0.1 || w1 >= finalPose.command[5] + 0.1 || w2 <= finalPose.command[6] - 0.1 || w2 >= finalPose.command[6] + 0.1)
				{
                                	pose_pub.publish(finalPose);
				}
				else
					break;
                                ros::spinOnce();
                                loop_rate.sleep();
                        }
                }
                else
                {
                        ROS_ERROR("Not a valid position.\n");
                        exit(1);
                }
        }
        else
        {
                ROS_ERROR("Failed to call service 'pose'.\n");
                exit(1);
        }

        return 0;
}
