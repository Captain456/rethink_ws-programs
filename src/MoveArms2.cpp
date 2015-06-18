#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/JointCommand.h"
#include "std_msgs/Header.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <cstring>

int main(int argc, char** argv)
{
        ros::init(argc, argv, "test_pose");

        if(argc != 8)
        {
                ROS_INFO("Incorrect usage. Try 'pose (position) <x> <y> <z> (orientation) <x> <y> <z> <w>'.\n");
                exit(1);
        }

        ros::NodeHandle n;
	ros::Publisher pose_pub;
	ros::ServiceClient client;

	if(strcmp(argv[1], "left") == 0)
	{
        	pose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);
        	client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
	}
	else if(strcmp(argv[1], "right") == 0)
	{
		pose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
                client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");
	}
	else
	{
		ROS_ERROR("%s You did not enter a valid limb. Try 'left' or 'right'.\n", argv[1]);
		exit(1);
	}
        ros::Rate loop_rate(10);

        baxter_core_msgs::SolvePositionIK srv;
        double posx, posy, posz, orix, oriy, oriz, oriw, heading, attitude, bank;
        int s0, s1, e0, e1, w0, w1, w2;
       
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
				finalPose.command.push_back(srv.response.joints[0].position[i]);
                        }
                        finalPose.mode = 1;

                        while(ros::ok)
                        {
                                pose_pub.publish(finalPose);
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
