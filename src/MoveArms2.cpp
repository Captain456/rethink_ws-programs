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

int main(int argc, char** argv)
{
	/*namespace gv
	{
		class Point{double x; double y; double z;};
		class Quaternion{double x; double y; double z; double w;};
		class Pose{Point point; Quaternion quaternion;};
		class PRY{double pitch; double roll; double yaw;};
		class PRYPose{Point point; PRY pry;}; 
	}*/

        ros::init(argc, argv, "test_pose");

        if(argc != 7)
        {
                ROS_INFO("Incorrect usage. Try 'pose (position) <x> <y> <z> (orientation) <x> <y> <z> <w>'.\n");
                exit(1);
        }

        ros::NodeHandle n;

        ros::Publisher pose_pub = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1000);
        ros::ServiceClient client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
        ros::Rate loop_rate(10);

        baxter_core_msgs::SolvePositionIK srv;
        double posx, posy, posz, orix, oriy, oriz, oriw, heading, attitude, bank;
        int s0, s1, e0, e1, w0, w1, w2;
       
        posx = atof(argv[1]);
        posy = atof(argv[2]);
        posz = atof(argv[3]);
        bank = atof(argv[4]);
        heading = atof(argv[5]);
        attitude = atof(argv[6]);

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
                        ROS_INFO("Here: Just before the for-loop! %d\n", srv.response.joints[0].name.size());
                        for(int i = 0; i < srv.response.joints[0].name.size(); i++)
                        {
                                ROS_INFO("Here: Inside the for-loop.\n");
                                ROS_INFO("%s\n", srv.response.joints[0].name[i].c_str());

                                switch(srv.response.joints[0].name[i][5])
                                {
                                case 's':
                                        ROS_INFO("Here: Inside 's'.\n");
                                        if(srv.response.joints[0].name[i][6] == '0')
                                        {
                                                ROS_INFO("Found 's0'.\n");
                                                s0 = i;
                                        }
                                        else
                                        {
                                                ROS_INFO("Found 's1'.\n");
                                                s1 = i;
                                        }
                                        break;
                                case 'e':
                                        ROS_INFO("Here: Inside 'e'.\n");
                                        if(srv.response.joints[0].name[i][6] == '0')
                                        {
                                                ROS_INFO("Found 'e0'.\n");
                                                e0 = i;
                                        }
                                        else
                                        {
                                                ROS_INFO("Found 'e1'.\n");
                                                e1 = i;
                                        }
                                        break;
                                case 'w':
                                        ROS_INFO("Here: Inside 'w'.\n");
                                        if(srv.response.joints[0].name[i][6] == '0')
                                        {
                                                ROS_INFO("Found 'w0'.\n");
                                                w0 = i;
                                        }
                                        else if(srv.response.joints[0].name[i][6] == '1')
                                        {
                                                ROS_INFO("Found 'w1'.\n");
                                                w1 = i;
                                        }
                                        else
                                        {
                                                ROS_INFO("Found 'w2'.\n");
                                                w2 = i;
                                        }
                                        break;
                                default:
                                        ROS_INFO("Here: Inside 'default'.\n");
                                        break;
                                }
                        }

                        ROS_INFO("Here: Back outside of the for-loop.\n");

                        baxter_core_msgs::JointCommand finalPose;
                        finalPose.mode = 1;
                        finalPose.command.push_back(srv.response.joints[0].position[w0]);
                        finalPose.names.push_back(srv.response.joints[0].name[w0]);
                        finalPose.command.push_back(srv.response.joints[0].position[w1]);
                        finalPose.names.push_back(srv.response.joints[0].name[w1]);
                        finalPose.command.push_back(srv.response.joints[0].position[w2]);
                        finalPose.names.push_back(srv.response.joints[0].name[w2]);
                        finalPose.command.push_back(srv.response.joints[0].position[e0]);
                        finalPose.names.push_back(srv.response.joints[0].name[e0]);
                        finalPose.command.push_back(srv.response.joints[0].position[e1]);
                        finalPose.names.push_back(srv.response.joints[0].name[e1]);
                        finalPose.command.push_back(srv.response.joints[0].position[s0]);
                        finalPose.names.push_back(srv.response.joints[0].name[s0]);
                        finalPose.command.push_back(srv.response.joints[0].position[s1]);
                        finalPose.names.push_back(srv.response.joints[0].name[s1]);

                        ROS_INFO("Here: Past all of the finalPose stuff.\n");

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
