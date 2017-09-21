#include <ros/ros.h>
#include <stdio.h>

#include "ros_falcon/falconSetPoint.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"

bool falcon_atpos, gripper_atpos;
ros_falcon::falconSetPoint SetPoint;

//Servo angles
const int Open = 100;
const int CloseBall = 60;
const int ClosePencil = 43;

//Function prototypes
void falcon_moving(const std_msgs::Bool::ConstPtr& falconmove);
void gripper_moving(const std_msgs::Bool::ConstPtr& grippermove);
ros_falcon::falconSetPoint moveto(double x, double y, double z);
int gripper_move(int angle);


int main(int argc, char* argv[])
{

    //ROS Initialisation
    ros::init(argc,argv, "RobotControl");
    ros::NodeHandle node;

	ros::Subscriber falcon_moving_sub = node.subscribe("falcon_atpos", 1, falcon_moving);
    ros::Subscriber gripper_moving_sub = node.subscribe("gripper_atpos", 1, gripper_moving);

    ros::Publisher setpoint_pub = node.advertise<ros_falcon::falconSetPoint>("falcon_setpoint", 1);
    ros::Publisher gripper_pub = node.advertise<std_msgs::UInt8>("servo_angle",1);

    ROS_INFO("Starting Robot Control node");

    //Falcon control code
    //setpoint_pub.publish(moveto(x,y,z)); to move Falcon. X,Y,Z are desired coordinates for end effector in metres
    //gripper_pub.publish(gripper_move(angle)); to move Gripper. Angle int between 43 and 77, or Open/CloseBall/ClosePencil/////////////////////////////////////////////////////////////////////

//////////////////Enter actions here//////////////////////////////////

	bool init = false;


	while(node.ok())
	{	
        std_msgs::UInt8 msg;

		if(!init)
		{

				setpoint_pub.publish(moveto(0, 0, 0.075));
                msg.data = gripper_move(Open);
                gripper_pub.publish(msg);


				ROS_INFO("Press Enter to commence");

				getchar();

				init = true;
		}

		else
		{

			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.11));	//1 (above)

			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.157));	//1
            msg.data = gripper_move(CloseBall);
            gripper_pub.publish(msg);

			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.11));	//1 (above)

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.12));	//2 (above)

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.16));	//2
            msg.data = gripper_move(Open);
            gripper_pub.publish(msg);

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.12));	//2 (above)

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.12));	//3 (above)

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.164));	//3
            msg.data = gripper_move(CloseBall);
            gripper_pub.publish(msg);

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.12));	//3 (above)

			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.11));	//1 (above)

			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.157));	//1

            msg.data = gripper_move(Open);
            gripper_pub.publish(msg);
		
			setpoint_pub.publish(moveto(-0.052823, 0.017132, 0.11));	//1 (above)

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.13));	//2 (above)

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.16));	//2
            msg.data = gripper_move(CloseBall);
            gripper_pub.publish(msg);

			setpoint_pub.publish(moveto(-0.008, -0.028478, 0.13));	//2 (above)

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.11));	//3 (above)

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.164));	//3
            msg.data = gripper_move(Open);
            gripper_pub.publish(msg);

			setpoint_pub.publish(moveto(0.0295, 0.032632, 0.11));	//3 (above)
		
		}

	}
/////////////////////////////////////////////////////////////////////

    ROS_INFO("Closing Robot Control node");

	return 0;
}


void falcon_moving(const std_msgs::Bool::ConstPtr& falconmove)
{
	falcon_atpos = falconmove->data;

}

void gripper_moving(const std_msgs::Bool::ConstPtr& grippermove)
{
    gripper_atpos = grippermove->data;
}

ros_falcon::falconSetPoint moveto(double x, double y, double z)
{
    ros::NodeHandle node;
    ros::Rate loop_rate(25);

    //Wait until both falcon and gripper are stationary before proceeding	
	do
	{
        loop_rate.sleep();
        ROS_DEBUG("Waiting for Falcon");
		ros::spinOnce();	//Get transit messages
	}
    while((!falcon_atpos || !gripper_atpos) && node.ok());
	
	ROS_INFO("Moving Falcon to X: %f, Y: %f, Z: %f", x, y, z);
	
	
	SetPoint.X = x;
	SetPoint.Y = y;
	SetPoint.Z = z;
	
	return SetPoint;
}


int gripper_move(int angle)
{
    ros::NodeHandle node;
    ros::Rate loop_rate(25);

    //Wait until both falcon and gripper are stationary before proceeding	
	do
	{
        loop_rate.sleep();
        ROS_DEBUG("Waiting for gripper");
		ros::spinOnce();	//Get transit messages
	}
    while((!falcon_atpos || !gripper_atpos) && node.ok());

    //Determine message to display
    switch (angle)
    {
        case Open:
            ROS_INFO("Opening gripper");
            break;

        case CloseBall:
            ROS_INFO("Closing gripper for ball");
            break;

        case ClosePencil:
            ROS_INFO("Closing gripper for pencil");
            break;

        default:	
            ROS_INFO("Moving gripper to %d degrees",angle);
    }

    return angle;
	
	
}
