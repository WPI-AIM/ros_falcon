#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <math.h>

#include "ros_falcon/falconSetPoint.h"
#include "std_msgs/Bool.h"

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"

using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;

FalconDevice m_falconDevice; 
FalconKinematicStamper falcon_legs;

std::array<double, 3> SetPoint, Pos, Error, prevError, dError, integral, forces;
boost::array<float, 3> KpGain, KiGain, KdGain;

const double ErrorThreshold = 0.005;	//Maximum error before falcon is considered at position
const double dErrorThreshold = 0.001;	//Maximum derivative of error before falcon considered at position

/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/
 
bool init_falcon(int NoFalcon) 

{
    ROS_INFO("Setting up LibUSB");
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
    //m_falconDevice.setFalconGrip<FalconGripFourButton>(); //Set Grip

    if(!m_falconDevice.open(NoFalcon)) //Open falcon @ index
    {
        ROS_ERROR("Failed to find Falcon");
        return false;
    }
    else
    {
        ROS_INFO("Falcon Found");
    }

    //There's only one kind of firmware right now, so automatically set that.
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>();
    //Next load the firmware to the device

    bool skip_checksum = false;
    //See if we have firmware
    bool firmware_loaded = false;
    firmware_loaded = m_falconDevice.isFirmwareLoaded();
    if(!firmware_loaded)
    {
        ROS_INFO("Loading firmware");
        uint8_t* firmware_block;
        long firmware_size;
        {

            firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
            firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


            for(int i = 0; i < 50; ++i)	//Attempt to load firmware 50 times
            {
                if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

                {
                    ROS_ERROR("Firmware loading try failed");
                }
                else
                {
                    firmware_loaded = true;
                    break;
                }
            }
        }
    }
    else if(!firmware_loaded)
    {
        ROS_ERROR("No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue");
        return false;
    }
    if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
    {
        ROS_ERROR("No firmware loaded to device, cannot continue");
        return false;
    }
    ROS_INFO("Firmware loaded");

    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    ROS_INFO("Homing Set");
    std::array<int, 3> forces;
    //m_falconDevice.getFalconFirmware()->setForces(forces);
    m_falconDevice.runIOLoop(); //read in data

    bool stop = false;
    bool homing = false;
    bool homing_reset = false;
    usleep(100000);

    while(!stop)
    {
        if(!m_falconDevice.runIOLoop()) continue;
        if(!m_falconDevice.getFalconFirmware()->isHomed())
        {
            if(!homing)
            {
                m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
                ROS_INFO("Falcon not currently homed. Move control all the way out then push straight all the way in.");

            }
            homing = true;
        }

        if(homing && m_falconDevice.getFalconFirmware()->isHomed())
        {
            m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
            ROS_INFO("Falcon homed.");
            homing_reset = true;
            stop = true;
        }
    }

    m_falconDevice.runIOLoop();
    return true;
}


//Read falcon set point (requested coordinates)
void get_setpoint(const ros_falcon::falconSetPoint::ConstPtr& point)
{

    std::array<double, 3> coords, LegAngles;

	//Get requested coordinates
	coords[0] = point->X;
	coords[1] = point->Y;
	coords[2] = point->Z;

    //Convert cartesian coordinates to leg angles to determine validity
	falcon_legs.getAngles(coords, LegAngles);
	for(int i = 0; i < 3; i++)
    {
        if(std::isnan(LegAngles[i]))
        {
           ROS_ERROR("Requested coordinates not in Falcon workspace");
           return;
        }
    }


   for(int i = 0; i < 3; i++)
    {

    	//Store falcon setpoint
    	SetPoint[i] = coords[i];

    }
 
    ROS_INFO("New Setpoint. %f, %f, %f", SetPoint[0], SetPoint[1], SetPoint[2]);



}

//Move to setpoint with PID control.
bool runPID()
{
			/////////////////////////////////////////////
		    //Request the current positon:		
		    Pos = m_falconDevice.getPosition();


            ROS_DEBUG("Position= %f %f %f",Pos[0], Pos[1], Pos[2]);
			
						
			//DETERMINE CURRENT ERROR, DELTA ERROR AND INTEGRAL
			for(int i = 0; i < 3; i++)
			{
				Error[i] = SetPoint[i] - Pos[i];
				dError[i] = Error[i] - prevError[i];
				integral[i] += Error[i];
				
				//Integral anti-windup
                if(integral[i] > 200)
                    integral[i] = 200;

                if(integral[i] < -200)
                    integral[i] = -200;
			
			}
            ROS_DEBUG("X Error= %f Y Error = %f Z Error = %f",Error[0], Error[1], Error[2]);
            ROS_DEBUG("X dError= %f Y dError = %f Z dError = %f",dError[0], dError[1], dError[2]);
            ROS_DEBUG("X Integral= %f Y Integral = %f Z Integral = %f",integral[0], integral[1], integral[2]);
			
            //Simple PID controller. Repeated for each axis.
			for(int i = 0; i < 3; i++)
			{
				forces[i]= KpGain[i]*Error[i] + KiGain[i]*integral[i] + KdGain[i]*dError[i];
			
			}

	        m_falconDevice.setForce(forces); //Write falcon forces to driver (Forces updated on IO loop) Should run @ ~1kHz
              
            ROS_DEBUG("Force= %f %f %f",forces[0], forces[1], forces[2]);

            prevError = Error;	//Store current error for comparison on next loop

			//IF CURRENT ERROR AND DELTA ERROR < SOME THRESHOLD, INDICATE DONE
			if(fabs(Error[0])<=ErrorThreshold && fabs(Error[1])<=ErrorThreshold && fabs(Error[2])<=ErrorThreshold && fabs(dError[0])<=dErrorThreshold && fabs(dError[1])<=dErrorThreshold && fabs(dError[2])<=dErrorThreshold)
            {
				//Return movement finished
                ROS_DEBUG("Movement finished");
				return true;
            }
			else
            {
				//Return still moving
				return false;
			}

}

int main(int argc, char* argv[])
{
    ros::init(argc,argv, "FalconCSPID");

    ros::NodeHandle node;
    ros::Rate loop_rate(1000);

	//Initialise to 0    
	for(int i = 0; i < 3; i++)
    {
        prevError[i] = 0;
        integral[i] = 0;
    }

	//GAIN VALUES

    KpGain[0] = 110;        //X axis Proportional gain. Ku: 185, Period of Oscillation at Ku: 166ms (Recommended 110)
    KiGain[0] = 0.3;      //X axis Integral gain. (Recommended 0.3)
    KdGain[0] = 2303;    //X axis Derivative gain. (Recommended 2303)

    KpGain[1] = 110;        //Y axis Proportional gain. Ku: 185, Period of Oscillation at Ku: 166ms (Recommended 110)
    KiGain[1] = 0.3;      //Y axis Integral gain. (Recommended 0.3)
    KdGain[1] = 2303;    //Y axis Derivative gain. (Recommended 2303)
    
    KpGain[2] = 111;    //Z axis Proportional gain. Ku: 200, Period of Oscillation at Ku: 300ms (Recommended 111)
    KiGain[2] = 0.2;    //Z axis Integral gain. (Recommended 0.2)
    KdGain[2] = 4500;   //Z axis Derivative gain. (Recommended 4500)

    //Default location (Falcon will move to this position following homing)
    SetPoint[0] = 0;    //-0.065 <= X <= 0.065
    SetPoint[1] = 0;    //-0.065 <= Y <= 0.065
    SetPoint[2] = 0.075; //0.0 <= Z <= 0.175

    
    int falcon_int;
    int atpos_count = 0;

    node.param<int>("falcon_number", falcon_int, 0);
	
	if(init_falcon(falcon_int))

	{ 
		ROS_INFO("Falcon Initialised. Starting Falcon Cartesian Space PID control node");

		m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();

		//Start publisher and subscriber

		ros::Publisher falcon_atpos_pub = node.advertise<std_msgs::Bool>("falcon_atpos", 1);
		
		ros::Subscriber setpoint_sub = node.subscribe("falcon_setpoint", 1, get_setpoint);

        while(node.ok())
        {
    
            if(m_falconDevice.runIOLoop())
            {
    
                ros::spinOnce();        //Get any new messages

                bool atpos = runPID();    //Perform PID control. Returns true when at setpoint, false when in transit
                std_msgs::Bool msg;
                //If falcon moving, immediately publish. Otherwise, wait 100 loops allowing for oscillation
                if (atpos == false)
                {
                    msg.data = atpos;
                    falcon_atpos_pub.publish(msg);
                    atpos_count = 0;
                }                
                else
                {
                    atpos_count++;

                    if(atpos_count >= 100)
                        msg.data = atpos;
                        falcon_atpos_pub.publish(msg);
                }
            }
	
		    loop_rate.sleep();
	    }

        ROS_INFO("Closing connection to Falcon");

        m_falconDevice.close();	
    
    }

    ROS_INFO("Closing Falcon Cartesian Space PID Control node");    

	return 0;
}
    

        
