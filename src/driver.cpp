//////////////////////////////////////////////////////////
// ROSfalcon Driver. Publishes and subscribes to falconMsgs for Novint Falcon.
//
// Using LibniFalcon 
// Steven Martin
// Based on barrow_mechanics example by Alistair Barrow


#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <rosfalcon/falconPos.h>
#include <rosfalcon/falconForces.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"


using namespace libnifalcon;
using namespace std;
using namespace StamperKinematicImpl;
FalconDevice m_falconDevice;

/**********************************************
This function initialises the novint falcon controller

NoFalcons is the index of the falcon which you wish to initialise
Index 0 is first falcon.
**********************************************/
 
bool init_falcon(int NoFalcon) 

{
  	cout << "Setting up LibUSB" << endl;
    m_falconDevice.setFalconFirmware<FalconFirmwareNovintSDK>(); //Set Firmware
  	if(!m_falconDevice.open(NoFalcon)) //Open falcon @ index 0  (index needed for multiple falcons, assuming only one connected)
  	{
  	    cout << "Failed to find falcon" << endl;
        return false;
  	}
    else
    {
        cout << "Falcon Found" << endl;
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
		cout << "Loading firmware" << endl;
		uint8_t* firmware_block;
		long firmware_size;
		{

			firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);
			firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;


			for(int i = 0; i < 20; ++i)
			{
				if(!m_falconDevice.getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))

				{
					cout << "Firmware loading try failed" <<endl;
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
		cout << "No firmware loaded to device, and no firmware specified to load (--nvent_firmware, --test_firmware, etc...). Cannot continue" << endl;
		return false;
	}
	if(!firmware_loaded || !m_falconDevice.isFirmwareLoaded())
	{
		cout << "No firmware loaded to device, cannot continue" << endl;
		return false;
	}
	cout << "Firmware loaded" << endl;
    
    m_falconDevice.getFalconFirmware()->setHomingMode(true); //Set homing mode (keep track of encoders !needed!)
    cout << "Homing Set" << endl;
    boost::array<int, 3> forces;
    m_falconDevice.getFalconFirmware()->setForces(forces);
  	m_falconDevice.runIOLoop(); //read in data  	

  	{
	    bool stop = false;
	    bool homing = false;
	    bool homing_reset = false;
	    usleep(100000);
        int tryLoad = 0;
	    while(!stop && tryLoad < 100)
	    {
		    if(!m_falconDevice.runIOLoop()) continue;
		    if(!m_falconDevice.getFalconFirmware()->isHomed())
		    {
			    if(!homing)
			    {
				    m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::RED_LED);
				    cout << "Falcon not currently homed. Move control all the way out then push straight all the way in." << endl;
       	        
			    }
			    homing = true;
		    }

		    if(homing && m_falconDevice.getFalconFirmware()->isHomed())
		    {
			    m_falconDevice.getFalconFirmware()->setLEDStatus(libnifalcon::FalconFirmware::BLUE_LED);
			    cout << "Falcon homed." << endl;
			    homing_reset = true;
			    stop = true;
		    }
            tryLoad++;
	    }
        while(!m_falconDevice.runIOLoop());
    }
    return true;	
}

void forceCallback(const rosfalcon::falconForcesConstPtr& msg)
{
    boost::array<double,3> forces;
    forces[0] = msg->X;
    forces[1] = msg->Y;
    forces[2] = msg->Z;
	m_falconDevice.setForce(forces);

    cout << "set Force" << endl;
    //TODO Add safety to only apply forces for limited time
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv, "ROSfalcon");
    
    //TODO Driver currently assumes there is only one falcon attached 
    if(init_falcon(0))
    { 
        cout << "Falcon Initialised Starting ROS Node" << endl;


        m_falconDevice.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
        ros::NodeHandle node;
        
        //Start ROS Subscriber
        ros::Subscriber sub = node.subscribe("/falconForce", 10, &forceCallback);
        
        //Start ROS Publisher
        ros::Publisher pub = node.advertise<rosfalcon::falconPos>("falconPos",10);        

        ros::Rate loop_rate(1000);

        while(node.ok())
	    {
		    if(m_falconDevice.runIOLoop())
            {
		        //////////////////////////////////////////////
		        //Request the current encoder positions:
		        boost::array<double, 3> Pos;
		        Pos = m_falconDevice.getPosition();

                //Publish ROS values
		        rosfalcon::falconPos position;
                position.X = Pos[0];
                position.Y = Pos[1];
                position.Z = Pos[2];
                pub.publish(position);
                
            }
            loop_rate.sleep();
	    }
        m_falconDevice.close();
    }

	return 0;
}

