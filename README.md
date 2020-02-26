# Description
This package allows ROS interface with the Novint Falcon Device.

## Authors
Steven Martin:

Adnan Munawar: amunawar@wpi.edu

## Dependencies
Tested on ROS Hydro, Indigo and Melodic.

Download and install the drivers from:

https://github.com/libnifalcon/libnifalcon
 
Make sure to install them in addition to building them.

Make sure udev permissions are set correctly. In Ubuntu 14.04, 16.04 and 18.04 

  **roscd ros_falcon**
  
  **sudo cp udev_rules/99-udev-novint.rules /etc/udev/rules.d**
  
  **unplug - replug falcon**


### Known Issues:

**Error while loading shared libraries: libnifalcon.so.1.0.2: cannot open shared object file: No such file or directory**

Try running `sudo ldconfig`

