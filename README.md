# Description
This package allows ROS interface with the Novint Falcon Device.

## Authors
Steven Martin:

Adnan Munawar: amunawar@wpi.edu

## Dependencies
Tested on ROS Hydro, Indigo and Melodic. For this package to work, please download, build and install the `libnifalcon` drivers from:

https://github.com/libnifalcon/libnifalcon
 
## Setting the permissions
Setting the udev permissions is required. The following commands can be used for this purpose: 
```bash
roscd ros_falcon
sudo cp udev_rules/99-udev-novint.rules /etc/udev/rules.d
```
Then unplug and replug the device. You may also need to restart the OS.

## How to Run:

Before running this ROS program. You may need to run the utility binaries from the `libnifalcon` package. 
If you installed the `libnifalcon` package properly (i.e. by running `sudo make install`), the binaries should be in 
your system include path. 

Make sure that the power cord in plugged into the Falcon and the Falcon is connected to the PC. Run the following in your terminal

```bash
findfalcons
```
You should see the Falcon's LEDs change color and the terminal should indicated whether the Falcon is ready or not. If the program fails, running it a few times helps. You may need to run the program with `sudo` if the `udev` permissions were not set properly. However, you would need the `udev` set correctly for the ROS pacakge to work.

**IMPORTANT: YOU MAY ALSO NEED TO HOME THE FALCON WHILE THE PROGRAM IS RUNNING. THIS IS DONE BY PULLING OUT THE FALCON ALL THE WAY AND PUSHING IT INWARDS AT WHICH POINT YOU SHOULD FEEL A SMALL LOCKING FORCE.**

Now you can run another utility program to test an example where a virtual spherical collider is placed at the center of the Falcon's workspace.

```bash
barrow_mechanics
```

If these steps work, then you should be able to use the ROS pacakge.

### Running the ROS package:

```
rosrun ros_falcon driver
```
This program should spawn the required ROS topics for state/command of the Falcon device.

### Known Issues:

**Error while loading shared libraries: libnifalcon.so.1.0.2: cannot open shared object file: No such file or directory**

Try running `sudo ldconfig`

