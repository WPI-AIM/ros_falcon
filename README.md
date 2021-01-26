# Description
This package allows ROS interface with the Novint Falcon Device.

## 1. Authors
Steven Martin:

Adnan Munawar: amunawar@wpi.edu

## 2. Dependencies
Tested on ROS Hydro, Indigo and Melodic. For this package to work, please download, build and install the `libnifalcon` drivers from:

https://github.com/libnifalcon/libnifalcon
 
## 3. Setting the permissions
Setting the udev permissions is required. The following commands can be used for this purpose: 
```bash
roscd ros_falcon
sudo cp udev_rules/99-udev-novint.rules /etc/udev/rules.d
```
Then unplug and replug the device. You may also need to restart the OS.

## 4. How to Run:

Before running this ROS program, you may need to run the utility binaries from the `libnifalcon` package. 
If you installed the `libnifalcon` package properly (i.e. by running `sudo make install`), the binaries should be in 
your system include path. Otherwise, they should be in `<libnifalcon/build/bin>` folder.

Make sure that the power cord in plugged into the Falcon and the Falcon is connected to the PC. Run the following in your terminal

```bash
findfalcons
```
You should see the Falcon's LEDs change color and the terminal should indicated whether the Falcon is ready or not. If the program fails, running it a few times helps. You may need to run the program with `sudo` if the `udev` permissions were not set properly. However, you would need the `udev` set correctly for the ROS pacakge to work.

**IMPORTANT: YOU MAY ALSO NEED TO HOME THE FALCON WHILE THE PROGRAM IS RUNNING. READ SECTION 4.1 ON HOW TO DO THAT**

Now you can run another utility program to test an example where a virtual spherical collider is placed at the center of the Falcon's workspace.

```bash
barrow_mechanics
```

### 4.1 Homing the Falcon:
The Falcon is homed by running any utility program that accesses the Falcon, then pulling the knob all the way outwards and then pushing it inwards until you sense a locking force. At this point, the falcon is homed.


If these steps work, then you should be able to use the ROS pacakge.

### 4.2. Running the ROS package:

```
rosrun ros_falcon driver
```
This program should spawn the required ROS topics for state/command of the Falcon device.

## 5. Known Issues:

### 5.1 Moving the Falcon does not change the state data
You may need to home the Falcon as described in 4.1

### 5.2 Library not found

For the following error, ``Error while loading shared libraries: libnifalcon.so.1.0.2: cannot open shared object file: No such file or directory``

Try running `sudo ldconfig`

