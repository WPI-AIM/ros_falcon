#!/usr/bin/env python

#sgillen - this program serves as a node that offers the arduino up to the rest of the ros system.

import serial, time, sys, select
import rospy
from std_msgs.msg import Int64, Float64, String, Float64MultiArray
from std_srvs.srv import Empty, EmptyResponse
from ros_falcon.msg import falconForces, falconPos

device = '/dev/ttyACM1' # TODO

pose_cmds = [0,0,0]

# When this gets flipped, send shutdown signal
shutdown_flag = False

#reads a command from stdin
def read_cmd_stdin():
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = sys.stdin.readline()
      line = line.rstrip()
      if line:
        num_bytes = ser.write(line)
        print  "bytes sent =", num_bytes


#sends an array of ints to the thrusters using the agreed upon protocol
#the actual over the wire value is t,x,y,z!
def send_pose_cmds(pose_cmd):
    cmd_str = "t"
    for cmd in pose_cmds:
        cmd_str += (",")
        cmd_str += (str(cmd))

    cmd_str += ("!")
    ser.write(cmd_str)
    #print "arduino return", ser.readline()
    ##TODO parse return value


##------------------------------------------------------------------------------
# callbacks
def shutdown_thrusters(srv):
    global shutdown_flag
    shutdown_flag = True
    return EmptyResponse()

def pose_callback(msg):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.X)
#    print "I heard %s" %msg.X
    global pose_cmds
    pose_cmds[0] = msg.X
    pose_cmds[1] = msg.Y
    pose_cmds[2] = msg.Z

##------------------------------------------------------------------------------
if __name__ == '__main__':

    #!!! this also restarts the arduino! (apparently)

    print "trying to connect to arduino"
   # Keep trying to open serial
    while True:
        try:
            ser = serial.Serial(device,115200, timeout=0,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            break
        except:
            time.sleep(0.25)
            continue


    print "found it!"
    time.sleep(3)


    force_pub = rospy.Publisher('falconForce', falconForces, queue_size = 10)
    rospy.init_node('lynx_node', anonymous=False)
    
    pose_sub = rospy.Subscriber('falconPos', falconPos, pose_callback)
    
    rate = rospy.Rate(1000) #100Hz

    while not rospy.is_shutdown():

        #send_pose_cmds(pose_cmds)

        #ser.write('c!')
        #temp = ser.readline()
        #print(temp)

        # get thruster cmd
        x = ser.readline().strip()
        print x
        
        # if x != '':
        #     msg[x[0]] = x[1:]

        # x = ser.readline().strip()
        # if x != '':
        #     msg[x[0]] = x[1:]
        #status = ser.readline().strip()

        
        force_pub.publish(falconForces(1,1,1))
        #print pose_cmds
        
        rate.sleep()
