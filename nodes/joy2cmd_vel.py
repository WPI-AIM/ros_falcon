import roslib; roslib.load_manifest('rosfalcon')

import rospy
from geometry_msgs.msg import Twist
from joy.msg import Joy

def joyCallBack(data,cmd_vel):
    #print data
    cmd_vel.linear.x = (data.axes[2]-0.1)*-10
    cmd_vel.angular.z = data.axes[0]*-10
    

def run():
    rospy.init_node('joy2cmd_vel', anonymous=True)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)
    cmd_vel = Twist()
    sub_joy = rospy.Subscriber('falconJoy', Joy, joyCallBack, cmd_vel)
    rate = rospy.Rate(15.0)
    while not rospy.is_shutdown():
        #print cmd_vel
        pub_cmd_vel.publish(cmd_vel)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
