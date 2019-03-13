import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ros_falcon.msg import falconForces, falconPos


pose_goal = geometry_msgs.msg.Pose()



# This callback will take a falconPos message, concert it to a geometry pose message, and then set the "group" pose
def pose_callback(msg):
    

    
    global pose_goal
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = msg.X*10.0
    pose_goal.position.y = msg.Y*10.0
    pose_goal.position.z = msg.Z*10.0

    print "pose goal now contains: ",  pose_goal.position.x, pose_goal.position.y, pose_goal.position.z
    #    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.X)

    
if __name__ == "__main__":

    
    moveit_commander.roscpp_initialize(sys.argv)

    force_pub = rospy.Publisher('falconForce', falconForces, queue_size = 10)

    rospy.init_node('lynx_node',  anonymous=True)


    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    
    rate = rospy.Rate(10) #Hz
    
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)


    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    
    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    
    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    
    
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    
    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()



    pose_sub = rospy.Subscriber('falconPos', falconPos, pose_callback)    

    while not rospy.is_shutdown():


        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)

        # We probably don't actually want to do this all in a callback
        
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
