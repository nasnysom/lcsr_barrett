#!/usr/bin/env python
import rospy
from math import pi, sin, cos
from sensor_msgs.msg import JointState

from copy import copy
import actionlib
import control_msgs.msg

from operator import add

import controller_manager_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg

import tf
from tf_conversions.posemath import fromTf, toTf, toMsg


def main():
    rospy.init_node('all_controllers_test_node')

    # Connect to the controller switching service
    rospy.loginfo("Connecting to controller manager...")
    rospy.wait_for_service('controller_manager/switch_controller')
    switch_controllers = rospy.ServiceProxy(
            'controller_manager/switch_controller',
            controller_manager_msgs.srv.SwitchController)

    rospy.wait_for_service('controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)

    rospy.loginfo("Connected to controller manager")

    tfl = tf.TransformListener()
    bc = tf.TransformBroadcaster()
    rospy.loginfo("Connected to tf")
    
    rospy.loginfo("Listing controllers:")
    print list_controllers()
    
    
    


    # Enable joint control group
    switch_controllers(['joint_control',],['ik_control','cart_imp_control'],1)
    
    # commanding through control_msgs.msg.FollowJoingTrajectoryAction action client
    rospy.loginfo("commanding joint trajectories through control_msgs.msg.FollowJointTrajectoryAction action client")
    
    traj_client = actionlib.SimpleActionClient(
            '/gazebo/traj_rml/action', control_msgs.msg.FollowJointTrajectoryAction)
    traj_client.wait_for_server()
    rospy.loginfo("Connected to traj action.")

    # Crate a traj goal
    traj_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    traj_goal.trajectory.header.stamp = rospy.Time(1.0,0)
    
    # Create the first test point
    point_pi4 = trajectory_msgs.msg.JointTrajectoryPoint()
    point_pi4.positions = [
            0,
            -pi/4.0,
            0,
            pi/4.0,
            pi/4.0,
            0,
            pi/4.0]

    traj_goal.trajectory.points.append(point_pi4)

    traj_client.send_goal(traj_goal)
    traj_client.wait_for_result()


    # commanding through raw topic publishing
    rospy.loginfo("testing raw topic streaming joint position")
    jointpoint_pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_point_cmd', 
                                              trajectory_msgs.msg.JointTrajectoryPoint, 
                                              queue_size=5)

    rospy.loginfo("commanding joint positions in a loop for 5 seconds")
    tstart = rospy.get_time()

    jointpoint = trajectory_msgs.msg.JointTrajectoryPoint()
    jointpoint.time_from_start = rospy.Time(0)
    

    jointpoint.positions = copy(point_pi4.positions)
    r = rospy.Rate(50)
    while True:
        t = rospy.get_time() - tstart
        if t > 5.0:
            break
        
        jointpoint.positions[3] = point_pi4.positions[3] + pi/3.0*sin(2.0*pi*0.25*t)

        jointpoint_pub.publish(jointpoint)
        r.sleep()


    rospy.loginfo("returning to traj goal")
    traj_client.send_goal(traj_goal)
    traj_client.wait_for_result()


    rospy.loginfo("switching to jtns pose control")



    

    # get current pose of wrist palm stump link
    rospy.loginfo("waiting for transform");

    tfl.waitForTransform("wam/base_link", "wam/wrist_palm_stump_link", rospy.Time(), rospy.Duration(4.0))
    (trans, rot)=tfl.lookupTransform("wam/base_link", "wam/wrist_palm_stump_link", rospy.Time())   
    
    print trans
    
    bc.sendTransform(trans, rot, rospy.Time(0),"/wam/cmd", "/wam/base_link")
    
    rospy.loginfo("commanding poses in a loop for 10 seconds")
    dp = [0.0, 0.0, 0.0]
    tstart = rospy.get_time()

    switch_controllers(['cart_imp_control'],['joint_control'],1)    
    while True:
        t = rospy.get_time() - tstart


        # if t>10.0:
        #     break
        dp[0] = 0.5*sin(2.0*pi*0.25*t)
        dp[1] = 0.5*cos(2.0*pi*0.25*t)
        bc.sendTransform(map(add, trans, dp), rot, rospy.Time(0),"/wam/cmd", "/wam/base_link")
        r.sleep()

        
    
    
    
if __name__ == '__main__':
    main()
