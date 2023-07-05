#!/usr/bin/env python

import argparse
import sys
from threading import Event
import signal
from copy import copy, deepcopy
import rospy
import actionlib
from time import sleep
import baxter_interface
from trajectoryPlanner import TrajectoryPlanner
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    GripperCommandActionGoal
)
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from geometry_msgs.msg import Pose

from baxter_moveit.msg import MoveitTrajectory

from baxter_interface import CHECK_VERSION

# Client class to provides trajectory execution service
class TrajectoryClient():

    def __init__(self, limb):
        self.limb = limb
        rospy.Subscriber("/right_arm/baxter_moveit_trajectory", MoveitTrajectory, self.callback)
        self.release_sub = rospy.Subscriber("/melexis_release", Bool, self.release_callback)
        self.aruco_objects = rospy.Subscriber("/aruco_detection", String, self.aruco_callback)

        self.melexis_activation_pub = rospy.Publisher("/melexis_activation", Bool, queue_size=10)
        self.camera_activation_pub = rospy.Publisher("/camera_listener_activation", Bool, queue_size=10)
        self.aruco_activation_pub = rospy.Publisher("/aruco_detection_activation", Bool, queue_size=10)

        self.trajTime = 6.0 # Time to complete each trajectory 
        self.traj_p = TrajectoryPlanner()
        self.stop_sleeping_sig = Event()

        self.current_obj = []

    def callback(self, msg):

        trajectory = msg.trajectory
        n = len(trajectory)

        for i in range(len(trajectory)):
            traj = Trajectory(self.limb)

            rospy.on_shutdown(traj.stop)
            # Command Current Joint Positions first
            limb_interface = baxter_interface.limb.Limb(self.limb)

            t = 0
            for point in trajectory[i].joint_trajectory.points:
  
                t += self.trajTime/len(trajectory[i].joint_trajectory.points)      #prima c'era 0.1
                traj.add_point(point.positions, point.velocities, point.accelerations, t)
                
            traj.start()
            traj.wait()
        
        self.ex_complete_pub.publish(True)
        print("Action Complete")

    def aruco_callback(self, msg):
        rospy.loginfo("aruco_callback")
        print(msg)
        self.current_obj = msg.data.split('_')
        if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()

    def release_callback(self, msg):
        rospy.loginfo("Release callback")
        if not self.stop_sleeping_sig.is_set():
            self.stop_sleeping_sig.set()
        if msg.data:
            self.traj_p.open_gripper('right')

    def execute_trajectory(self, trajectory, side):

        traj = Trajectory(side)

        rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        # limb_interface = baxter_interface.limb.Limb(self.limb)

        t = 0
        for point in trajectory.joint_trajectory.points:

            t += self.trajTime/len(trajectory.joint_trajectory.points)      #prima c'era 0.1
            traj.add_point(point.positions, point.velocities, point.accelerations, t)
            
        traj.start()
        traj.wait()
    
        print("Action Complete")

    def transfer(self, poses, side):
        failures = False

        for pose in poses:
            rospy.logerr("Transfer function called")
            rospy.logerr(pose)
            rospy.logerr(side)
            temp_pose = deepcopy(pose)
            #reach XY location above object
            simple_traj, fract = self.traj_p.plan_cartesian_trajectory(temp_pose, side)
            
            print('fraction', fract)
            if fract > 0.95:
                self.execute_trajectory(simple_traj, side)
            else:
                rospy.logerr(fract)
                failures = True
        return not failures
            # input("Press Enter to continue...")

        # #remove height offset once you reach XY location
        # temp_pose.position.z -= 0.1
        # simple_traj, fract = self.traj_p.plan_cartesian_trajectory(temp_pose)
        # print('fraction', fract)
        # if fract > 0.99:
        #     self.execute_trajectory(simple_traj)        



    def reach_handover_location(self):
        pose = Pose()
        pose.position.x = 0.82
        pose.position.y = -0.13
        pose.position.z = 0.16
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        simple_traj, fract = self.traj_p.plan_cartesian_trajectory(pose)
        print('fraction', fract)
        if fract > 0.90:
            self.execute_trajectory(simple_traj)



    def handover(self, pose, mode):
        self.transfer(pose)
            #grasp object
        self.traj_p.close_gripper()
        self.reach_handover_location()
        if mode == 'tool':
            self.melexis_activation_pub.publish(True)
        elif mode == 'box':
            self.camera_activation_pub.publish(True)
        print('start sleeping')
        # if self.stop_sleeping_sig.is_set():
        #     self.stop_sleeping_sig.clear()
        # self.stop_sleeping_sig.wait()
        sleep(3)
        print('stop sleeping')



# Trajectory class to handle joint trajectory action
class Trajectory(object):

    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.05)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)
        gripper_topic = "robot/end_effector/" + limb + "_gripper/gripper_action/goal"
        self.gripper_publisher = rospy.Publisher(gripper_topic, GripperCommandActionGoal, queue_size=10)

    def add_point(self, positions, velocities, accelerations, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.accelerations = copy(accelerations)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
            
    def open_gripper(self): 
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 100.0
        self.gripper_publisher.publish(gripperCommandMsg)
    	
    def close_gripper(self):
        gripperCommandMsg = GripperCommandActionGoal()
        gripperCommandMsg.goal.command.position = 0.0
        self.gripper_publisher.publish(gripperCommandMsg)

    

def main():
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l',
        '--limb',
        required=True,
        choices=['left', 'right'],
        help='Send joint trajectory to which limb'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    # Instantiate trajectory handler object
    client = TrajectoryClient(args.limb)

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (args.limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running...")
    

    client.traj_p.open_gripper()
    screwdrivre_pose = Pose()
    screwdrivre_pose.position.x = 0.52
    screwdrivre_pose.position.y = -0.31
    screwdrivre_pose.position.z = -0.22
    screwdrivre_pose.orientation.x = 1.0
    screwdrivre_pose.orientation.y = 0.0
    screwdrivre_pose.orientation.z = 0.0
    screwdrivre_pose.orientation.w = 0.0

    while not rospy.is_shutdown():
        # input("Press Enter to continue...")
        # client.pick_object(screwdrivre_pose)
        # client.reach_handover_location()
        # client.melexis_activation_pub.publish(True)
        # print('start sleeping')
        # if client.stop_sleeping_sig.is_set():
        #     client.stop_sleeping_sig.clear()
        # client.stop_sleeping_sig.wait()
        # print('stop sleeping')
        client.handover(screwdrivre_pose, 'box')
    
    rospy.spin()


if __name__ == "__main__":
    main()
