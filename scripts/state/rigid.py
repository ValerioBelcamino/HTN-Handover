from gtpyhop import State
from geometry_msgs.msg import Pose
from copy import copy

rigid = State("rigid")

rigid.screwdriver_pose = Pose()
rigid.screwdriver_pose.position.x = 0.52 - 0.06
rigid.screwdriver_pose.position.y = -0.31 
rigid.screwdriver_pose.position.z = -0.20 #-0.22
rigid.screwdriver_pose.orientation.x = 1.0
rigid.screwdriver_pose.orientation.y = 0.0
rigid.screwdriver_pose.orientation.z = 0.0
rigid.screwdriver_pose.orientation.w = 0.0

rigid.screwdriver_pose2 = Pose()
rigid.screwdriver_pose2.position.x = 0.52
rigid.screwdriver_pose2.position.y = -0.31
rigid.screwdriver_pose2.position.z = -0.32
rigid.screwdriver_pose2.orientation.x = 1.0
rigid.screwdriver_pose2.orientation.y = 0.0
rigid.screwdriver_pose2.orientation.z = 0.0
rigid.screwdriver_pose2.orientation.w = 0.0

rigid.handover_location = Pose()
rigid.handover_location.position.x = 0.82
rigid.handover_location.position.y = -0.13
rigid.handover_location.position.z = 0.16
rigid.handover_location.orientation.x = 1.0
rigid.handover_location.orientation.y = 0.0
rigid.handover_location.orientation.z = 0.0
rigid.handover_location.orientation.w = 0.0
# rigid.handover_location.position.x = 0.95
# rigid.handover_location.position.y = -0.13
# rigid.handover_location.position.z = 0.21
# rigid.handover_location.orientation.x = 0.0
# rigid.handover_location.orientation.y = 0.7071068
# rigid.handover_location.orientation.z = 0.0
# rigid.handover_location.orientation.w = 0.7071068

rigid.X = Pose()
rigid.X.position.x = 0.55
rigid.X.position.y = -0.77
rigid.X.position.z = 0.046
rigid.X.orientation.x = 1.0
rigid.X.orientation.y = 0.0
rigid.X.orientation.z = 0.0
rigid.X.orientation.w = 0.0

rigid.Y = Pose()
rigid.Y.position.x = 0.55
rigid.Y.position.y = -0.77
rigid.Y.position.z = 0.046
rigid.Y.orientation.x = 1.0
rigid.Y.orientation.y = 0.0
rigid.Y.orientation.z = 0.0
rigid.Y.orientation.w = 0.0

rigid.test = Pose()
rigid.test.position.x = 0.67
rigid.test.position.y = -0.61
rigid.test.position.z = -0.27 #-0.22
rigid.test.orientation.x = 1.0
rigid.test.orientation.y = 0.0
rigid.test.orientation.z = 0.0
rigid.test.orientation.w = 0.0

rigid.workspace = Pose()
rigid.workspace.position.x = 0.52+0.10
rigid.workspace.position.y = -0.31+0.10
rigid.workspace.position.z = -0.20 #-0.22
rigid.workspace.orientation.x = 1.0
rigid.workspace.orientation.y = 0.0
rigid.workspace.orientation.z = 0.0
rigid.workspace.orientation.w = 0.0

rigid.workspace2 = Pose()
rigid.workspace2.position.x = 0.52+0.10
rigid.workspace2.position.y = -0.31+0.10
rigid.workspace2.position.z = -0.32
rigid.workspace2.orientation.x = 1.0
rigid.workspace2.orientation.y = 0.0
rigid.workspace2.orientation.z = 0.0
rigid.workspace2.orientation.w = 0.0

rigid.locations = {'table': [rigid.screwdriver_pose, rigid.screwdriver_pose2], 
                   'exchange point': [rigid.handover_location],
                    'X': [rigid.X],
                    'Y': [rigid.Y],
                    'test': [rigid.test],
                    'workspace': [rigid.workspace, rigid.workspace2]} #place in front of the human where he assembles the chair
