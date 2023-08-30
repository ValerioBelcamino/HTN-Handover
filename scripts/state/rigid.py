from gtpyhop import State
from geometry_msgs.msg import Pose, Point, Quaternion
from copy import copy

rigid = State("rigid")



rigid.screwdriver_pose = Pose(
    Point(
        x = -0.165,
        y = -0.82,
        z = -0.20 #-0.22,
    ),
    Quaternion(
        x = -0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.screwdriver_pose2 = Pose(
    Point(
        x = -0.165,
        y = -0.82,
        z = -0.32 #-0.22,
    ),
    Quaternion(
        x = -0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box_loc1 = Pose(
    Point(
        x = -0.139,
        y = 0.795,
        z = -0.20 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box_loc2 = Pose(
    Point(
        x = -0.139,
        y = 0.815,
        z = -0.31 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box2_loc1 = Pose(
    Point(
        x = -0.003,
        y = 0.783,
        z = -0.20 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box2_loc2 = Pose(
    Point(
        x = -0.003,
        y = 0.803,
        z = -0.31 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box3_loc1 = Pose(
    Point(
        x = 0.103,
        y = 0.786,
        z = -0.20 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)

rigid.box3_loc2 = Pose(
    Point(
        x = 0.103,
        y = 0.806,
        z = -0.31 #-0.22,
    ),
    Quaternion(
        x = 0.7071068,
        y = 0.7071068,
        z = 0.0,
        w = 0.0
    )
)


rigid.handover_location = Pose(
    Point(
        x = 0.82,
        y = -0.23 ,
        z = 0.16,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.X = Pose(
    Point(
        x = 0.55,
        y = -0.77 ,
        z = 0.046,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)


rigid.Y = copy(rigid.X)

rigid.workspace = Pose(
    Point(
        x = 0.60,
        y = -0.13 ,
        z = -0.20,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspace2 = Pose(
    Point(
        x = 0.60,
        y = -0.13 ,
        z = -0.32,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspaceL = Pose(
    Point(
        x = 0.60,
        y = 0.13 ,
        z = -0.20,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.workspace2L = Pose(
    Point(
        x = 0.60,
        y = 0.13 ,
        z = -0.32,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.tuck_poseR = Pose(
    Point(
        # x = 0.38,
        # y = -0.43,
        # z = -0.20,
        x = -0.1,
        y = -0.59,
        z = -0.18,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)

rigid.tuck_poseL = Pose(
    Point(
        # x = 0.38,
        # y = 0.43,
        # z = -0.20,
        x = -0.1,
        y = 0.57,
        z = -0.18,
    ),
    Quaternion(
        x = 1.0,
        y = 0.0,
        z = 0.0,
        w = 0.0
    )
)




# rigid.screwdriver_pose.position.x = 0.52 - 0.06
# rigid.screwdriver_pose.position.y = -0.31 
# rigid.screwdriver_pose.position.z = -0.20 #-0.22
# rigid.screwdriver_pose.orientation.x = 1.0
# rigid.screwdriver_pose.orientation.y = 0.0
# rigid.screwdriver_pose.orientation.z = 0.0
# rigid.screwdriver_pose.orientation.w = 0.0

# rigid.screwdriver_pose2 = Pose()
# rigid.screwdriver_pose2.position.x = 0.52
# rigid.screwdriver_pose2.position.y = -0.31
# rigid.screwdriver_pose2.position.z = -0.32
# rigid.screwdriver_pose2.orientation.x = 1.0
# rigid.screwdriver_pose2.orientation.y = 0.0
# rigid.screwdriver_pose2.orientation.z = 0.0
# rigid.screwdriver_pose2.orientation.w = 0.0

# rigid.screwdriver_poseTMP = Pose()
# rigid.screwdriver_poseTMP.position.x = 0.52 - 0.06
# rigid.screwdriver_poseTMP.position.y = 0.31 
# rigid.screwdriver_poseTMP.position.z = -0.20 #-0.22
# rigid.screwdriver_poseTMP.orientation.x = 1.0
# rigid.screwdriver_poseTMP.orientation.y = 0.0
# rigid.screwdriver_poseTMP.orientation.z = 0.0
# rigid.screwdriver_poseTMP.orientation.w = 0.0

# rigid.screwdriver_pose2TMP = Pose()
# rigid.screwdriver_pose2TMP.position.x = 0.52
# rigid.screwdriver_pose2TMP.position.y = 0.31
# rigid.screwdriver_pose2TMP.position.z = -0.32
# rigid.screwdriver_pose2TMP.orientation.x = 1.0
# rigid.screwdriver_pose2TMP.orientation.y = 0.0
# rigid.screwdriver_pose2TMP.orientation.z = 0.0
# rigid.screwdriver_pose2TMP.orientation.w = 0.0




# rigid.handover_location.position.x = 0.95
# rigid.handover_location.position.y = -0.13
# rigid.handover_location.position.z = 0.21
# rigid.handover_location.orientation.x = 0.0
# rigid.handover_location.orientation.y = 0.7071068
# rigid.handover_location.orientation.z = 0.0
# rigid.handover_location.orientation.w = 0.7071068


# rigid.test = Pose()
# rigid.test.position.x = 0.67
# rigid.test.position.y = -0.61
# rigid.test.position.z = -0.27 #-0.22
# rigid.test.orientation.x = 1.0
# rigid.test.orientation.y = 0.0
# rigid.test.orientation.z = 0.0
# rigid.test.orientation.w = 0.0

# rigid.test = Pose()
# rigid.test.position.x = 0.67
# rigid.test.position.y = -0.2
# rigid.test.position.z = -0.31 #-0.22
# rigid.test.orientation.x = 1.0
# rigid.test.orientation.y = 0.0
# rigid.test.orientation.z = 0.0
# rigid.test.orientation.w = 0.0


# rigid.workspace2 = Pose()
# rigid.workspace2.position.x = 0.52+0.10
# rigid.workspace2.position.y = -0.31+0.10
# rigid.workspace2.position.z = -0.32
# rigid.workspace2.orientation.x = 1.0
# rigid.workspace2.orientation.y = 0.0
# rigid.workspace2.orientation.z = 0.0
# rigid.workspace2.orientation.w = 0.0



rigid.locations = {'table': [rigid.screwdriver_pose, rigid.screwdriver_pose2], 
                   'box_location': [rigid.box_loc1, rigid.box_loc2],
                   'box2_location': [rigid.box2_loc1, rigid.box2_loc2],
                   'box3_location': [rigid.box3_loc1, rigid.box3_loc2],
                #    'table2': [rigid.screwdriver_poseTMP, rigid.screwdriver_pose2TMP], 
                   'exchange point': [rigid.handover_location],
                    'X': [rigid.X],
                    'Y': [rigid.Y],
                    # 'test': [rigid.test],
                    'workspace': [rigid.workspace, rigid.workspace2],
                    'workspaceL': [rigid.workspaceL, rigid.workspace2L],
                    'tuck_positionR': [rigid.tuck_poseR],
                    'tuck_positionL': [rigid.tuck_poseL],
                    'brick1_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick2_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick3_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick4_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick5_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick6_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick7_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'brick8_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'shelf1_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'shelf2_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2],
                    'shelf3_pose': [rigid.screwdriver_pose, rigid.screwdriver_pose2]} #place in front of the human where he assembles the chair
