#!/usr/bin/env python

import gtpyhop
import rospy
from joint_trajectory_client import TrajectoryClient
import baxter_interface
from baxter_interface import CHECK_VERSION
import argparse

domain = gtpyhop.Domain('handover')
from methods.methods import *
from actions.actions import *
from state.rigid import rigid

def main():

    gtpyhop.current_domain = domain

    from state.state import state

    state.display('This is initial state')

    state1 = state.copy()
    # state1.display('This is state1')
    gtpyhop.verbose = 3

    gtpyhop.print_domain()

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
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_%s" % (args.limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running...")
    client = TrajectoryClient(args.limb)    
    client.traj_p.open_gripper()
    client.transfer(rigid.locations['test'])
    time.sleep(1)
    result = gtpyhop.find_plan(state1, [('handover', 'robot', 'human', 'box', client), ('handover', 'robot', 'human', 'screwdriver', client)])

    print(result)

if __name__ == "__main__":
    main()
