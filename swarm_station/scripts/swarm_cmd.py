#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
from swarm_msgs.msg import swarm_remote_command, drone_onboard_command
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A easy command tool for sending command to swarm drone')
    parser.add_argument('command_type', metavar='command_type', choices=["takeoff", "landing", "flyto", "arm", "disarm", "joy_control"], help="Type of command to send")
    parser.add_argument("-i", "--node_id", type=int, default=-1,
        help="The node id that pending to control, when default node_id=-1, that is control all UAVs")
    parser.add_argument("-o","--onboard", action="store_true", help="If use this onboard.")
    parser.add_argument("params",nargs="*", type=float, help="parameters for command")
    args = parser.parse_args()

    print("Will send command {} by id {} with params {}".format(args.command_type, args.node_id, args.params))

    if args.onboard:
        print("The command will send onboard!")
    else:
        print("The command will send remote!")

    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)

    rospy.init_node('cmded', anonymous=True)

    cmd = drone_onboard_command()

    if args.command_type == "takeoff":
        if len(args.params) < 1:
            rospy.logwarn("No height specs, will fly to default 1.0m")
            height = 1.0
        else:
            height = args.params[0]
        cmd.command_type = drone_onboard_command.CTRL_TAKEOF_COMMAND
        cmd.param1 = int(height*10000)

    elif args.command_type == "landing":
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND

    elif args.command_type == "arm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 1

    elif args.command_type == "disarm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 0

    elif args.command_type == "flyto":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        if len(args.params) < 3:
            rospy.logerr("Must give xyz when using flyto")
            sys.exit(-1)
        else:
            cmd.param1 = int(args.params[0]*10000)
            cmd.param2 = int(args.params[1]*10000)
            cmd.param3 = int(args.params[2]*10000)

            if len(args.params) == 4:
                cmd.param4 = int(args.params[3]*10000)
            else:
                cmd.param4 = 666666
            cmd.param5 = 0
            cmd.param6 = 0
            cmd.param7 = 0
            cmd.param8 = 0

    if args.onboard:
        print("Sending to onboard")
        print(cmd)
        pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)
        # pub.publish(cmd)
        
    else:
        print("Sending to remote")
        print(scmd)
        scmd = swarm_remote_command()
        scmd.target_id = args.target_id
        scmd.cmd = cmd        
        pub = rospy.Publisher("/swarm_drones/send_swarm_command", swarm_remote_command, queue_size=1)
        # pub.publish(scmd)
        cmd = scmd

    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        print('Connections ', connections)
        if connections > 0:
            pub.publish(cmd)
            break
        rate.sleep()