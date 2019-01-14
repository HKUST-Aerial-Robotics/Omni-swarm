#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
from swarm_msgs.msg import swarm_remote_command, drone_onboard_command
import sys
import math

def send(cmd, args, pub):
    if args.onboard:
        pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)
        pub.publish(cmd)
    else:
        scmd = swarm_remote_command()
        scmd.target_id = args.node_id
        scmd.cmd = cmd        
        pub.publish(scmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A easy command tool for sending command to swarm drone')
    parser.add_argument('command_type', metavar='command_type', choices=
        ["takeoff", "landing", "flyto", "arm", "disarm", "joy_control", "circle"], help="Type of command to send")
    parser.add_argument("-i", "--node_id", type=int, default=-1,
        help="The node id that pending to control, when default node_id=-1, that is control all UAVs")
    parser.add_argument("-o","--onboard", action="store_true", help="If use this onboard.")
    parser.add_argument("-c","--center", nargs=3, type=float, help="center for circle", default=[0, 0, 1])
    parser.add_argument("-r","--radius", nargs=1, type=float, help="radius for circle", default=0.5)
    parser.add_argument("-T","--cycle", nargs=1, type=float, help="cycle for circle", default=10)

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


    if args.onboard:
        print("Sending to onboard")
        pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)
        # pub.publish(cmd)
        
    else:
        print("Sending to remote")
        pub = rospy.Publisher("/swarm_drones/send_swarm_command", swarm_remote_command, queue_size=1)

    rate = rospy.Rate(20)  # 20hz
    
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        print("Wait for pub")
        if connections > 0:
            break
        rate.sleep()
    cmd = drone_onboard_command()

    if args.command_type == "takeoff":
        if len(args.params) < 1:
            rospy.logwarn("No height specs, will fly to default 1.0m")
            height = 1.0
        else:
            height = args.params[0]
        cmd.command_type = drone_onboard_command.CTRL_TAKEOF_COMMAND
        cmd.param1 = int(height*10000)
        send(cmd, args, pub)

    elif args.command_type == "landing":
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND
        send(cmd, args, pub)


    elif args.command_type == "arm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 1
        send(cmd, args, pub)


    elif args.command_type == "disarm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 0
        send(cmd, args, pub)


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


        while not rospy.is_shutdown():
            send(cmd, args, pub)
            rate.sleep()

    elif args.command_type == "circle":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        print("Will draw circle @ origin {} {} {}, r {} T {}".format(
            args.center[0],
            args.center[1],
            args.center[2],
            args.radius,
            args.cycle
        ))

        ox = args.center[0]
        oy = args.center[1]
        oz = args.center[2]
        r = args.radius
        T = args.cycle


        cmd.param1 = 0
        cmd.param2 = 0
        cmd.param3 = 0
        cmd.param4 = 666666
        cmd.param5 = 0
        cmd.param6 = 0
        cmd.param7 = 0
        cmd.param8 = 0
        t = 0
        while not rospy.is_shutdown():
            x = ox + math.sin(t*math.pi*2/T)*r
            y = oy + math.cos(t*math.pi*2/T)*r
            cmd.param1 = int(x*10000)
            cmd.param2 = int(y*10000)
            cmd.param3 = int(oz*10000)
            print("{:3.2f} xyz {:3.2f} {:3.2f} {:3.2f}".format(t, x, y, oz))
            send(cmd, args, pub)
            t = t + 0.05
            rate.sleep()
