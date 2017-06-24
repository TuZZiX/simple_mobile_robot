#!/usr/bin/python

import time
import sys
import subprocess
import rospy


def kill_process():
    global process
    if process:
        process.terminate()

def run_command(command):
    global process
    process = subprocess.Popen(command, shell=True)

def main():
    sleep_time = 5.0    # default
    args = []
    if len(sys.argv) >= 3:
        sleep_time = float(sys.argv[1])
        args = sys.argv[2:]
    else:
        print("Wrong number of arguments!")
        exit(0)

    # something for this project only
    for i, val in enumerate(args):
        if args[i] == "keyboard":
            args[i] = "rosrun simple_mobile_robot keyboard_controller"
        elif args[i] == "none":
            exit(0)
        elif args[i] == "square":
            args[i] = "rosrun simple_mobile_robot path_move"
        elif args[i] == "circle":
            args[i] = "rosrun simple_mobile_robot path_move"
        elif '__' in args[i]:
            args[i] = ""

    command = " ".join(args)
    print("Run [%s], after %f s" % (str(command), sleep_time))
    time.sleep(sleep_time)
    run_command(command)
    while not rospy.is_shutdown():
        time.sleep(sleep_time)  # wait forever, this process will run as daemon process


if __name__ == "__main__":
    rospy.on_shutdown(kill_process)
    main()