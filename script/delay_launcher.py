#!/usr/bin/python

import time
import sys
import subprocess
import rospy


def kill_process():
    if "process" in globals():
        process.terminate()

def run_command(command):
    global process
    process = subprocess.Popen(command, shell=True)

def main():
    rospy.init_node("delay_launcher", anonymous=True)
    sleep_time = 5.0    # default
    args = []
    if len(sys.argv) >= 3:
        sleep_time = float(sys.argv[1])
        args = sys.argv[2:]
    else:
        print("Wrong number of arguments!")
        exit(0)

    for i, val in enumerate(args):
        if "__" in args[i]:
            args.pop(i)
        # something for this project only
        elif args[i] == "keyboard":
            args[i] = "rosrun simple_mobile_robot keyboard_controller"
        elif args[i] == "square":
            args[i] = "rosrun simple_mobile_robot square_move"
        elif args[i] == "circle":
            args[i] = "rosrun simple_mobile_robot circle_move"
        elif args[i] == "none":
            exit(0)

    command = " ".join(args)
    rospy.loginfo("Run: %s, after %f s" % (command, sleep_time))
    time.sleep(sleep_time)
    rospy.loginfo("Start: %s" % command)
    run_command(command)
    while not rospy.is_shutdown():
        time.sleep(sleep_time)  # wait forever, this process will run as daemon process


if __name__ == "__main__":
    try:
        rospy.on_shutdown(kill_process)
        main()
    except Exception as e:
        print(e)
        kill_process()
        exit(0)