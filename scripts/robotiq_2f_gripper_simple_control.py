#!/usr/bin/env python3

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from six.moves import input

def gripper_ready():
    rospy.loginfo("Initialize the gripper...")
    rospy.loginfo("Step 1: Reset")
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 0
    rospy.sleep(0.1)

    rospy.loginfo("Step 2: Activate")
    command.rACT = 1 # Activate
    command.rGTO = 1 # Calls for movement
    command.rSP = 127 # Desired speed
    command.rFR = 127 # Desired force

    rospy.sleep(0.1)

    return command

def gripper_grasp(val_int, command):
    rospy.loginfo("The gripper grasps to the value="+ str(val_int))
    try:
        command.rPR = val_int
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass

    return command

def gripper_open(command):
    rospy.loginfo("The gripper opens")
    command.rPR = 0

    return command
    

def publisher():
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node("control_2f_gripper")
    rospy.loginfo("The gripper node has been started")


    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output, queue_size=10
    )

    # Initialize the gripper
    command_gripper_ready = gripper_ready()
    pub.publish(command_gripper_ready)
    rospy.loginfo("Initialization is successful")
    rospy.loginfo("==================================")
    rospy.sleep(0.5)

    # Open the gripper
    command_open = gripper_open(command_gripper_ready)
    pub.publish(command_open)
    rospy.loginfo("Opening the gripper is successful")
    rospy.loginfo("==================================")
    rospy.sleep(0.5)

    # Move the robot to a predefined position


    # Grasp an object (val (0-255) 0: full open, 255: full close)
    command_grasp = gripper_grasp(120, command_gripper_ready)
    pub.publish(command_grasp)
    rospy.loginfo("The gripper grasped an object")
    rospy.sleep(0.5)

    # Move the robot to a predefined position
    

    # Open the gripper
    command_open = gripper_open(command_gripper_ready)
    pub.publish(command_open)
    rospy.loginfo("Opening the gripper is successful")
    rospy.loginfo("==================================")
    rospy.sleep(1)

    while not rospy.is_shutdown():

        #command = genCommand(askForCommand(command), command)
        
        #pub.publish(command)

        

        rospy.sleep(1)
        rospy.loginfo("test")

        # Grasping code
        
        

if __name__ == '__main__':
    publisher()