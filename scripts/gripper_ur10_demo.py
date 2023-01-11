#!/usr/bin/env python3

import sys
import rospy
import copy, math

# gripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from six.moves import input

# UR10
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, DisplayTrajectory 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

GROUP_NAME_ARM = "manipulator"
FIXED_FRAME = 'world'

JOINT_HOME = (math.radians(107.18), math.radians(-73.03), math.radians(120.64), math.radians(-138.37), math.radians(-90.02), math.radians(288.48))

class UR10_move():

    def __init__(self):
        roscpp_initialize(sys.argv)        
        #rospy.init_node('ur10_move', anonymous=True)

        self.scene = PlanningSceneInterface()
        self.robot_cmd = RobotCommander()

        self.robot_arm = MoveGroupCommander(GROUP_NAME_ARM)
        #self.robot_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER) # adding gripper
        self.robot_arm.set_goal_orientation_tolerance(0.005)
        self.robot_arm.set_planning_time(5)
        self.robot_arm.set_num_planning_attempts(5)

        rospy.sleep(2)
        # Allow replanning to increase the odds of a solution
        self.robot_arm.allow_replanning(True) 
        
    def display_trajectory(self):
        display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        DisplayTrajectory,
        queue_size=20,
        )
        
        # We can get the name of the reference frame for this robot:
        planning_frame = self.robot_arm.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.robot_arm.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot_cmd.get_group_names()
        print("============ Available Planning Groups:", self.robot_cmd.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot_cmd.get_current_state())
        print("")           

    def move_home(self, joint_home):
          
        self.robot_arm.set_named_target("home")  # go to goal state. tool exchange state
        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(107.18)
        joint_goal[1] = math.radians(-73.03)
        joint_goal[2] = math.radians(120.64)
        joint_goal[3] = math.radians(-138.37)
        joint_goal[4] = math.radians(-90.02)
        joint_goal[5] = math.radians(288.48)
        
        self.robot_arm.go(joint_goal, wait=True)
        print("====== move plan go to home (tool exchange) ======") 
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()       
        rospy.sleep(1)
        robot_state = self.robot_arm.get_current_pose()
        robot_angle = self.robot_arm.get_current_joint_values()

        print(robot_state)
        print(robot_angle)

        if joint_home != joint_goal:
            return True
        return False

    def poses(self):

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(107.18)
        joint_goal[1] = math.radians(-73.03)
        joint_goal[2] = math.radians(120.64)
        joint_goal[3] = math.radians(-138.37)
        joint_goal[4] = math.radians(-90.02)
        joint_goal[5] = math.radians(288.48)
        self.robot_arm.remember_joint_values("waypoint_1", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(82.75)
        joint_goal[1] = math.radians(-96.58)
        joint_goal[2] = math.radians(116.78)
        joint_goal[3] = math.radians(-110.76)
        joint_goal[4] = math.radians(-89.46)
        joint_goal[5] = math.radians(264.15)
        self.robot_arm.remember_joint_values("waypoint_2", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(104.23)
        joint_goal[1] = math.radians(-86.07)
        joint_goal[2] = math.radians(112.98)
        joint_goal[3] = math.radians(-116.54)
        joint_goal[4] = math.radians(-89.47)
        joint_goal[5] = math.radians(196.48)
        self.robot_arm.remember_joint_values("waypoint_3", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(102.29)
        joint_goal[1] = math.radians(-108.76)
        joint_goal[2] = math.radians(119.65)
        joint_goal[3] = math.radians(-115.98)
        joint_goal[4] = math.radians(-78.39)
        joint_goal[5] = math.radians(183.96)
        self.robot_arm.remember_joint_values("waypoint_4", joint_goal)  # go to goal state.

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(89.44)
        joint_goal[1] = math.radians(-117.20)
        joint_goal[2] = math.radians(131.90)
        joint_goal[3] = math.radians(-111.14)
        joint_goal[4] = math.radians(-79.60)
        joint_goal[5] = math.radians(125.52)
        self.robot_arm.remember_joint_values("waypoint_5", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(73.40)
        joint_goal[1] = math.radians(-102.99)
        joint_goal[2] = math.radians(124.24)
        joint_goal[3] = math.radians(-116.94)
        joint_goal[4] = math.radians(-85.71)
        joint_goal[5] = math.radians(27.60)
        self.robot_arm.remember_joint_values("waypoint_6", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.06)
        joint_goal[1] = math.radians(-81.44)
        joint_goal[2] = math.radians(121.59)
        joint_goal[3] = math.radians(-130.40)
        joint_goal[4] = math.radians(-91.40)
        joint_goal[5] = math.radians(-10.61)
        self.robot_arm.remember_joint_values("waypoint_7", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.06)
        joint_goal[1] = math.radians(-81.43)
        joint_goal[2] = math.radians(121.61)
        joint_goal[3] = math.radians(-130.39)
        joint_goal[4] = math.radians(-91.40)
        joint_goal[5] = math.radians(-99.27)
        self.robot_arm.remember_joint_values("waypoint_8", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.06)
        joint_goal[1] = math.radians(-81.44)
        joint_goal[2] = math.radians(121.61)
        joint_goal[3] = math.radians(-130.39)
        joint_goal[4] = math.radians(-91.40)
        joint_goal[5] = math.radians(-62.46)
        self.robot_arm.remember_joint_values("waypoint_9", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.06)
        joint_goal[1] = math.radians(-81.43)
        joint_goal[2] = math.radians(121.65)
        joint_goal[3] = math.radians(-130.39)
        joint_goal[4] = math.radians(-91.40)
        joint_goal[5] = math.radians(-122.34)
        self.robot_arm.remember_joint_values("waypoint_10", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.06)
        joint_goal[1] = math.radians(-81.43)
        joint_goal[2] = math.radians(121.65)
        joint_goal[3] = math.radians(-130.39)
        joint_goal[4] = math.radians(-91.38)
        joint_goal[5] = math.radians(-76.93)
        self.robot_arm.remember_joint_values("waypoint_11", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(77.39)
        joint_goal[1] = math.radians(-81.47)
        joint_goal[2] = math.radians(106.88)
        joint_goal[3] = math.radians(-145.81)
        joint_goal[4] = math.radians(-81.58)
        joint_goal[5] = math.radians(-39.26)
        self.robot_arm.remember_joint_values("waypoint_12", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(69.13)
        joint_goal[1] = math.radians(-78.21)
        joint_goal[2] = math.radians(104.03)
        joint_goal[3] = math.radians(-145.73)
        joint_goal[4] = math.radians(-54.59)
        joint_goal[5] = math.radians(65.28)
        self.robot_arm.remember_joint_values("waypoint_13", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(62.52)
        joint_goal[1] = math.radians(-83.01)
        joint_goal[2] = math.radians(113.00)
        joint_goal[3] = math.radians(-128.22)
        joint_goal[4] = math.radians(-98.54)
        joint_goal[5] = math.radians(145.16)
        self.robot_arm.remember_joint_values("waypoint_14", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(54.28)
        joint_goal[1] = math.radians(-102.20)
        joint_goal[2] = math.radians(123.17)
        joint_goal[3] = math.radians(-97.42)
        joint_goal[4] = math.radians(-103.06)
        joint_goal[5] = math.radians(192.49)
        self.robot_arm.remember_joint_values("waypoint_15", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(64.84)
        joint_goal[1] = math.radians(-104.14)
        joint_goal[2] = math.radians(106.29)
        joint_goal[3] = math.radians(-60.70)
        joint_goal[4] = math.radians(-92.76)
        joint_goal[5] = math.radians(208.60)
        self.robot_arm.remember_joint_values("waypoint_16", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(84.03)
        joint_goal[1] = math.radians(-104.17)
        joint_goal[2] = math.radians(107.47)
        joint_goal[3] = math.radians(-53.69)
        joint_goal[4] = math.radians(-78.20)
        joint_goal[5] = math.radians(273.71)
        self.robot_arm.remember_joint_values("waypoint_17", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(95.15)
        joint_goal[1] = math.radians(-104.07)
        joint_goal[2] = math.radians(127.98)
        joint_goal[3] = math.radians(-92.42)
        joint_goal[4] = math.radians(-56.90)
        joint_goal[5] = math.radians(342.30)
        self.robot_arm.remember_joint_values("waypoint_18", joint_goal)  # go to goal state. Ready to grip

        joint_goal = self.robot_arm.get_current_joint_values()
        joint_goal[0] = math.radians(100.31)
        joint_goal[1] = math.radians(-80.08)
        joint_goal[2] = math.radians(120.22)
        joint_goal[3] = math.radians(-130.05)
        joint_goal[4] = math.radians(-77.96)
        joint_goal[5] = math.radians(350.85)
        self.robot_arm.remember_joint_values("waypoint_19", joint_goal)  # go to goal state. Ready to grip

        robot_state = self.robot_arm.get_current_pose()
        joint_goal = self.robot_arm.get_current_joint_values()
        print(robot_state)
        print(joint_goal)
        goal_names = self.robot_arm.get_remembered_joint_values()
        print(f'goal_names = {goal_names}')
        return goal_names

    def move_grip(self):
        joint_goal = self.poses()
        goal = joint_goal.get('waypoint_1')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 1 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_2')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 2 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_3')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 3 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_4')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 4 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_5')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 5 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_6')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 6 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_7')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 7 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_8')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 8 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_9')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 9 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_10')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 10 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_11')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 11 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_12')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 12 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_13')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 13 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_14')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 14 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_15')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 15 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_16')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 16 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_17')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 17 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_18')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 18 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)

        goal = joint_goal.get('waypoint_19')
        print(f'joint_goal = {goal}')

        self.robot_arm.go(goal, wait=True)
        print("====== Waypoint 19 done ======")
        # Calling ``stop()`` ensures that there is no residual movement
        self.robot_arm.stop()
        #rospy.sleep(3)


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
    # Initialize the robot
    robot = UR10_move()
    robot.__init__()
    robot.display_trajectory()

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
    robot.move_home(JOINT_HOME)

    # Grasp an object (val (0-255) 0: full open, 255: full close)
    command_grasp = gripper_grasp(140, command_gripper_ready)
    pub.publish(command_grasp)
    rospy.loginfo("The gripper grasped an object")
    rospy.sleep(0.5)

    # Move the robot to a predefined position
    robot.move_grip()
    robot.move_home(JOINT_HOME)

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