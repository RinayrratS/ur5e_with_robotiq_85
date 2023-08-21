#! /usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


# Instantiate a RobotCommander object. Provides information such as the robot’s kinematic model and the robot’s current joint states
robot = moveit_commander.RobotCommander()
# Instantiate a PlanningSceneInterface object. This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
scene = moveit_commander.PlanningSceneInterface()



arm_group = moveit_commander.MoveGroupCommander("arm")
# Put the arm in the start position
arm_group.set_max_velocity_scaling_factor(0.5)  # Adjust the value as needed
arm_group.set_named_target("home")
plan1 = arm_group.go()

gripper_group = moveit_commander.MoveGroupCommander("gripper")
# Put the gripper in the start position
gripper_group.set_max_velocity_scaling_factor(0.5)  # Adjust the value as needed
gripper_group.set_named_target("open")
plan2 = gripper_group.go()



# To move in Rviz
display_trajectory_publisher = rospy.Publisher(
            "/arm_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
display_trajectory_publisher = rospy.Publisher(
            "/gripper_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )



# We can get the name of the reference frame for this robot:
planning_frame = arm_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = arm_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")


# < putting arm grasping position with Carthesian Paths > -> (?) don't know what it is
def generate_cartesian_path():
    waypoints = []
    scale = 0.1

    wpose = arm_group.get_current_pose().pose

    print("============ Printing robot position")
    print(wpose.position.x, wpose.position.y, wpose.position.z)
    print("")

    wpose.position.z -= scale * 0.1
    wpose.position.y += scale * 0.2     # 0.5 makes error
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Compute Cartesian path
    (plan, fraction) = arm_group.compute_cartesian_path(
        waypoints, 0.01, 0.0
    )

    return plan, fraction
# Call the function and receive the result
generated_plan, generated_fraction = generate_cartesian_path()


# Now you can use generated_plan and generated_fraction as needed
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(generated_plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

arm_group.execute(generated_plan, wait=True)



arm_group.set_named_target("up")
pose_up = arm_group.go()
# Create a Box next to arm
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.5
box_pose.pose.position.z = 0.5   # above the panda_hand frame
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))




# < putting arm grasping position with joint angle >
joint_goal = arm_group.get_current_joint_values()
joint_goal[0] = math.radians(90)
joint_goal[1] = math.radians(90)
joint_goal[2] = math.radians(90)
joint_goal[3] = math.radians(90)
joint_goal[4] = math.radians(90)
joint_goal[5] =math.radians(90)
arm_group.go(joint_goal, wait=True)
arm_group.stop()


# # < putting arm grasping position with joint position > -> (?) too hard to measure
# pose_target = geometry_msgs.msg.Pose()
# pose_target.orientation.w = 1.0
# # pose_target.orientation.x = 0
# # pose_target.orientation.y = 0
# # pose_target.orientation.z = 0
# pose_target.position.x = 5.0
# pose_target.position.y = -5.0
# pose_target.position.z = 1.0
# arm_group.set_pose_target(pose_target)

# # `go()` returns a boolean indicating whether the planning and execution was successful.
# success = arm_group.go(wait=True)
# # Calling `stop()` ensures that there is no residual movement
# arm_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets().
# arm_group.clear_pose_targets()




def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True



def go_to_pose_goal():
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.6

    arm_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    arm_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = arm_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
result=go_to_pose_goal()


def check_box_state(box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

# 함수 호출
result = check_box_state()

# grasping_group = "arm"
# touch_links = robot.get_link_names(group=grasping_group)
# scene.attach_box(eef_link, box_name, touch_links=touch_links)

# scene.remove_attached_object(eef_link, name=box_name)
# scene.remove_world_object(box_name)


rospy.sleep(5)
moveit_commander.roscpp_shutdown()


# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html