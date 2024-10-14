from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
rclpy.init()

nav = BasicNavigator()

# Get initial pose
init_pose = PoseStamped()
init_pose.header.frame_id = 'odom'
init_pose.pose.position.x = 1.84
init_pose.pose.position.y = -9.05
init_pose.pose.position.z = 0.1
init_pose.pose.orientation.z = 3.14

nav.setInitialPose(init_pose)
nav.waitUntilNav2Active()


# Get goal poses
raw_poses = [[-0.12, -2.35, 3.14],
             [1.86, 2.56, 0.97],
             [-3.84, 2.64, 2.78]]
poses = []

for raw_pose in raw_poses:
    pose = PoseStamped()
    pose.header.frame_id = 'odom'
    pose.pose.position.x = raw_pose[0]
    pose.pose.position.y = raw_pose[1]
    pose.pose.orientation.z = raw_pose[2]
    poses.append(pose)

# Go through each goal pose
nav.goToPose(poses[0])
while not nav.isTaskComplete():
    feedback = nav.getFeedback()

nav.goToPose(poses[1])
while not nav.isTaskComplete():
    feedback = nav.getFeedback()

nav.goToPose(poses[2])
while not nav.isTaskComplete():
    feedback = nav.getFeedback()