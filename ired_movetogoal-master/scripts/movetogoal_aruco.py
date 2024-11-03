#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
import numpy as np
import math

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [2, 2],
    [2, 2],
    [2, 2],
    [2, 3],
]

# Starting position for the robot in the grid
start_pos = (3, 1)

# Cell size in meters (considering the variation)
cell_size = 1.0  # approximately 1x1 meter grid cells

current_pose = None  # Global variable to store the current pose
aruco_message_received = False
saved_positions = []  # List to save detected ArUco marker positions

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return -1 * grid_pos[0] * cell_size + 3, -1 * grid_pos[1] * cell_size + 1  # Start from (0, 0) in the grid map converted to real-world coordinates of Cartesian plane

def pose_callback(pose_with_covariance):
    """Callback to update current robot pose."""
    global current_pose
    pose = pose_with_covariance.pose.pose
    current_pose = (pose.position.x, pose.position.y, pose.orientation.z)

def move_base_status_callback(status):
    """Callback for move base status updates."""
    pass

def move_base_result_callback(result):
    """Callback for move base result updates."""
    pass

def aruco_position_callback(msg):
    global aruco_message_received, saved_positions
    aruco_message_received = True  # Set flag when a new message is received
    rospy.loginfo("Received ArUco positions:")
    rospy.loginfo(f"Number of markers found: {msg.number_found}")

    for i in range(msg.number_found):
        # Store the marker positions and IDs
        position = (msg.x[i], msg.y[i])
        marker_id = msg.id[i]
        saved_positions.append((marker_id, position))  # Save marker ID and position
        rospy.loginfo(f"Marker ID: {marker_id}, Position: {position}")

class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, yaw):
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)  # Orientation based on yaw
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    def moveToPoint(self, x, y, yaw):
        target_point = self.createGoal(x, y, yaw)
        return self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        rospy.loginfo("Move to %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y
        ))
        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Complete")
            return True
        else:
            rospy.logwarn("Fail")
            self.move_base_action.cancel_goal()
            return False

    def spin360(self):
        """Perform a slow 360-degree spin in place."""
        for angle in np.linspace(0, 2 * math.pi, num=36):  # 10-degree increments
            spin_goal = self.createGoal(current_pose[0], current_pose[1], angle)
            if not self.moveToGoal(spin_goal):
                rospy.logwarn("Failed to complete 360-degree spin.")
                return False
            rospy.sleep(0.5)  # Adjust as needed for slower rotation
        rospy.loginfo("Completed 360-degree spin.")
        return True

def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)
    rospy.Subscriber("/aruco_mps_explored", MPSPosition, aruco_position_callback)

    # Initialize the move_base action interface
    mba = moveBaseAction()

    # Iterate through every cell in the grid
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            # Convert grid cell to real-world coordinates
            x, y = convert_grid_to_real_world((i, j))

            # Try to move to the cell
            if mba.moveToPoint(x, y, 0):
                rospy.loginfo(f"Reached cell at ({x:.2f}, {y:.2f})")
            else:
                rospy.logwarn(f"Could not reach cell at ({x:.2f}, {y:.2f}). Spinning in current cell instead.")

            # Perform a 360-degree spin in place
            rospy.loginfo("Starting 360-degree spin in current cell...")
            mba.spin360()

            # Check if an ArUco message has been received before proceeding
            rospy.loginfo("Waiting for ArUco message before moving to the next cell...")
            while not aruco_message_received:
                rospy.sleep(0.1)  # Polling interval

            # Reset the flag for the next iteration
            aruco_message_received = False  
            rospy.sleep(5)  # Optional delay after processing the ArUco message

    rospy.loginfo("Completed navigation through all grid cells.")

    # Log or process the saved positions after navigation
    rospy.loginfo("Saved ArUco Marker Positions:")
    for marker_id, position in saved_positions:
        rospy.loginfo(f"Marker ID: {marker_id}, Position: {position}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
