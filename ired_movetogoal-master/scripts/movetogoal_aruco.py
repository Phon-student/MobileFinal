#!/usr/bin/python3
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
from collections import deque
import numpy as np

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [2, 0],
    [0, 0],
    [0, 0],
    [0, 3],
]

# Starting position for the robot in the grid
start_pos = (3, 1)

# Cell size in meters (considering the variation)
cell_size = 1.0  # approximately 1x1 meter grid cells

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return grid_pos[0] * cell_size, grid_pos[1] * cell_size

def calculate_convex_hull(points):
    """Calculate the convex hull of a set of points."""
    points = np.array(points)
    # Sort the points lexicographically (by x, then by y)
    points = points[np.argsort(points[:, 0])]
    
    # Build the lower hull
    lower = []
    for p in points:
        while len(lower) >= 2 and np.cross(lower[-1] - lower[-2], p - lower[-1]) <= 0:
            lower.pop()
        lower.append(p)

    # Build the upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and np.cross(upper[-1] - upper[-2], p - upper[-1]) <= 0:
            upper.pop()
        upper.append(p)

    # Remove the last point of each half because it's repeated at the beginning of the other half
    return np.array(lower[:-1] + upper[:-1])

class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, theta):
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        return goal

    def moveToPoint(self, x, y, theta):
        target_point = self.createGoal(x, y, theta)
        self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        rospy.loginfo("Move to %f, %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.z
        ))
        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo(" Complete")
            return True
        else:
            rospy.logwarn(" Fail")
            self.move_base_action.cancel_goal()
            return False

# ArUco callback
def aruco_callback(data):
    if data.detected:
        rospy.loginfo(f"Detected QR Code: {data.qr_code} at position: {data.position}")

def pose_callback(pose_with_covariance):
    pose = pose_with_covariance.pose.pose
    rospy.loginfo("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))

def move_base_status_callback(status):
    pass

def move_base_result_callback(result):
    pass

# Main program
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)
    rospy.Subscriber('/aruco/detected', YourArucoMsgType, aruco_callback)  # Replace with your actual message type

    # Initialize the move_base action interface
    mba = moveBaseAction()

    # Find all target locations
    targets = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]
    
    # Calculate the convex hull of the target positions
    hull = calculate_convex_hull(np.array(targets))
    
    # Follow the convex hull path
    for grid_cell in hull:
        x, y = convert_grid_to_real_world(tuple(grid_cell))
        theta = 0.0  # Assuming no rotation needed initially; adjust if required
        if mba.moveToPoint(x, y, theta):
            rospy.loginfo(f"Reached convex hull point {grid_cell} at ({x:.2f}, {y:.2f})")
            rospy.sleep(5)  # Allow some time between movements
        else:
            rospy.logwarn(f"Failed to reach convex hull point {grid_cell}")
        rospy.sleep(1)  # Allow some time between movements

    # End move at the final destination
    end_move = [(2, 0), (3, 0), (3, 1)]  # Customize the end move position as desired
    for grid_cell in end_move:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y, 0.0):
            rospy.loginfo(f"Reached the end move position at ({x:.2f}, {y:.2f})")
            rospy.sleep(5)
        else:
            rospy.logwarn("Failed to reach the end move position.")
        rospy.sleep(1)  # Allow some time between movements

    rospy.loginfo("Path traversal and end move completed.")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
