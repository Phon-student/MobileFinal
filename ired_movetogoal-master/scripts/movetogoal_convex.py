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
from collections import deque

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [0, 2],
    [2, 2],
    [0, 2],
    [0, 3],
]

# Starting position for the robot in the grid
start_pos = (3, 1)

# Cell size in meters (considering the variation)
cell_size = 1.0  # approximately 1x1 meter grid cells

current_pose = None  # Global variable to store the current pose

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return -1 * grid_pos[0] * cell_size + 3, -1 * grid_pos[1] * cell_size + 1  # start from (0, 0) in the grid map converted to real-world coordinates of cartesian plane

def pose_callback(pose_with_covariance):
    """Callback to update current robot pose."""
    global current_pose
    pose = pose_with_covariance.pose.pose
    current_pose = (pose.position.x, pose.position.y, pose.orientation.z)  # You may want to use a full orientation quaternion for more accuracy

def move_base_status_callback(status):
    """Callback for move base status updates."""
    pass

def move_base_result_callback(result):
    """Callback for move base result updates."""
    pass

class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)  # Fixed orientation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(quat[0], quat[1], quat[2], quat[3]))

        return goal

    def moveToPoint(self, x, y):
        target_point = self.createGoal(x, y)
        self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        print("Move to %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False

def cross(o, a, b):
    """Calculate the cross product of vectors OA and OB."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def graham_scan(points):
    """Calculate the convex hull using Graham's scan."""
    points = sorted(set(points))
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    hull = lower[:-1] + upper[:-1]  # Combine lower and upper hull

    # Interpolate to create new points if fewer than 5
    if len(hull) < 5:
        while len(hull) < 5:
            for i in range(len(hull) - 1):
                if len(hull) >= 5:
                    break
                # Insert midpoint between consecutive points
                midpoint = ((hull[i][0] + hull[i + 1][0]) / 2, (hull[i][1] + hull[i + 1][1]) / 2)
                hull.insert(i + 1, midpoint)

    return hull

# Main program
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # Initialize the move_base action interface
    mba = moveBaseAction()

    # Set initial moves
    initial_moves = [(3, 0), (2, 0)]  # Customize the initial positions as needed

    # Execute initial moves
    for grid_cell in initial_moves:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached initial grid cell {grid_cell} at ({x:.2f}, {y:.2f})")
        else:
            rospy.logwarn(f"Failed to reach initial grid cell {grid_cell}")
        rospy.sleep(1)  # Small delay between moves

    # Find all target locations in real-world coordinates
    target_cells = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]
    target_points = [convert_grid_to_real_world(cell) for cell in target_cells]

    # Compute the convex hull for target locations
    if len(target_points) >= 3:  # Convex hull requires at least 3 points
        hull_path, boundary_count = graham_scan(target_points)
        rospy.loginfo(f"Convex hull computed with {boundary_count} boundary points.")
    else:
        hull_path = target_points  # If less than 3 points, just use available points
        boundary_count = len(hull_path)
        rospy.loginfo(f"Only {boundary_count} target points, no convex hull needed.")

    # Follow the convex hull path
    for point in hull_path:
        x, y = point
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached convex hull point at ({x:.2f}, {y:.2f})")
            rospy.sleep(5)  # Allow some time between movements
        else:
            rospy.logwarn(f"Failed to reach convex hull point at ({x:.2f}, {y:.2f})")
        rospy.sleep(1)  # Allow some time between movements

    # End move at the final destination
    end_move = [(2, 0), (3, 0), (3, 1)]  # Customize the end move position as desired
    for grid_cell in end_move:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached the end move position at ({x:.2f}, {y:.2f})")
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

