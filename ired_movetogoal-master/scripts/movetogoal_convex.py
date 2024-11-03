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

def orientation(p, q, r):
    """Return orientation of the triplet (p, q, r).
    - 0 if p, q and r are collinear
    - 1 if clockwise
    - -1 if counterclockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        # Handle collinear case by returning 0
        # Optionally sort or check distance to select the next point
        return 0
    return 1 if val > 0 else -1


def gift_wrapping_hull(points):
    """Gift Wrapping Algorithm (Jarvis March) to find the convex hull."""
    n = len(points)
    if n < 3:
        return points  # Convex hull is undefined for fewer than 3 points

    hull = []
    
    # Start with the leftmost point
    leftmost = np.argmin(points[:, 0])
    point_on_hull = leftmost
    
    while True:
        hull.append(points[point_on_hull])
        endpoint = (point_on_hull + 1) % n
        
        for j in range(n):
            # Check if 'j' is more counterclockwise than 'endpoint'
            if orientation(points[point_on_hull], points[j], points[endpoint]) == -1:
                endpoint = j
            elif orientation(points[point_on_hull], points[j], points[endpoint]) == 0:
                # If collinear, choose the farther one
                dist_endpoint = np.linalg.norm(points[point_on_hull] - points[endpoint])
                dist_j = np.linalg.norm(points[point_on_hull] - points[j])
                if dist_j > dist_endpoint:
                    endpoint = j
        
        point_on_hull = endpoint
        if point_on_hull == leftmost:
            break
    
    return np.array(hull)

def fixed_five_point_hull(points, fixed=True):
    """Get the convex hull points, optionally fixed to 5 points."""
    hull_points = gift_wrapping_hull(points)

    if fixed:
        # If hull has more than 5 points, reduce to 5 points
        if len(hull_points) > 5:
            selected_points = hull_points[::len(hull_points) // 5][:5]  # Select every nth point
        elif len(hull_points) < 5:
            # If fewer than 5, add points from the original set that aren’t in the hull
            extra_points = []
            for point in points:
                if len(hull_points) + len(extra_points) >= 5:
                    break
                if not any(np.array_equal(point, hp) for hp in hull_points):
                    extra_points.append(point)
            selected_points = np.vstack([hull_points, extra_points[:5 - len(hull_points)]])
        else:
            selected_points = hull_points  # Exactly 5 points
    else:
        selected_points = hull_points  # Do not fix the number of points

    # Sort selected points in counterclockwise order for consistent plotting
    center = selected_points.mean(axis=0)
    angles = np.arctan2(selected_points[:, 1] - center[1], selected_points[:, 0] - center[0])
    selected_points = selected_points[np.argsort(angles)]

    return selected_points

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
    target_points = np.array([convert_grid_to_real_world(cell) for cell in target_cells])

    # Compute the convex hull for target locations
    if len(target_points) >= 3:  # Convex hull requires at least 3 points
        hull_path = fixed_five_point_hull(target_points, fixed=True)  # Use fixed 5-point hull option
        rospy.loginfo(f"Convex hull computed with {len(hull_path)} boundary points.")
    else:
        hull_path = target_points  # If less than 3 points, just use available points
        rospy.loginfo(f"Only {len(hull_path)} target points, no convex hull needed.")

    # Follow the convex hull path
    for point in hull_path:
        x, y = point
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached convex hull point at ({x:.2f}, {y:.2f})")
            mba.move_base_action.cancel_goal()  # Cancel the goal to avoid waiting for completion
            rospy.sleep(5)  # stop for 5 seconds at each point on the convex hull 
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

    rospy.loginfo("Path traversal and convex hull navigation completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
