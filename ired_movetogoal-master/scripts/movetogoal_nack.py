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
from collections import deque
import numpy as np

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [3, 2],
    [0, 0],
    [2, 0],
    [0, 0]
]

# Starting position in the grid
start_pos = (3, 1)

# Directions for 8-directional movement
directions = [
    (-1, 0), (1, 0), (0, -1), (0, 1),  # up, down, left, right
    (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonals
]

# Cell size in meters
cell_size = 1.0  # approximately 1x1 meter grid cells

def convert_to_real_world_cord(x, y):
    """Convert grid coordinates to real-world coordinates."""
    return -1 * x + 3, -1 * y + 1  # Adjust to starting reference (0,0)

class moveBaseAction:
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
        print("Move to %f, %f, %f ->" % (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.orientation.z
        ))
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            return True
        else:
            print(" Fail")
            self.move_base_action.cancel_goal()
            return False

def flood_fill(start, targets):
    """Perform a BFS flood fill to reach all target cells."""
    queue = deque([start])
    visited = {start}
    path = []

    while queue:
        current = queue.popleft()
        path.append(current)

        if current in targets:
            targets.remove(current)
            if not targets:
                break

        for dx, dy in directions:
            new_x, new_y = current[0] + dx, current[1] + dy
            new_pos = (new_x, new_y)

            if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y] != 1 and new_pos not in visited:
                queue.append(new_pos)
                visited.add(new_pos)

    return path

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

    return lower[:-1] + upper[:-1]

def convex_hull_path(points):
    """Calculate the convex hull for a set of points using Graham's scan."""
    return graham_scan(points)

def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # Initialize the move_base action interface
    mba = moveBaseAction()

    # Set initial moves
    initial_moves = [(3, 0), (2, 0)]
    for grid_cell in initial_moves:
        x, y = convert_to_real_world_cord(*grid_cell)
        theta = 0.0
        if mba.moveToPoint(x, y, theta):
            rospy.loginfo(f"Reached initial grid cell {grid_cell} at ({x:.2f}, {y:.2f})")
        else:
            rospy.logwarn(f"Failed to reach initial grid cell {grid_cell}")
        rospy.sleep(1)

    # Choose navigation method
    navigation_method = "convex_hull"  # Options: "convex_hull", "flood_fill"

    if navigation_method == "flood_fill":
        targets = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]
        path = flood_fill(start_pos, targets)
        for grid_cell in path:
            x, y = convert_to_real_world_cord(*grid_cell)
            if mba.moveToPoint(x, y, 0.0):
                rospy.loginfo(f"Reached flood fill cell {grid_cell} at ({x:.2f}, {y:.2f})")
            else:
                rospy.logwarn(f"Failed to reach flood fill cell {grid_cell}")
            rospy.sleep(1)

    elif navigation_method == "convex_hull":
        target_points = [convert_to_real_world_cord(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]
        
        if len(target_points) >= 3:
            hull_path = convex_hull_path(target_points)
        else:
            hull_path = target_points

        for point in hull_path:
            x, y = point
            if mba.moveToPoint(x, y, 0.0):
                rospy.loginfo(f"Reached convex hull point at ({x:.2f}, {y:.2f})")
            else:
                rospy.logwarn(f"Failed to reach convex hull point at ({x:.2f}, {y:.2f})")
            rospy.sleep(1)

    # End move at the final destination
    end_move = [(2, 0), (3, 0), (3, 1)]
    for grid_cell in end_move:
        x, y = convert_to_real_world_cord(*grid_cell)
        if mba.moveToPoint(x, y, 0.0):
            rospy.loginfo(f"Reached the end move position at ({x:.2f}, {y:.2f})")
            rospy.sleep(5)
        else:
            rospy.logwarn("Failed to reach the end move position.")
        rospy.sleep(1)

    rospy.loginfo("Path traversal and end move completed.")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
