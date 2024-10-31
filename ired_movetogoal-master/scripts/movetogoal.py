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

# Grid configuration (0: free, 1: obstacle, 2: target, 3: exit)
grid = [
    [2, 0,],
    [0, 0,],
    [0, 0,],
    [0, 3,],
]

# Starting position for the robot in the grid
start_pos = (3, 1)

# Directions for 8-directional movement (up, down, left, right, and diagonals)
directions = [
    (-1, 0), (1, 0), (0, -1), (0, 1),  # up, down, left, right
    (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonals
]

# Cell size in meters (considering the variation)
cell_size = 1.0  # approximately 1x1 meter grid cells

def is_valid_move(x, y):
    rows, cols = len(grid), len(grid[0])
    return 0 <= x < rows and 0 <= y < cols and grid[x][y] != 1

def flood_fill(start, targets):
    # Perform a BFS flood fill to reach all target cells
    queue = deque([start])
    visited = {start}
    path = []

    while queue:
        current = queue.popleft()
        path.append(current)

        # If current cell is a target, remove it from the list
        if current in targets:
            targets.remove(current)
            if not targets:
                break  # Exit if all targets are reached

        for dx, dy in directions:
            new_x, new_y = current[0] + dx, current[1] + dy
            new_pos = (new_x, new_y)

            if is_valid_move(new_x, new_y) and new_pos not in visited:
                queue.append(new_pos)
                visited.add(new_pos)

    return path

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return grid_pos[0] * cell_size, grid_pos[1] * cell_size

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

# Main program
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # Initialize the move_base action interface
    mba = moveBaseAction()

    # Set initial moves (for example: [(row, col), ...])
    initial_moves = [(3, 0), (2, 0)]  # Customize the initial positions as needed

    # Execute initial moves
    for grid_cell in initial_moves:
        x, y = convert_grid_to_real_world(grid_cell)
        theta = 0.0  # Adjust orientation if necessary
        if mba.moveToPoint(x, y, theta):
            rospy.loginfo(f"Reached initial grid cell {grid_cell} at ({x:.2f}, {y:.2f})")
        else:
            rospy.logwarn(f"Failed to reach initial grid cell {grid_cell}")
        rospy.sleep(1)  # Small delay between moves

    # Find all target locations
    targets = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]
    # Perform flood fill to generate the path
    path = flood_fill(start_pos, targets)

    # Follow the path
    for grid_cell in path:
        x, y = convert_grid_to_real_world(grid_cell)
        theta = 0.0  # Assuming no rotation needed initially; adjust if required
        if mba.moveToPoint(x, y, theta):
            rospy.loginfo(f"Reached grid cell {grid_cell} at ({x:.2f}, {y:.2f})")
            ospy.sleep(5)  # Allow some time between movements
        else:
            rospy.logwarn(f"Failed to reach grid cell {grid_cell}")
        rospy.sleep(1)  # Allow some time between movements

    rospy.loginfo("Path traversal completed.")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
