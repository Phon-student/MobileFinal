#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib
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
    return -1 * grid_pos[0] * cell_size + 3, -1 * grid_pos[1] * cell_size + 1

def pose_callback(pose_with_covariance):
    """Callback to update current robot pose."""
    global current_pose
    pose = pose_with_covariance.pose.pose
    current_pose = (pose.position.x, pose.position.y, pose.orientation.z)

class MoveBaseAction():
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
        return self.moveToGoal(target_point)

    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        if success and state == GoalStatus.SUCCEEDED:
            return True
        else:
            self.move_base_action.cancel_goal()
            return False

def find_next_target_path(grid, start, target):
    """Find the shortest path to the next target cell."""
    queue = deque([(start, [])])
    visited = set([start])

    while queue:
        current, path = queue.popleft()
        x, y = current

        # If we've reached the target, return the path
        if current == target:
            return path + [current]

        # Explore neighboring cells
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            next_cell = (nx, ny)
            if (0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and
                next_cell not in visited and grid[nx][ny] in (0, 2, 3)):
                visited.add(next_cell)
                queue.append((next_cell, path + [current]))

    return []  # Return empty path if no path found

def move_base_status_callback(status):
    """Callback for move base status updates."""
    pass

def move_base_result_callback(result):
    """Callback for move base result updates."""
    pass

# Main program
def main():
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    # Initialize the move_base action interface
    global mba
    mba = MoveBaseAction()

    # Collect all target cells in a list
    target_cells = [(i, j) for i in range(len(grid)) for j in range(len(grid[0])) if grid[i][j] == 2]

    # Traverse targets one by one
    current_position = start_pos
    for target in target_cells:
        path_to_target = find_next_target_path(grid, current_position, target)
        if path_to_target:
            for cell in path_to_target:
                real_x, real_y = convert_grid_to_real_world(cell)
                if mba.moveToPoint(real_x, real_y):
                    rospy.loginfo(f"Reached grid cell {cell} at ({real_x:.2f}, {real_y:.2f})")
                    current_position = cell  # Update current position
                else:
                    rospy.logwarn(f"Failed to reach grid cell {cell}")
                rospy.sleep(1)  # Small delay between moves
        else:
            rospy.logwarn(f"No path found to target cell {target}")

    # End move at the final destination
    end_move = [(2, 0), (3, 0), (3, 1)]  # Customize the end move position as desired
    for grid_cell in end_move:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached the end move position at ({x:.2f}, {y:.2f})")
        else:
            rospy.logwarn("Failed to reach the end move position.")
        rospy.sleep(1)  # Allow some time between movements

    rospy.loginfo("Path traversal and target cell visits completed.")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
