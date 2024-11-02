import rospy
import tf.transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib

# Grid configuration (0: free, 1: obstacle, 2: target, 3: end move)
grid = [
    [2, 0],  # Target at (0, 0)
    [0, 2],  # Target at (1, 1)
    [0, 2],  # Target at (2, 1)
    [0, 3],  # End move at (3, 1)
]

# Starting position for the robot in the grid
start_pos = (3, 1)

# Cell size in meters
cell_size = 1.0  # approximately 1x1 meter grid cells

current_pose = None  # Global variable to store the current pose

def convert_grid_to_real_world(grid_pos):
    """Convert grid coordinates to real-world coordinates."""
    return -1 * grid_pos[0] * cell_size + 3, -1 * grid_pos[1] * cell_size + 1  # Converts grid coordinates to the real-world coordinates

def pose_callback(pose_with_covariance):
    """Callback to update current robot pose."""
    global current_pose
    pose = pose_with_covariance.pose.pose
    current_pose = (pose.position.x, pose.position.y)

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

def heuristic(a, b):
    """Calculate the heuristic (Manhattan distance)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid):
    """A* algorithm implementation."""
    open_set = {start}
    came_from = {}
    
    g_score = {pos: float('inf') for row in grid for pos in enumerate(row)}
    g_score[start] = 0

    f_score = {pos: float('inf') for row in grid for pos in enumerate(row)}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = min(open_set, key=lambda pos: f_score[pos])
        
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Return reversed path

        open_set.remove(current)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Neighboring positions
            neighbor = (current[0] + dx, current[1] + dy)

            if (0 <= neighbor[0] < len(grid) and
                0 <= neighbor[1] < len(grid[0]) and
                grid[neighbor[0]][neighbor[1]] != 1):  # Check for valid and free space
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

    return []  # No path found

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
    initial_moves = [(3, 1), (3, 0)]  # Customize the initial positions as needed

    # Execute initial moves
    for grid_cell in initial_moves:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached initial grid cell {grid_cell} at ({x:.2f}, {y:.2f})")
        else:
            rospy.logwarn(f"Failed to reach initial grid cell {grid_cell}")
        rospy.sleep(1)  # Small delay between moves

    # Define target and goal position
    target_cell = (0, 1)  # Example target position
    end_move = (3, 1)  # End move position

    # Perform A* pathfinding
    path = a_star(start_pos, target_cell, grid)

    # Move along the path
    for grid_cell in path:
        x, y = convert_grid_to_real_world(grid_cell)
        if mba.moveToPoint(x, y):
            rospy.loginfo(f"Reached path point at ({x:.2f}, {y:.2f})")
            rospy.sleep(5)  # Allow some time between movements
        else:
            rospy.logwarn(f"Failed to reach path point at ({x:.2f}, {y:.2f})")
        rospy.sleep(1)  # Allow some time between movements

    # Move to end position
    if mba.moveToPoint(*convert_grid_to_real_world(end_move)):
        rospy.loginfo(f"Reached end move position at {end_move}")
    else:
        rospy.logwarn("Failed to reach end move position.")

    rospy.loginfo("Path traversal and end move completed.")
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
