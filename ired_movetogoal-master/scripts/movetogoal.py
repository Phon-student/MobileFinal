#!/usr/bin/python3
"""
http://wiki.ros.org/move_base
http://wiki.ros.org/actionlib
"""
import rospy
import tf.transformations
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import actionlib


def pose_callback(pose_with_covariance):
    # print(pose_with_covariance)
    pose = pose_with_covariance.pose.pose
    print("amcl_pose = {x: %f, y:%f, orientation.z:%f" % (pose.position.x, pose.position.y, pose.orientation.z))


def move_base_status_callback(status):
    pass


def move_base_result_callback(result):
    pass


class moveBaseAction():
    def __init__(self):
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))

    def createGoal(self, x, y, theta):
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
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
        print ("Move to %f, %f, %f ->" % (
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

grid = [
    [3, 2],
    [0, 0],
    [2, 0],
    [0, 0]
]

# convert list to realworld to command motor 
def convert_to_real_world_cord(x,y):
    return -1 * x + 3, -1 * y +1


def main():
    global grid
    rospy.init_node('move_to_goal', anonymous=True)
    publisher_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_base_status_callback)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, move_base_result_callback)

    mba = moveBaseAction()

    while not rospy.is_shutdown():
        # Move to target locations in the grid
        command = False  # Reset command for each loop
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == 2:  # Target found
                    
                    x, y = convert_to_real_world_cord(i, j)
                    print(f"Moving to target at: ({x}, {y})")  # Log the target location
                    success = mba.moveToPoint(x, y, 0)  # Move to the target point
                    if success:
                        print(f"Reached target at: ({x}, {y})")
                    else:
                        print(f"Failed to reach target at: ({x}, {y})")
                    time.sleep(2)  # Wait for a moment after reaching a target
                    command = True
                    break
            if command:  # Exit both loops if a command was executed
                break
        else:
            # If no target found, you can either break or loop again after a delay
            rospy.loginfo("No more targets to move to. Waiting...")
            time.sleep(5)  # Wait before checking again




if __name__ == '__main__':
    try:
        # print(convert_to_real_world_cord(2,0))
        main()
    except rospy.ROSInterruptException:
        pass
