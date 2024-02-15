import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration

from control_msgs.action import GripperCommand
import time

def send_goal(positionz, time_interval):
    rclpy.init()

    # Create a node
    node = rclpy.create_node('action_client_node')

    # Create an action client
    action_client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    # Wait for the action server to become available
    if not action_client.wait_for_server(timeout_sec=20.0):
        node.get_logger().info('Action server not available')
        return

    # Create a FollowJointTrajectory goal
    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = JointTrajectory()
    goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.positions = positionz
    point.velocities = [0.0, 0.0, 0.0, 0.0]
    point.accelerations = [0.0, 0.0, 0.0, 0.0]
    point.time_from_start = Duration(sec=5, nanosec=0)

    goal_msg.trajectory.points.append(point)

    tolerance = JointTolerance()
    tolerance.name = 'joint1'
    tolerance.position = 0.01
    tolerance.velocity = 0.01
    tolerance.acceleration = 0.01

    goal_msg.path_tolerance.append(tolerance)
    goal_msg.goal_tolerance.append(tolerance)
    goal_msg.goal_time_tolerance = Duration(sec=10, nanosec=0)

    # Send the goal
    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Goal sent successfully')
    else:
        node.get_logger().warning('Failed to send goal')

    # Destroy the action client explicitly before shutting down the node
    action_client.destroy()

    # Sleep for the specified time interval
    time.sleep(time_interval)

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

def send_gripper_goal(goalz, time_interval):
    rclpy.init()

    # Create a node
    node = rclpy.create_node('gripper_client')

    # Create an ActionClient
    action_client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')

    # Wait for the action server to become available
    action_client.wait_for_server()

    # Create a goal
    goal_msg = GripperCommand.Goal()
    goal_msg.command.position = goalz
    goal_msg.command.max_effort = 10.0

    # Set the feedback callback
    def feedback_callback(feedback):
        print(f"Received feedback: {feedback}")

    # Send the goal
    goal_handle = action_client.send_goal_async(goal_msg, feedback_callback)

    # Check if goal_handle is None
    if goal_handle is None:
        print("Failed to send the goal. Exiting.")
        return

    # Wait for the goal to finish
    while not goal_handle.done():
        rclpy.spin_once(node, timeout_sec=1.0)

    # Print the result
    result = goal_handle.result()
    if result:
        print(f"Goal finished with status: {result.status}")
        print(f"Result: {result.status}")
    else:
        print("Failed to get the result.")

    # Clean up
    action_client.destroy()  # Destroy the action client
    node.destroy_node()
    rclpy.shutdown()

    # Sleep for the specified time interval
    time.sleep(time_interval)

if __name__ == '__main__':

    gripper_pos = -0.018
    send_gripper_goal(gripper_pos, time_interval=2)

    pos = [0.0, 1.4, -0.8, -0.7]    

    send_goal(pos, time_interval=5)
    
    gripper_pos = 0.018
    send_gripper_goal(gripper_pos, time_interval=2)
    
    pos = [0.0, 0.0, 0.0, 0.0]
    send_goal(pos, time_interval=5)
    
    pos = [0.0,- 1.5, 0.9, 0.6]
    send_goal(pos, time_interval=5)
    


    
