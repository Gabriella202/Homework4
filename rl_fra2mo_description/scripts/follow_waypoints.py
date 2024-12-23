#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml 
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import math

yaml_path = get_package_share_directory('rl_fra2mo_description')
yaml_file = os.path.join(yaml_path, "config", "waypoints.yaml")

with open(yaml_file, 'r') as file:
    waypoints = yaml.safe_load(file)

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Crea un subscriber per la posa
    current_pose = None
    def pose_callback(msg):
        nonlocal current_pose
        current_pose = msg.pose.pose  # Nota: pose.pose perché è un PoseWithCovarianceStamped

    pose_sub = navigator.create_subscription(
        PoseWithCovarianceStamped,
        '/pose',
        pose_callback,
        10)

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    # Ordine desiderato dei goal
    goal_order = ["goal_3", "goal_4", "goal_2", "goal_1"]

    # Riordina i goal in base a goal_order
    ordered_goals = [goal for name in goal_order for goal in waypoints["waypoints"] if goal["goal"] == name]

    # Crea una lista di PoseStamped dai goal ordinati
    goal_poses = list(map(create_pose, ordered_goals))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    current_waypoint = -1
    printed_executing = False
    
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback:
            # Se siamo passati a un nuovo waypoint
            if current_waypoint != feedback.current_waypoint:
                current_waypoint = feedback.current_waypoint
                printed_executing = False
                # Stampa del completamento per i waypoint dopo il primo
                if current_waypoint > 0:
                    print(f"\nCompleted waypoint {current_waypoint}")
                    if current_pose:
                        x = current_pose.position.x
                        y = current_pose.position.y
                        z = current_pose.position.z
                        orientation_q = current_pose.orientation
                        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                        roll, pitch, yaw = euler_from_quaternion(orientation_list)
                        roll_deg = math.degrees(roll)
                        pitch_deg = math.degrees(pitch)
                        yaw_deg = math.degrees(yaw)
                        print(f"\nRobot pose:")
                        print(f"Position -> X: {x:.2f} m, Y: {y:.2f} m, Z: {z:.2f} m")
                        print(f"Rotation -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
                    else:
                        print("Warning: No pose data available")
            
            # Stampa "Executing" solo una volta per waypoint
            if not printed_executing:
                print(f'Executing waypoint {feedback.current_waypoint + 1}/{len(goal_poses)}')
                printed_executing = True

            now = navigator.get_clock().now()
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        if current_pose:
            x = current_pose.position.x
            y = current_pose.position.y
            z = current_pose.position.z
            orientation_q = current_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            print(f"\nFinal robot pose:")
            print(f"Position -> X: {x:.2f} m, Y: {y:.2f} m, Z: {z:.2f} m")
            print(f"Rotation -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")
        else:
            print("Warning: No pose data available for final position")
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    exit(0)

if __name__ == '__main__':
    main()