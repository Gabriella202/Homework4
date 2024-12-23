#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import yaml 
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # Aggiunto quaternion_from_euler
import math
import time
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Questo è importante per le trasformazioni delle pose


class MarkerNavigator(Node):
    def __init__(self):
        super().__init__('marker_navigator')
        self.navigator = BasicNavigator()
        self.marker_pose = None
        self.marker_detected = False

        # Inizializza il buffer e listener di tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to ArUco marker pose
        self.marker_sub = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.marker_callback,
            10
        )

        # Subscribe to robot pose
        self.pose_sub = self.navigator.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        self.current_pose = None
        

    def marker_callback(self, msg):
     #   self.marker_pose = msg.pose
     #   self.marker_detected = True
        try:
            # Aspetta che la trasformazione sia disponibile
            self.tf_buffer.can_transform('map', 
                                       msg.header.frame_id,  # camera_link
                                       rclpy.time.Time(),
                                       timeout=rclpy.duration.Duration(seconds=1.0))

            # Prima applica la correzione dell'orientamento
            (roll, pitch, yaw) = euler_from_quaternion([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ])
            
            # Applica la correzione dell'orientamento 
            roll_corrected = pitch + math.pi/2
            pitch_corrected = pitch + math.pi/2
            yaw_corrected = yaw + math.pi
            
            q_corrected = quaternion_from_euler(roll_corrected, pitch_corrected, yaw_corrected)
            
            # Applica il quaternione corretto al messaggio
            msg.pose.orientation.x = q_corrected[0]
            msg.pose.orientation.y = q_corrected[1]
            msg.pose.orientation.z = q_corrected[2]
            msg.pose.orientation.w = q_corrected[3]
                                       
            
            # Trasforma la posa dal frame camera_link al frame map
            pose_map = self.tf_buffer.transform(msg, 'map')
            
            # Ora pose_map contiene la posa del marker nel frame map
            self.marker_pose = pose_map.pose
            self.marker_detected = True
            
            # Stampa per debug
            print(f"Marker pose in map frame:")
            print(f"Position: x={pose_map.pose.position.x:.2f}, "
                  f"y={pose_map.pose.position.y:.2f}, "
                  f"z={pose_map.pose.position.z:.2f}")
            
        except Exception as e:
            self.get_logger().error(f'Failed to transform pose: {str(e)}')
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def create_pose(self, transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    def navigate(self):
        # Load waypoints
        yaml_path = get_package_share_directory('rl_fra2mo_description')
        yaml_file = os.path.join(yaml_path, "config", "detection.yaml")

        with open(yaml_file, 'r') as file:
            waypoints = yaml.safe_load(file)

        # Define goal order
        goal_order = ["goal_1", "goal_2"]
        ordered_goals = [goal for name in goal_order for goal in waypoints["waypoints"] if goal["goal"] == name]
        goal_poses = list(map(self.create_pose, ordered_goals))

        # Wait for navigation to become active
        self.navigator.waitUntilNav2Active(localizer="smoother_server")
        
        # Navigate to first goal (near obstacle 9)
        print("Navigating to obstacle 9 area...")
        self.navigator.goToPose(goal_poses[0])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(f'Distance remaining: {feedback.distance_remaining:.2f}m')
            time.sleep(0.5)

        # Check if we reached the first goal
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached observation point! Looking for ArUco marker...')
            
            # Wait for up to 10 seconds to detect the marker
            detection_timeout = 10.0
            start_time = time.time()
            
            while not self.marker_detected and (time.time() - start_time) < detection_timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.marker_detected:
                print("\nMarker detected!")
                
                # Get marker pose (already in map frame)
                x = self.marker_pose.position.x
                y = self.marker_pose.position.y
                z = self.marker_pose.position.z
                
                # Get orientation
                orientation_q = self.marker_pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                roll, pitch, yaw = euler_from_quaternion(orientation_list)
                
                print(f"\nMarker pose (in map frame):")
                print(f"Position -> X: {x:.2f} m, Y: {y:.2f} m, Z: {z:.2f} m")
                print(f"Rotation -> Roll: {math.degrees(roll):.2f}°, Pitch: {math.degrees(pitch):.2f}°, Yaw: {math.degrees(yaw):.2f}°")
            else:
                print("No marker detected within timeout!")

            # Return to starting position
            print("\nReturning to starting position...")
            self.navigator.goToPose(goal_poses[1])
            
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    print(f'Distance remaining: {feedback.distance_remaining:.2f}m')
                time.sleep(0.5)

            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                print('Successfully returned to starting position!')
            else:
                print('Failed to return to starting position')
        else:
            print('Failed to reach observation point')

def main():
    rclpy.init()
    navigator = MarkerNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()