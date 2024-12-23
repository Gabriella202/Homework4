#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
import yaml
import time
import os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import math
import sys

class ImprovedNavigator(Node):
    def __init__(self, config_choice):
        super().__init__('improved_navigator')
        
        # Define configurations
        self.configurations = {
            '1': {  # Conservative
                'slam_file': 'slam_config1.yaml',
                'explore_file': 'explore_config1.yaml',
                'description': 'Conservative - High precision, careful movement'
            },
            '2': {  # Aggressive
                'slam_file': 'slam_config2.yaml',
                'explore_file': 'explore_config2.yaml',
                'description': 'Aggressive - Fast movement, lower precision'
            },
            '3': {  # Balanced
                'slam_file': 'slam_config3.yaml',
                'explore_file': 'explore_config3.yaml',
                'description': 'Balanced - Default configuration'
            },
            '4': {  # Performing
                'slam_file': 'slam_config4.yaml',
                'explore_file': 'explore_config4.yaml',
                'description': 'Performing - High update frequency'
            }
        }

        # Select configuration based on user input
        if config_choice not in self.configurations:
            self.get_logger().error(f"Invalid configuration choice: {config_choice}")
            raise ValueError("Configuration choice must be 1, 2, 3, or 4")
        
        self.selected_config = self.configurations[config_choice]
        self.get_logger().info(f"\nUsing configuration {config_choice}: {self.selected_config['description']}")

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Navigation parameters
        self.MAX_RETRY = 3
        self.WAIT_TIME = 2.0    # Reduced wait time
        self.MAP_WAIT_TIME = 5.0  # Reduced mapping wait time
        
        # Initialize metrics
        self.metrics = {
            'start_time': None,
            'end_time': None,
            'total_time': 0,
            'waypoint_times': [],
            'waypoint_distances': [],
            'waypoint_speeds': [],
            'total_distance': 0.0,
            'average_speed': 0.0,
            'min_obstacle_distances': [],
            'overall_min_obstacle_distance': float('inf'),
            'configuration_used': config_choice,
            'configuration_description': self.selected_config['description']
        }
        
        self.last_pose = None
        self.current_waypoint_distance = 0.0
        self.current_waypoint_min_obstacle = float('inf')
        self.waypoint_distances_history = []
        
        # Set up subscriptions
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.laser_callback,
            10
        )

    def wait_for_servers(self):
        """Wait for the navigation server to become available."""
        self.get_logger().info("\nWaiting for Navigation server...")
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Navigation not available, waiting...")
        self.get_logger().info("Navigation server connected!")
        
        self.get_logger().info(f"\nWaiting {self.MAP_WAIT_TIME} seconds for mapping...")
        time.sleep(self.MAP_WAIT_TIME)
        self.get_logger().info("Ready to navigate!")

    def laser_callback(self, msg):
        """Process laser scan data and record minimum distances."""
        # Filtro più preciso per i range validi
        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            # Verifica che il valore sia valido e nel range del sensore
            if (not math.isinf(r) and not math.isnan(r) and 
                r > msg.range_min and r < msg.range_max):
                valid_ranges.append(r)
        
        if valid_ranges:
            min_dist = min(valid_ranges)
            
            # Aggiornamento delle metriche
            if min_dist < self.current_waypoint_min_obstacle:
                self.current_waypoint_min_obstacle = min_dist
                self.metrics['overall_min_obstacle_distance'] = min(
                    self.metrics['overall_min_obstacle_distance'],
                    min_dist
                )
                # Log per distanze molto piccole
                if min_dist < 0.3:
                    self.get_logger().warn(f'Close obstacle detected! Distance: {min_dist:.3f}m')
                    
            # Aggiunta alla storia delle distanze
            if hasattr(self, 'waypoint_distances_history'):
                self.waypoint_distances_history.append(min_dist)

    def pose_callback(self, msg):
        current_pose = msg.pose.pose
        
        if self.last_pose is not None:
            distance = self.calculate_distance(self.last_pose, current_pose)
            self.metrics['total_distance'] += distance
            self.current_waypoint_distance += distance
        
        self.last_pose = current_pose

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses."""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

    def create_pose_stamped(self, position, orientation):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = position['x']
        pose.pose.position.y = position['y']
        pose.pose.position.z = position['z']
        
        pose.pose.orientation.x = orientation['x']
        pose.pose.orientation.y = orientation['y']
        pose.pose.orientation.z = orientation['z']
        pose.pose.orientation.w = orientation['w']
        
        return pose

    def navigate_to_pose(self, goal_pose, goal_name):
        retry_count = 0
        waypoint_start_time = time.time()
        
        while retry_count < self.MAX_RETRY:
            retry_count += 1
            self.get_logger().info(f"\nAttempt {retry_count}/{self.MAX_RETRY}")
            
            # Prepare and send goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            
            self.get_logger().info(f"Sending goal for {goal_name}...")
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda msg: self.feedback_callback(msg, goal_name)
            )
            
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected")
                time.sleep(self.WAIT_TIME)
                continue
            
            self.get_logger().info("Goal accepted, navigating...")
            result_future = goal_handle.get_result_async()
            
            start_time = time.time()
            while not result_future.done():
                if time.time() - start_time > 180.0:  # 3 minutes timeout
                    self.get_logger().warn("Navigation timeout!")
                    goal_handle.cancel_goal_async()
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            
            try:
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    waypoint_time = time.time() - waypoint_start_time
                    self.metrics['waypoint_times'].append(waypoint_time)
                    self.metrics['waypoint_distances'].append(self.current_waypoint_distance)
                    waypoint_speed = self.current_waypoint_distance / waypoint_time if waypoint_time > 0 else 0
                    self.metrics['waypoint_speeds'].append(waypoint_speed)
                    self.metrics['min_obstacle_distances'].append(self.current_waypoint_min_obstacle)
                    
                    self.get_logger().info(f"\nSuccessfully reached {goal_name}!")
                    return True
                else:
                    self.get_logger().error(f"Navigation failed with status: {status}")
            except Exception as e:
                self.get_logger().error(f"Navigation error: {e}")
            
            time.sleep(self.WAIT_TIME)
        return False

    def feedback_callback(self, feedback_msg, goal_name):
        feedback = feedback_msg.feedback
        sys.stdout.write(f"\r🚗 {goal_name} - Distance remaining: {feedback.distance_remaining:.2f}m")
        sys.stdout.flush()

    def print_waypoint_metrics(self, waypoint_num):
        """Print detailed metrics for a waypoint."""
        if hasattr(self, 'waypoint_distances_history') and self.waypoint_distances_history:
            distances = self.waypoint_distances_history
            min_dist = min(distances)
            avg_dist = sum(distances) / len(distances)
            
            self.get_logger().info(
                f'\nWaypoint {waypoint_num} Distance Statistics:'
                f'\n  Absolute Minimum: {min_dist:.3f}m'
                f'\n  Average: {avg_dist:.3f}m'
                f'\n  Samples below 0.3m: {len([d for d in distances if d < 0.3])}'
            )

    def print_metrics_summary(self):
        self.get_logger().info('\nNavigation Metrics Summary:')
        self.get_logger().info(f'Configuration: {self.metrics["configuration_description"]}')
        self.get_logger().info(f'Total Time: {self.metrics["total_time"]:.2f} seconds')
        self.get_logger().info(f'Total Distance: {self.metrics["total_distance"]:.2f} meters')
        self.get_logger().info(f'Average Speed: {self.metrics["average_speed"]:.2f} m/s')
        self.get_logger().info(f'Overall Minimum Distance to Obstacle: {self.metrics["overall_min_obstacle_distance"]:.3f} m')
        
        if self.metrics['waypoint_times']:
            self.get_logger().info('\nPer-Waypoint Metrics:')
            for i in range(len(self.metrics['waypoint_times'])):
                self.get_logger().info(
                    f'Waypoint {i+1}:'
                    f'\n\tTime: {self.metrics["waypoint_times"][i]:.2f} s'
                    f'\n\tDistance: {self.metrics["waypoint_distances"][i]:.2f} m'
                    f'\n\tAverage Speed: {self.metrics["waypoint_speeds"][i]:.2f} m/s'
                    f'\n\tMinimum Distance to Obstacle: {self.metrics["min_obstacle_distances"][i]:.3f} m'
                )

    def reset_waypoint_metrics(self):
        """Reset metrics for new waypoint."""
        self.current_waypoint_distance = 0.0
        self.current_waypoint_min_obstacle = float('inf')
        self.waypoint_distances_history = []

    def navigate_waypoints(self):
        try:
            # Load waypoints from config
            config_dir = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'config')
            with open(os.path.join(config_dir, 'tomapping.yaml'), 'r') as file:
                waypoints = yaml.safe_load(file)['waypoints']
            
            # Goal order: 3 → 4 → 2 → 1
            goal_order = [
                (0, "Goal 1 "),
                (1, "Goal 2 "),
                (2, "Goal 3 "),
                (3, "Goal 4 "),
                (4, "Goal 5 "),
                (5, "Goal 6 "),
                (6, "Goal 7 "),
                (7, "Goal 8 "),
                (8, "Goal 9 "),
                (9, "GOal 10 - Last one!")

            ]
            
            self.metrics['start_time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # Execute navigation sequence
            for idx, (waypoint_idx, goal_name) in enumerate(goal_order, 1):
                self.get_logger().info(f"\n{'='*50}")
                self.get_logger().info(f"Navigating to {goal_name}")
                self.get_logger().info(f"{'='*50}")
                
                waypoint = waypoints[waypoint_idx]
                goal_pose = self.create_pose_stamped(waypoint['position'], waypoint['orientation'])
                
                # Reset per-waypoint metrics
                self.reset_waypoint_metrics()
                
                success = self.navigate_to_pose(goal_pose, goal_name)
                
                # Print detailed metrics for this waypoint
                self.print_waypoint_metrics(idx)
                
                if idx < len(goal_order):
                    self.get_logger().info(f"\nWaiting between goals ({self.WAIT_TIME}s)...")
                    time.sleep(self.WAIT_TIME)
            
            # Calculate final metrics
            self.metrics['end_time'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            start = datetime.strptime(self.metrics['start_time'], '%Y-%m-%d %H:%M:%S')
            end = datetime.strptime(self.metrics['end_time'], '%Y-%m-%d %H:%M:%S')
            self.metrics['total_time'] = (end - start).total_seconds()
            
            if self.metrics['total_time'] > 0:
                self.metrics['average_speed'] = self.metrics['total_distance'] / self.metrics['total_time']
            
            self.print_metrics_summary()
            self.get_logger().info("\n🎯 Navigation sequence completed!")
            
        except Exception as e:
            self.get_logger().error(f"\nError during navigation: {str(e)}")
            raise
def main():
    rclpy.init()
    
    # Get configuration choice from command line
    if len(sys.argv) != 2:
        print("Usage: ros2 run <package> testing.py <config_number>")
        print("Config options: 1(Conservative), 2(Aggressive), 3(Balanced), 4(Performing)")
        sys.exit(1)
    
    config_choice = sys.argv[1]
    
    try:
        navigator = ImprovedNavigator(config_choice)
        navigator.wait_for_servers()
        navigator.navigate_waypoints()
    except KeyboardInterrupt:
        print("\n⚠️ Navigation cancelled by user!")
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()