#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from turtlebot_kinova_transport.gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import threading
import sys
import os
import argparse  # Added for command-line arguments

from ament_index_python.packages import get_package_share_directory

# Get the package path for the CSV file
package_share_dir = get_package_share_directory('turtlebot_kinova_transport')
csv_path = os.path.join(package_share_dir, 'data', 'data_final_demo.csv')

class TurtleBotController(Node):
    """
    Controller for the TurtleBot to handle open-loop navigation.
    """
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('TurtleBot controller initialized')
        
        # Start a thread for ROS spinning
        self.spin_thread = threading.Thread(target=self._spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def _spin(self):
        """Background thread for processing ROS callbacks."""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def move_linear(self, distance, speed=0.2, forward_correction=0.01, backward_correction=-0.01):
        """
        Move the turtlebot linearly by the specified distance in meters using open-loop control.
        Includes angular correction for both directions to counter drift.
        
        Args:
            distance (float): Distance to move in meters (positive for forward, negative for backward)
            speed (float): Linear speed in m/s (always positive)
            forward_correction (float): Angular velocity correction when moving forward (positive to turn left)
            backward_correction (float): Angular velocity correction when moving backward (negative to turn right)
        """
        direction = 1 if distance > 0 else -1
        abs_distance = abs(distance)
        abs_speed = abs(speed)
        
        # Apply the appropriate correction based on direction
        if direction > 0:
            angular_correction = forward_correction  # Moving forward, apply forward correction
            direction_str = "forward"
        else:
            angular_correction = backward_correction  # Moving backward, apply backward correction
            direction_str = "backward"
        
        self.get_logger().info(f'Moving {direction_str} {abs_distance} meters at {abs_speed} m/s')
        self.get_logger().info(f'Using angular correction of {angular_correction} rad/s')
        
        # Calculate time needed to travel the distance at the given speed
        time_to_travel = abs_distance / abs_speed
        
        # Create twist message
        twist = Twist()
        twist.linear.x = direction * abs_speed
        twist.angular.z = angular_correction  # Apply direction-specific correction
        
        # Record start time
        start_time = time.time()
        
        # Publish velocity command for the calculated duration
        while time.time() - start_time < time_to_travel:
            self.publisher.publish(twist)
            time.sleep(0.1)  # Small delay to avoid flooding
        
        # Stop the robot
        self._stop()
        self.get_logger().info('Stopped moving')

    def _stop(self):
        """Stop the turtlebot by publishing zero velocity."""
        twist = Twist()
        for _ in range(5):  # Send multiple stop commands to ensure it stops
            self.publisher.publish(twist)
            time.sleep(0.1)

def load_poses_from_csv(file_path):
    """
    Load pose data from a CSV file and return a dictionary of Pose objects.
    
    Args:
        file_path (str): Path to the CSV file containing pose data
        
    Returns:
        dict: Dictionary mapping pose names to Pose objects
    """
    poses = {}
    try:
        with open(file_path, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                pose = Pose()
                pose.position = Point(
                    x=float(row['pos_x']),
                    y=float(row['pos_y']),
                    z=float(row['pos_z'])
                )
                pose.orientation = Quaternion(
                    x=float(row['ori_x']),
                    y=float(row['ori_y']),
                    z=float(row['ori_z']),
                    w=float(row['ori_w'])
                )
                poses[row['name']] = pose
        print(f"Successfully loaded {len(poses)} poses from {file_path}")
    except Exception as e:
        print(f"Error loading poses from CSV: {e}")
        sys.exit(1)
    
    # Print available poses for debugging
    print(f"Available poses: {list(poses.keys())}")
    return poses

def move(arm, pose=None, delay=1):
    """
    Execute a movement and then wait for a short delay.
    
    Args:
        arm: Robot arm controller
        pose: Target pose
        delay (float): Time to wait after moving in seconds
    """
    print(f"Moving to: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
    arm.inverse_kinematic_movement(pose)
    time.sleep(delay)

def perform_pick_and_place_sequence(arm, gripper, positions):
    """
    Perform a complete pick-and-place sequence following the exact order of positions from the CSV.
    Releases the cube only at the retreat_position (the last position in sequence).
    
    The sequence followed is:
    1. pick_position
    2. mid_position
    3. place_position
    4. approach_position
    5. retreat_position
    
    Args:
        arm: The robot arm controller
        gripper: The gripper controller
        positions: Dictionary of named positions from the CSV
    """
    # Verify we have all required positions
    required_positions = ["pick_position", "mid_position", "place_position", "approach_position", "retreat_position"]
    for pos in required_positions:
        if pos not in positions:
            print(f"Error: Required position '{pos}' not found in CSV data")
            print(f"Available positions: {list(positions.keys())}")
            return
    
    # Move to pick position
    print("Moving to pick position")
    move(arm, positions["pick_position"], 2)
    
    # Close gripper to grasp object
    print("Closing gripper")
    gripper.move_to_position(0.65)
    time.sleep(1.0)
    
    # Move to mid position
    print("Moving to mid position")
    move(arm, positions["mid_position"], 2)
    
    # Move to place position
    print("Moving to place position")
    move(arm, positions["place_position"], 2)
    
    # Move to approach position
    print("Moving to approach position")
    move(arm, positions["approach_position"], 2)
    
    # Move to retreat position
    print("Moving to retreat position")
    move(arm, positions["retreat_position"], 2)
    
    # Open gripper to release object at the retreat position (the last position)
    print("Opening gripper to release object at retreat position")
    gripper.move_to_position(0.0)
    time.sleep(1.0)

def test_turtlebot_movement():
    """Test TurtleBot movement forward and backward."""
    print("\n=== Testing TurtleBot Movement ===")
    
    turtlebot = TurtleBotController()
    
    try:
        # Test forward movement (0.5 meters)
        print("\nTesting forward movement (0.5 meters)...")
        turtlebot.move_linear(3.4, speed=0.2)
        time.sleep(2.0)  # Wait between movements
        
        # Test backward movement (0.5 meters)
        print("\nTesting backward movement (0.5 meters)...")
        turtlebot.move_linear(-3.3, speed=0.2)
        
        print("\nTurtleBot movement test completed successfully")
    except Exception as e:
        print(f"Error during TurtleBot testing: {e}")

def test_arm_pick_place():
    """Test Kinova arm pick and place functionality."""
    print("\n=== Testing Kinova Arm Pick and Place ===")
    
    arm = None
    gripper = None
    
    try:
        arm = Gen3LiteArm()
        gripper = Gen3LiteGripper()
        
        # Move arm to a safe vertical/home position
        print("Moving arm to home position")
        arm.go_vertical()
        gripper.move_to_position(0.0)
        time.sleep(1.0)
        
        # Load poses from the CSV file
        print(f"Loading poses from {csv_path}")
        poses = load_poses_from_csv(csv_path)
        
        # Execute pick-and-place sequence
        print("Executing pick and place sequence")
        perform_pick_and_place_sequence(arm, gripper, poses)
        
        # Return to home position
        print("Moving arm back to home position")
        arm.go_vertical()
        
        print("\nKinova arm pick and place test completed successfully")
    except Exception as e:
        print(f"Error during Kinova arm testing: {e}")
    finally:
        # Shutdown procedures
        if gripper:
            gripper.shutdown()
        if arm:
            arm.shutdown()

def full_task():
    """Execute the full task with both TurtleBot and Kinova arm."""
    print("\n======= Kinova-TurtleBot Pick and Place Task =======")
    print("1. TurtleBot will move forward 3.16 meters")
    print("2. Kinova arm will pick up an object and place it in the basket")
    print("3. TurtleBot will return to the starting position")
    print("====================================================\n")
    
    # Initialize controllers
    print("Initializing controllers...")
    turtlebot = None
    arm = None
    gripper = None
    
    try:
        turtlebot = TurtleBotController()
        arm = Gen3LiteArm()
        gripper = Gen3LiteGripper()
        print("Controllers initialized successfully")

        # Move arm to a safe vertical/home position
        print("Moving arm to home position")
        arm.go_vertical()
        gripper.move_to_position(0.0)
        time.sleep(1.0)
        
        # Load poses from the CSV file
        print(f"Loading poses from {csv_path}")
        poses = load_poses_from_csv(csv_path)
        
        # Step 1: Move turtlebot forward 316 cm
        print("\n--- Step 1: Moving TurtleBot Forward ---")
        turtlebot.move_linear(3.16, speed=0.2)
        time.sleep(2.0)  # Wait for turtlebot to settle
        
        # Step 2: Execute pick-and-place sequence with the arm
        print("\n--- Step 2: Executing Pick and Place Sequence ---")
        perform_pick_and_place_sequence(arm, gripper, poses)
        
        # Give time for the object to settle in the basket
        time.sleep(2.0)
        
        # Step 3: Move turtlebot backward 316 cm to return to start
        print("\n--- Step 3: Returning TurtleBot to Start Position ---")
        turtlebot.move_linear(-3.16, speed=0.2)
        
        # Move arm back to a safe vertical/home position
        print("Moving arm to home position")
        arm.go_vertical()
        
        print("\n--- Task Completed Successfully ---")
    except Exception as e:
        print(f"Error during execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Shutdown procedures
        print("Shutting down...")
        if gripper:
            gripper.shutdown()
        if arm:
            arm.shutdown()

def main():
    """
    Main function with command-line argument support for testing components separately.
    """
    # Set up argument parser
    parser = argparse.ArgumentParser(description='TurtleBot-Kinova Integration with Component Testing')
    parser.add_argument('--test', choices=['turtlebot', 'arm', 'full'], default='full',
                        help='Test specific component: turtlebot, arm, or full task (default)')
    
    # Parse arguments
    args = parser.parse_args()
    
    # Initialize ROS
    rclpy.init()
    
    try:
        # Run the selected test mode
        if args.test == 'turtlebot':
            test_turtlebot_movement()
        elif args.test == 'arm':
            test_arm_pick_place()
        else:  # 'full' mode
            full_task()
    finally:
        # Final shutdown
        rclpy.shutdown()

if __name__ == "__main__":
    main()