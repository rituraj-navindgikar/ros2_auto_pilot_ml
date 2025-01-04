#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gymnasium import Env
from gymnasium.spaces import Box
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import numpy as np
import threading
import time

class RobotEnv(Node, Env):
    def __init__(self):
        super().__init__('robot_rl_env')

        # Create a publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to the LaserScan topic
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Initialize state and control variables
        self.lidar_data = np.ones(360) * 10  # Initialize with max range
        self.collision = False
        self.current_step = 0
        self.max_steps = 500

        # Action and observation space
        self.action_space = Box(low=np.array([0.5, -0.6]), high=np.array([0.6, 0.6]), dtype=np.float64)  # [linear_x, angular_z]
        self.observation_space = Box(low=0.0, high=10.0, shape=(360,), dtype=np.float64)  # 360-degree laser scan

    def lidar_callback(self, msg: LaserScan):
        """Process LaserScan data."""
        self.lidar_data = np.array(msg.ranges)
        self.lidar_data = np.clip(self.lidar_data, 0, 10)  # Clip ranges to max distance (10m)
        if np.min(self.lidar_data) < 0.5:  # If very close to an obstacle
            self.collision = True

    def reset(self, seed=None, options=None):
        """Reset the environment."""
        self.get_logger().info('Environment reset')
        self.collision = False
        self.current_step = 0

        # Stop the robot
        self._send_cmd_vel(0.0, 0.0)

        # Allow sensors to stabilize
        time.sleep(1)

        # Ensure the lidar data is float32
        self.lidar_data = self.lidar_data.astype(np.float32)

        # Return initial observation and additional info
        return self.lidar_data, {}


    def step(self, action):
        """Perform a step in the environment."""
        linear_x, angular_z = action
        self._send_cmd_vel(linear_x, angular_z)

        # Wait for the action to complete
        time.sleep(0.1)

        self.current_step += 1
        reward = self._calculate_reward(action)
        done = self._check_done()

        # Ensure the lidar data is float32
        self.lidar_data = self.lidar_data.astype(np.float32)

        return self.lidar_data, reward, done, False, {}


    def _send_cmd_vel(self, linear, angular):
        """Send velocity commands to the robot."""
        twist = Twist()
        twist.linear.x = float(linear)  # Convert to Python float
        twist.angular.z = float(angular)  # Convert to Python float
        self.cmd_vel_pub.publish(twist)


    def _calculate_reward(self, action):
        """Calculate the reward for the current step."""
        linear_x, angular_z = action
        reward = 0
        # Strong reward for moving forward

        reward = linear_x * 10
        
        # Penalize over rotation
        reward -= abs(angular_z) * 2

        # Reward rotation
        reward += abs(angular_z) * 2

        # Penalize longer episodes
        reward -= self.current_step * 0.001

        # Punish for collisions
        if self.collision:
            reward = -100
        print(reward)
        return reward

    def _check_done(self):
        """Check if the episode is done."""
        if self.collision:
            return True  # End the episode on collision
        if self.current_step >= self.max_steps:
            return True  # End the episode if max steps are reached
        return False

    def close(self):
        """Close the environment."""
        self._send_cmd_vel(0.0, 0.0)  # Stop the robot

    def render(self):
        pass


def main(args=None):
    rclpy.init()

    # Initialize environment
    env = RobotEnv()

    # Check environment compatibility
    check_env(env)

    # Train PPO agent
    def train_rl():
        # Create the PPO agent
        model = PPO("MlpPolicy", env, verbose=1, learning_rate=1e-3)
                
        # Train the agent
        model.learn(total_timesteps=100000)

        # Save the trained model
        model.save("ppo_robot_model")
        env.get_logger().info("Training complete and model saved.")

    # Run training in a separate thread
    thread = threading.Thread(target=train_rl)
    thread.start()

    # Spin the ROS node
    rclpy.spin(env)

    # Shutdown
    env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()