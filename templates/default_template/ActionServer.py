#!/usr/bin/env python3 
"""
Description:
  Action Server to connect to 5G ERA gateway.
  Action is successful when the power_supply_status goes from
  NOT_CHARGING(i.e. 3) to CHARGING(i.e. 1)
-------
Subscription Topics:
  Current battery state
  /battery_status – sensor_msgs/BatteryState
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
Author: Adrian Lendinez
Website: A
Date: 11 Abril 2022
"""

#Active/Down/Instantiating/Idle/Terminating/Problem
import time
 
# Import our action definition
from interfaces5g.action import resource
 
# ROS Client Library for Python
import rclpy
 
# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer
 
# Enables publishers, subscribers, and action servers to be in a single node
from rclpy.executors import MultiThreadedExecutor
 
# Handles the creation of nodes
from rclpy.node import Node
 
# Handle Twist messages, linear and angular velocity
from geometry_msgs.msg import Twist


# Rest python library
import requests


 
# Holds the current state of the battery
this_battery_state = BatteryState()
 
class ConnectToGatewayActionServer(Node):
  """
  Create a ConnectToGatewayActionServer class, 
  which is a subclass of the Node class.
  """
  def __init__(self):
   
    # Initialize the class using the constructor
    super().__init__('connect_to_gateway_action_server')
     
    # Instantiate a new action server
    # self, type of action, action name, callback function for executing goals
    self._action_server = ActionServer(
      self,
      ConnectToGateway,
      'connect_to_gateway',
      self.execute_callback)
    
       
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot
    self.publisher_cmd_vel = self.create_publisher(
      Twist,
      '/cmd_vel',
      10)   
 
    # Declare velocities
    self.linear_velocity = 0.0
    self.angular_velocity = 0.15
       
  def execute_callback(self, goal_handle):
    """
    Action server callback to execute accepted goals.
    """ 
    self.get_logger().info('Executing goal...')
 
    # Interim feedback
    feedback_msg = resource.Feedback() #send plan
    feedback_msg.resource_update = 'Not ready'
     
    # For every 1 minute , get feedback
    while this_battery_state.power_supply_status != 1:
     
      # Publish the current battery state
      feedback_msg.current_battery_state = this_battery_state
      self.get_logger().info('NOT CHARGING...')
      goal_handle.publish_feedback(feedback_msg)
       
      # Send the velocity command to the robot by publishing to the topic
      cmd_vel_msg = Twist()
      cmd_vel_msg.linear.x = self.linear_velocity
      cmd_vel_msg.angular.z = self.angular_velocity
      self.publisher_cmd_vel.publish(cmd_vel_msg)
       
      time.sleep(0.1)
     
    # Stop the robot
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.0
    cmd_vel_msg.angular.z = 0.0
    self.publisher_cmd_vel.publish(cmd_vel_msg)
 
    # Update feedback    
    feedback_msg.current_battery_state = this_battery_state
    goal_handle.publish_feedback(feedback_msg)
   
    # Indicate the goal was successful
    goal_handle.succeed()
    self.get_logger().info('CHARGING...')
    self.get_logger().info('Successfully connected to the charging dock!')
   
    # Create a result message of the action type
    result = ConnectToChargingDock.Result()
     
    # Update the final battery state
    result.final_battery_state = feedback_msg.current_battery_state
     
    return result
 
class BatteryStateSubscriber(Node):
    """
    Subscriber node to the current battery state
    """     
    def __init__(self):
   
      # Initialize the class using the constructor
      super().__init__('battery_state_subscriber')
     
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/BatteryState
      self.subscription_battery_state = self.create_subscription(
        BatteryState,
        '/battery_status',
        self.get_battery_state,
        10)
       
    def get_battery_state(self, msg):
      """
      Update the current battery state.
      """
      global this_battery_state
      this_battery_state = msg
     
def main(args=None):
  """
  Entry point for the program.
  """
 
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  try: 
   
    # Create the Action Server node
    connectToGatewayActionServer = ConnectToGatewayActionServer()
     
    # Create the Battery State subscriber node
    battery_state_subscriber = BatteryStateSubscriber()
     
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(connect_to_charging_dock_action_server)
    executor.add_node(battery_state_subscriber)
     
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connectToGatewayActionServer.destroy_node()
      battery_state_subscriber.destroy_node()
 
  finally:
    # Shutdown
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
