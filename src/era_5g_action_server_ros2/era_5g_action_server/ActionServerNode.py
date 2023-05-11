from typing import List
import rclpy
from rclpy.node import Node
import requests
from requests.exceptions import HTTPError
import os
import uuid
import time
import json
from era_5g_client.client import NetAppClient, MiddlewareInfo, FailedToConnect, RunTaskMode
from rclpy.action import ActionServer
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import *
from rclpy.action.server import ServerGoalHandle
from era_5g_action_interfaces.action import Goal5g
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import collections
from rospy_message_converter import json_message_converter
from rclpy.node import Node


# ip address or hostname of the middleware server
MIDDLEWARE_ADDRESS = os.getenv("MIDDLEWARE_ADDRESS", "127.0.0.1")
# middleware user
MIDDLEWARE_USER = os.getenv("MIDDLEWARE_USER", "00000000-0000-0000-0000-000000000000")
# middleware password
MIDDLEWARE_PASSWORD = os.getenv("MIDDLEWARE_PASSWORD", "password")
# middleware NetApp id (task id)
MIDDLEWARE_TASK_ID = os.getenv("MIDDLEWARE_TASK_ID", "00000000-0000-0000-0000-000000000000")
# middleware robot id (robot id)
MIDDLEWARE_ROBOT_ID = os.getenv("MIDDLEWARE_ROBOT_ID", "00000000-0000-0000-0000-000000000000")
# middleware generated plan
MIDDLEWARE_PLAN = {}

class ActionServerNode(Node):
    def __init__(self, *args):
        # Defines class of type ActionServerNode and inherits from subclass Node
        super().__init__('actionServer')

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            Goal5g,
            'goal_5g',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)  # instantiate a new action server
        
        self.client = NetAppClient(self.results_callback)

    def results_callback(self, data):
            message = json_message_converter.convert_json_to_ros_message('std_msgs/String', data) # Type we know from topic info from input_topic
            # ros pub

    #Destroy action server
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
    
    # Parser to get ["type", "topic_name", "rate"] from plan input topics
    def get_output_topics(self, plan: dict) -> list:
        #TODO
        input_topic = []
        return input_topic
    # Parser to get ["type", "topic_name", "rate"] from plan output topics
    def get_input_topics(self, plan: dict) -> list:
        #TODO
        output_topics = []
        return output_topics
    
    # Get all ids of each action in the action sequence
    def gateway_get_actionSequenceIds(self, plan: dict) -> List:
        try:
            Action_Sequence_Data = plan['ActionSequence']
            number_steps = len(Action_Sequence_Data)
            ActionSequenceIdsList = []
            for x in range(0,number_steps):  # Iterate over all the actions within the action sequence and get their ids.
                Action_SequenceFullData = Action_Sequence_Data[x]
                ActionSequenceIdsList.append(Action_SequenceFullData['Id'])
            return ActionSequenceIdsList  # Return a list of the actions ids.
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the number list of action ids, revisit the log files for more details.'

    # This function will handle new accepted goals
    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info("-------------------------> Handle accepted goal")
        with self._goal_queue_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    # This function will decide if a goal is accepted or rejected.
    def goal_callback(self, goal_handle):
        self.get_logger().info("RECEIVED GOAL REQUEST - Trying to process request")
        try:
            self.client.connect_to_middleware(MiddlewareInfo(MIDDLEWARE_ADDRESS, MIDDLEWARE_USER, MIDDLEWARE_PASSWORD))
        except FailedToConnect as ex:
            print(f"Failed to connect to server ({ex})")
            return GoalResponse.REJECT
        try:
            MIDDLEWARE_PLAN=self.client.gateway_get_plan(goal_handle.goal_taskid, resource_lock, robot_id)
        except FailedToConnect as ex:
            print(f"Failed to connect to server ({ex})")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    # Will execute the goal if it was accepted by the register_goal_callback function.
    def execute_callback(self, goal_handle: ServerGoalHandle):
        from utils import rosImports
        rosImports.load_library()  

        from utils import Publishers
        pub = Publishers(self.get_output_topics())
        pub_as = [a for a in dir(pub) if not a.startswith('__')]

        from utils import Subscriptions
        sub = Subscriptions(self.get_input_topics())
       
        SubactionRef = goal_handle.request.action_reference
        feedback_msg = Goal5g.Feedback()  # create action instance feedback

        self.get_logger().info('SubactionRef '+str(SubactionRef))
        self.get_logger().info('SubactionRef '+str(type(SubactionRef)))
        if SubactionRef == 0:  # This is a completely new action.
            FirstActionId = SubactionRef
            self.get_logger().info('Executing New action goal... ')

            actionSequenceIds = self.gateway_get_actionSequenceIds(MIDDLEWARE_PLAN)
            self.get_logger().info("ActionSequenceIds: " + str(actionSequenceIds))
            feedback_msg.action_sequence = actionSequenceIds
            feedback_msg.feedback_resources_status = ''
            goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
            while goal_handle.is_active:

                self.get_logger().info("Processing goal")
                if not goal_handle.is_active:
                    self.get_logger().info('Goal aborted')

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')

                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID, TOKEN)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(MIDDLEWARE_PLAN)
                goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
                time.sleep(1)

        elif SubactionRef == -1:  # if received -1 as second parameter to the action goal, goal was succesful

            self.get_logger().info('Removing resources')
            self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
            resultObject = Goal5g.Result()
            resultObject.result = "Finished"
            goal_handle.succeed()

        else:  # This is a subaction
            self.get_logger().info("Subaction "+str(SubactionRef))

            while goal_handle.is_active:
                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID, TOKEN)  
                actionSequenceIds = self.gateway_get_actionSequenceIds(MIDDLEWARE_PLAN)
                goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
                time.sleep(1)

    # function to be triggered once a goal is canceled by client side.
    def cancel_callback(self, cancel_callback):
        global ACTION_PLAN_ID
        global TOKEN
        # It will delete all the resources.
        self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    actionServer = ActionServerNode()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(actionServer)
    rclpy.spin(actionServer, executor=multi_thread_executor)
    rclpy.shutdown()

