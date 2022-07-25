# Usfeul documentation:
# http://design.ros2.org/articles/actions.html#goal-states
# http://design.ros2.org/articles/actions.html
# https://docs.ros2.org/latest/api/rclpy/api/actions.html
# http://wiki.ros.org/actionlib


#===========================IMPORTS=======================================#
# ROS Client Library for Python
from typing import List
import rclpy
# Handles the creation of nodes
from rclpy.node import Node
# REST Library for Python
import requests
from requests.exceptions import HTTPError
# JSON format python library
import json
import os
import uuid
import time

# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import *

from rclpy.action.server import ServerGoalHandle

from era_5g_action_interfaces.action import Goal5g
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import collections

#===========================VARIABLE DECLARATION==========================#
global PASSWORD
global TOKEN  # Token provided by the ocelot gateway after login succesful
global PLAN  # Plan generated by the middleware
global LOGIN_OCELOT  # URL to log in to middleware
global PLAN_OCELOT  # URL to get a plan from the middleware
# URL to get an update from the resources alocated from the middleware
global RESOURCE_STATUS_OCELOT
global ACTION_PLAN_ID  # Unique identifier of the plan generated by the middleware
global K8IP

TOKEN = ''
PLAN = ''
LOGIN_OCELOT = ""
PLAN_OCELOT = ""
RESOURCE_STATUS_OCELOT = ""
# Username to log into the middleware
ID = "97f4059f-acfa-48f9-b22c-165ecbc51ed1"
ACTION_PLAN_ID = ""
PASSWORD = "5g-era"
K8IP = "10.64.140.43"  # IP generated by the K8 neccesary for the middleware API calls


#============================================================================#

class ActionServerNode(rclpy.node.Node):
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

        global K8IP
        global LOGIN_OCELOT
        global PLAN_OCELOT
        global RESOURCE_STATUS_OCELOT

        LOGIN_OCELOT = "http://"+str(K8IP)+"/Login"
        PLAN_OCELOT = "http://"+str(K8IP)+"/Task/Plan"
        RESOURCE_STATUS_OCELOT = "http://" + \
            str(K8IP)+"/orchestrate/orchestrate/plan/"

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def gateway_login(self, ID, PASSWORD):
        self.get_logger().info("Trying to log into the middleware")
        # Request Login
        try:
            r = requests.post(LOGIN_OCELOT, json={
                              "Id": ID, "Password": PASSWORD})
            Token = jsondata = r.json()
            newToken = (Token['token'])  # Token is stored here
            return newToken

        except HTTPError as e:
            self.get_logger().ERROR("Could not login to the middleware gateway")
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not login to gateway, revisit the log files for more details.'

    def gateway_get_plan(self, newToken, taskid, PLAN_OCELOT):
        # Request plan
        try:
            print("Goal task is: ", taskid)
            self.get_logger().info("Goal task is : " + taskid)
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": str(taskid)}

            response = requests.post(PLAN_OCELOT, json=data, headers=hed)
            self.get_logger().info("PLAN: " + str(response.json()))
            Action_Sequence = jsondata = response.json()
            actionseq = Action_Sequence['ActionSequence']
            ACTION_PLAN_ID = Action_Sequence['ActionPlanId']
            self.get_logger().info("ActionPlanId ** is: " + str(actionseq))
            return response.json(), ACTION_PLAN_ID

        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the plan, revisit the log files for more details.'

    # Get number of steps in plan
    def gateway_get_number_steps_in_plan(self, newToken: str, taskid) -> int:
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": taskid}
            response = requests.post(PLAN_OCELOT, json=data, headers=hed)
            Action_Sequence = jsondata = response.json()
            actionseq = (Action_Sequence['ActionSequence'])
            return (len(actionseq))
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the number of steps, revisit the log files for more details.'

    def gateway_get_actionSequenceIds(self, plan: dict) -> List:
        try:
            self.get_logger().info("type "+str(type(plan)))
            Action_Sequence_Data = plan['ActionSequence']
            number_steps = len(Action_Sequence_Data)
            ActionSequenceIdsList = []
            for x in range(0,
                           number_steps):  # Iterate over all the actions within the action sequence and get their ids.
                Action_SequenceFullData = Action_Sequence_Data[x]
                ActionSequenceIdsList.append(Action_SequenceFullData['Id'])

            return ActionSequenceIdsList  # Return a list of the actions ids.
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the number list of action ids, revisit the log files for more details.'

    def deleteAllResources(self, newToken, ACTION_PLAN_ID):
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.delete(url, headers=hed)

            if '200' in str(response):
                self.get_logger().info('Resource deleted with id: ' + ID)

        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get delete the resource, revisit the log files for more details.'

    def deleteSingleResource(self):
        pass  # TO DO

    def getResourceStatus(self, ACTION_PLAN_ID, newToken):
        try:  # query orchestrator for latest information regarding the status of resources.
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.get(url, headers=hed)
            return str(response.json())
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the resource status, revisit the log files for more details.'

    def gatewayLogOff(self):
        self.get_logger().info('Middleware log out succesful ')
        # TO DO
        pass

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
        global TOKEN
        global ACTION_PLAN_ID
        global PLAN
        global ID

        index = goal_handle.action_reference
        self.get_logger().info('Feedback: {0}'.format("index: "+str(index)))

        if (index != 0) and (len(PLAN) != 0):  # received update for the task
            return GoalResponse.ACCEPT

        # Login to gateway function
        newToken = self.gateway_login(ID, PASSWORD)
        TOKEN = newToken

        if 'Error' in newToken:
            self.get_logger().info('Feedback: {0}'.format(
                "Error, could not login to gateway, revisit the log files for more details."))  # Update feedback disply in command line
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Feedback: {0}'.format(
                "Login successful, token received"))
            PLAN, ActionPlanId = self.gateway_get_plan(newToken, goal_handle.goal_taskid,
                                                       PLAN_OCELOT)  # Get the plan by sending the token and TaskId

            ACTION_PLAN_ID = ActionPlanId  # Update the global variable actionPlanId
            self.get_logger().info("ActionPlanId is: " + ACTION_PLAN_ID)
            if 'Error' in PLAN:
                self.get_logger().info(
                    'Feedback: {0}'.format("Error, could not get the plan, revisit the log files for more details."))
                return GoalResponse.REJECT
            else:
                self.get_logger().info('Feedback: {0}'.format(
                    "Got new plan successfully."))
                return GoalResponse.ACCEPT

    # Will execute the goal if it was accepted by the register_goal_callback function.
    def execute_callback(self, goal_handle: ServerGoalHandle):
        global FirstActionId

        global TOKEN
        global ACTION_PLAN_ID
        global PASSWORD
        global ID
        global PLAN

        SubactionRef = goal_handle.request.action_reference
        feedback_msg = Goal5g.Feedback()  # create action instance feedback

        self.get_logger().info('SubactionRef '+str(SubactionRef))
        self.get_logger().info('SubactionRef '+str(type(SubactionRef)))
        if SubactionRef == 0:  # This is a completely new action.
            FirstActionId = SubactionRef
            self.get_logger().info('Executing New action goal... ')

            actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
            self.get_logger().info("ActionSequenceIds: " + str(actionSequenceIds))
            feedback_msg.action_sequence = actionSequenceIds
            feedback_msg.feedback_resources_status = ''
            goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
            self.get_logger().info("Publishing first feedback")
            self.get_logger().info("goal handle: " + str(goal_handle))

            # goal_handle.succeed()
            # result.success = True

            while goal_handle.is_active:

                self.get_logger().info("Processing goal")

                if not goal_handle.is_active:
                    #  result = goal_5g.Result()
                    self.get_logger().info('Goal aborted')
                    # result = 'Goal aborted'
                    # return goal_5g.Result

                if goal_handle.is_cancel_requested:
                    # result = goal_5g.Result()
                    # result = 'Goal canceled'
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                #      goal_handle.canceled()
                # return goal_5g.Result()

                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID,
                                                                                TOKEN)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
                goal_handle.publish_feedback(
                    feedback_msg)  # Publish the feedback
                time.sleep(1)

        elif SubactionRef == -1:  # if received -1 as second parameter to the action goal, goal was succesful

            self.get_logger().info('Removing resources')
            self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
            resultObject = Goal5g.Result()
            resultObject.result = "Finished"
            goal_handle.succeed()

        else:  # This is a subaction - check
            self.get_logger().info("Subaction")

            while goal_handle.is_active:
                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID,
                                                                                TOKEN)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
                goal_handle.publish_feedback(
                    feedback_msg)  # Publish the feedback
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
