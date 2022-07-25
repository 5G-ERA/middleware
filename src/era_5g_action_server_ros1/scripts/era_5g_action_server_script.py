#! /usr/bin/env python

import rospy
import actionlib

from era_5g_action_client_ros1.msg import *
from era_5g_action_client_ros1.msg import goal_5gActionGoal

from actionlib_msgs.msg import *
from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle

import requests
from requests.exceptions import HTTPError
import time
import uuid
import os
import json

import collections
import threading

global PASSWORD
global TOKEN
global PLAN
global LOGIN_OCELOT
global PLAN_OCELOT
global RESOURCE_STATUS_OCELOT
global K8IP

TOKEN = ''
PLAN = ''
LOGIN_OCELOT = ''
PLAN_OCELOT = ''
RESOURCE_STATUS_OCELOT = ''
ID = '97f4059f-acfa-48f9-b22c-165ecbc51ed1'
ACTION_PLAN_ID = ''
PASSWORD = '5g-era'
K8IP = '10.64.140.43' # Change to the middleware deployed IP

class ActionServerNode():
    def __init__(self):
        # Create an instance of simpleActionServer
        self.a_server = actionlib.SimpleActionServer("goal_5g", goal_5gAction, execute_cb=self.execute_cb,auto_start=False)
        # Set the callback to be executed when a goal is received
        # self.a_server.register_goal_callback(self.goal_callback)
        # Set the callback that should be executed when a preempt request is received
        self.a_server.register_preempt_callback(self.preempt_callback)
        # Start the server
        rospy.loginfo("Starting 5g-era-action-server...")
        self.a_server.start()


        global K8IP
        global LOGIN_OCELOT
        global PLAN_OCELOT
        global RESOURCE_STATUS_OCELOT

        LOGIN_OCELOT = "http://"+str(K8IP)+"/Login"
        PLAN_OCELOT = "http://"+str(K8IP)+"/Task/Plan"
        RESOURCE_STATUS_OCELOT = "http://" + str(K8IP)+"/orchestrate/orchestrate/plan/"


    def destroy(self):
        rospy.loginfo("Shutting down Action Server....")
        self.a_server.shutdown()


    def gateway_login(self,ID,PASSWORD):
        rospy.loginfo("Trying to log into the middleware")
        # Request Login
        try:
            r = requests.post(LOGIN_OCELOT, json={"Id": ID, "Password": PASSWORD})
            Token = jsondata = r.json()
            newToken = (Token['token'])  # Token is stored here
            return newToken

        except HTTPError as e:
            rospy.loginfo("Could not login to the middleware gateway")
            rospy.loginfo(e.response.status_code)
            return 'Error, could not login to gateway, revisit the log files for more details.'


    def gateway_get_plan(self,newToken,taskid,PLAN_OCELOT):
        # Request plan
        try:
            rospy.loginfo("Goal task is: "+str(taskid))
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": str(taskid)}

            response = requests.post(PLAN_OCELOT, json=data, headers=hed)
            rospy.loginfo("PLAN: " + str(response.json()))
            Action_Sequence = jsondata = response.json()
            actionseq = Action_Sequence['ActionSequence']
            ACTION_PLAN_ID = Action_Sequence['ActionPlanId']
            rospy.loginfo("ActionPlanId ** is: " + str(actionseq))
            return response.json(), ACTION_PLAN_ID
        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get the plan, revisit the log files for more details.'

    def gateway_get_number_steps_in_plan(self,newToken, taskid):
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": taskid}
            response = requests.post(PLAN_OCELOT, json=data, headers=hed)
            Action_Sequence = jsondata = response.json()
            actionseq = (Action_Sequence['ActionSequence'])
            return (len(actionseq))
        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get the number of steps, revisit the log files for more details.'

    def gateway_get_actionSequenceIds(self, plan):
         try:
            rospy.loginfo("type "+str(type(plan)))
            Action_Sequence_Data = plan['ActionSequence']
            number_steps = len(Action_Sequence_Data)
            ActionSequenceIdsList = []
            for x in range(0, number_steps):  # Iterate over all the actions within the action sequence and get their ids.
                Action_SequenceFullData = Action_Sequence_Data[x]
                ActionSequenceIdsList.append(Action_SequenceFullData['Id'])
            return ActionSequenceIdsList
         except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get the number list of action ids, revisit the log files for more details.'

    def deleteAllResources(self, newToken, ACTION_PLAN_ID):
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.delete(url, headers=hed)

            if '200' in str(response):
                rospy.loginfo('Resource deleted with id: ' + ID)

        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get delete the resource, revisit the log files for more details.'

    def deleteSingleResource(self):
        pass #TODO

    def getResourceStatus(self, ACTION_PLAN_ID, newToken):
        try:  # query orchestrator for latest information regarding the status of resources.
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.get(url, headers=hed)
            return str(response.json())
        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get the resource status, revisit the log files for more details.'

    def gatewayLogOff(self):
        rospy.loginfo('Middleware log out succesful ')
        #TODO
        pass

# This function will decide if a goal is accepted or rejected.
    def goal_callback(self):

        rospy.loginfo("RECEIVED GOAL REQUEST - Trying to process request")
        global TOKEN
        global ACTION_PLAN_ID
        global PLAN
        global ID

        index = goal_handle.action_reference
        rospy.loginfo('Feedback: {0}'.format("index: "+str(index)))

        if (index != 0) and (len(PLAN) != 0):
            self.a_server.accept_new_goal()

        # Login to gateway function
        newToken = self.gateway_login(ID, PASSWORD)
        TOKEN = newToken

        if 'Error' in newToken:
            rospy.loginfo('Feedback: {0}'.format(
                "Error, could not login to gateway, revisit the log files for more details."))  # Update feedback disply in command line
            self.a_server.set_aborted()
        else:
            rospy.loginfo('Feedback: {0}'.format(
                "Login successful, token received"))
            PLAN, ActionPlanId = self.gateway_get_plan(newToken, goal_handle.goal_taskid,PLAN_OCELOT)  # Get the plan by sending the token and TaskId

            ACTION_PLAN_ID = ActionPlanId  # Update the global variable actionPlanId
            rospy.loginfo("ActionPlanId is: " + ACTION_PLAN_ID)
            if 'Error' in PLAN:
                rospy.loginfo(
                    'Feedback: {0}'.format("Error, could not get the plan, revisit the log files for more details."))
                self.a_server.set_aborted()
            else:
                rospy.loginfo('Feedback: {0}'.format(
                    "Got new plan successfully."))
                self.a_server.accept_new_goal()

    # Will execute the goal if it was accepted by the register_goal_callback function.
    def execute_cb(self, goal):
        global FirstActionId

        global TOKEN
        global ACTION_PLAN_ID
        global PASSWORD
        global ID
        global PLAN

        SubactionRef = goal.request.action_reference
        feedback_msg = goal_5gFeedback.Feedback()  # create action instance feedback

        rospy.loginfo('SubactionRef '+str(SubactionRef))
        rospy.loginfo('SubactionRef '+str(type(SubactionRef)))
        if SubactionRef == 0:  # This is a completely new action.
            FirstActionId = SubactionRef
            rospy.loginfo('Executing New action goal... ')

            actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
            rospy.loginfo("ActionSequenceIds: " + str(actionSequenceIds))
            feedback_msg.action_sequence = actionSequenceIds
            feedback_msg.feedback_resources_status = ''

            self.a_server.publish_feedback(feedback_msg)  # Publish the feedback

            rospy.loginfo("Publishing first feedback")
            rospy.loginfo("goal handle: " + str(goal_handle))

            # goal_handle.succeed()
            # result.success = True

            while self.a_server.is_active:

                rospy.loginfo("Processing goal")

                if not self.a_server.is_active:
                    #  result = goal_5g.Result()
                    rospy.loginfo('Goal aborted')
                    # result = 'Goal aborted'
                    # return goal_5g.Result


                rospy.loginfo("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID,TOKEN)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
                self.a_server.publish_feedback(
                    feedback_msg)  # Publish the feedback
                time.sleep(1)

        elif SubactionRef == -1:  # if received -1 as second parameter to the action goal, goal was succesful

            rospy.loginfo('Removing resources')
            self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
            resultObject = Goal5g.Result()
            resultObject.result = "Finished"
            self.a_server.succeed()

        else:  # This is a subaction - check
            rospy.loginfo("Subaction")

            while goal_handle.is_active:
                rospy.loginfo("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ACTION_PLAN_ID,TOKEN)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(PLAN)
                self.a_server.publish_feedback(
                    feedback_msg)  # Publish the feedback
                time.sleep(1)


    def preempt_callback(self):
        """
            Callback executed when a preempt request has been received. --> cancel the goal.
        """
        rospy.loginfo("Action preempted - canceling")
        global ACTION_PLAN_ID
        global TOKEN
        # It will delete all the resources.
        self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
        self.action_server.set_preempted()

if __name__ = "__main__":
    rospy.loginfo("Starting 5g-era-action-server")
    rospy.init_node("action_server")
    actionServerObj = ActionServerNode()
    rospy.spin()

