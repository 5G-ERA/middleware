#!/usr/bin/env python

import rospy
import actionlib

from era_5g_action_client_ros1.msg import *
from era_5g_action_client_ros1.msg import goal_5gActionGoal
import rospy
from std_msgs.msg import String

#rospy.init_node('era-5g-action-client-script')


def feedback_cb(msg):
    print('Feedback received: '+str(msg))

def goal():
        client = actionlib.SimpleActionClient('goal_5g', goal_5gAction)
        taskId = "11071d4d-d1ae-4e55-8de2-e562c6078277"  # Example of task that the robot wants to execute - make sure the taskid is in redis.

        goal = goal_5gGoal
        goal.goal_taskid =taskId
        goal.action_reference =0
        #goal = era_5g_action_client_ros1.msg.5gGoal((taskId,0))
        rospy.loginfo('Waiting for 5g-era-server...')
        client.wait_for_server()

        client.send_goal(goal,feedback_cb=feedback_cb)


if __name__ == '__main__':
    try:
        rospy.init_node('era_5g_action_client_script')
        rospy.loginfo('Running 5g-era-client...')
        goal()
    except rospy.ROSInterruptException:
        pass
