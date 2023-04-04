#!/usr/bin/env python3

__author__ = "Adrian Lendinez Ibañez"
__licence__ = "GPL"
__version__ = "1.0"
__manteiner__ = "Adrian Lendinez Ibañez"
__email__ = "adrian.lendinez@outlook.com"
__status__ = "Testing"
__date__ = "03/04/2023"

import json
import os
import subprocess
import time
from collections import OrderedDict
import collections
# Json file content:

{
      "name": "",
      "type": "",
      "description": "",
      "nodes": [
      ],
      "number": 1
    }

sensor_dict = {
    "Name": "",
    "Type": "",
    "Description": "",
    "Nodes": [],
    "number": 1
}

actuator_dict = {
    "Name": "",
    "Type": "",
    "Number": 1,
    "Nodes": []
}

robot_onboarding_dict = [("relations", []),
("Id", ""),
("Name", ""),
("RosVersion", ""),
("RosDistro", ""),
("MaximumPayload", ""),
("MaximumTranslationalVelocity", ""),
("MaximumRotationalVelocity", ""),
("RobotWeight", ""),
("ROSRepo", ""),
("ROSNodes", []),
("Manufacturer", ""),
("ManufacturerUrl", ""),
("RobotModel", ""),
("RobotStatus", ""),
("currentTaskId", ""),
("TaskList", ""),
("BatteryStatus", 0),
("MacAddress", ""),
("LocomotionSystem", ""),
("LocomotionTypes", ""),
("Sensors", [sensor_dict]),
("Actuator", [actuator_dict]),
("Manipulators", []),
("CPU", 1),
("RAM",1),
("StorageDisk", 8),
("NumberCores", 1),
("Questions",[]) 
]
'''
robot_onboarding_dict = {
    "relations": [],
    "Id": "",
    "Name": "",
    "ROSRepo": "",
    "ROSNodes": [],
    "Manufacturer": "",
    "ManufacturerUrl": "",
    "RobotModel": "",
    "RobotStatus": "",
    "TaskList": "",
    "BatteryStatus": 0,
    "MacAddress": "",
    "LocomotionSystem": "",
    "LocomotionTypes": "",
    "Sensors": [sensor_dict],
    "Actuator": [actuator_dict],
    "Manipulators": [],
    "CPU": 1,
    "RAM": 1,
    "StorageDisk": 8,
    "NumberCores": 1,
    "Questions": []
}
'''
robot_onboarding_dict = collections.OrderedDict(robot_onboarding_dict)


'''
Save a dictionary to json file format
'''
def save_json(robot_onboarding_dict):
    try:
        with open('robot_onboarding_v1.json', 'w') as outfile:
            json.dump(robot_onboarding_dict, outfile,sort_keys=False, indent=4)
    except Exception as e:
        print("Could not save to json file. "+str(e)) 

'''
Run a linux command and save content to file if desired.
'''
def run_linux_command(command, save = False):
    if save == False:
        output = subprocess.Popen([command], shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = output.communicate()
        #if stderr != "": raise Exception('The Following command: ' + str(command) + "is giving the error: " +str(stderr))
        
        return stdout.decode("utf-8")
    else:
        pr = open("output.txt", "w")
        subprocess.Popen(command, stdout=pr, shell=True)
        pr.close()
        time.sleep(1)
        content = read_file('output.txt')
        return content

'''
Read a txt file in current execution directory.
'''
def read_file(file_name):
    with open(file_name, 'r') as f:
        try:
            lines = f.readlines()
            f.close()
            return lines
        except:
            f.close()
            raise Exception('Could not read file')

'''
Parser an array of lines with the content of ros command: rosnode info -node_name-
'''
def parser_node_info(node_info: list):
    def clean():
        
        node_info.pop(0) # Remove first item of list
        pos = [i for i, s in enumerate(node_info) if 'contacting' in s]
        print("pos: "+str(pos[0]))
        del node_info[pos[0]:] # Remove all after "contacting ..."
        print("node_info: "+str(node_info))
        return node_info
    publishers = []
    subscribers = []
    services = []
    node_info = clean()
    node_name = node_info[0][node_info[0].find("[")+2:node_info[0].find("]")]
    print("node name parsed: "+str(node_name))
    
    # Complete publications topics info.
    if "Publications:" in node_info[1]:
        pos = [i for i, s in enumerate(node_info) if 'Subscriptions' in s]
        num_pub_topics = pos[0] - 3
        print("Number of pub topics: "+str(num_pub_topics))
        print("node_info again: "+str(node_info))       
        for x in range(0, num_pub_topics):
            square_braket_open =  [i for i, s in enumerate(node_info[x+2]) if '[' in s]
            print("name of topic: "+str((node_info[x+2].replace("*","")[:square_braket_open[0]-1])).strip())
            print("type of topic: "+str((node_info[x+2].replace("*","")[square_braket_open[0]-1:])).strip())
            publishers.append(
                {
                "Name": str((node_info[x+2].replace("*","")[:square_braket_open[0]-1])).strip(),
                "Type": str((node_info[x+2].replace("*","")[square_braket_open[0]-1:])).strip(),
                "Description": ""
            })
            print("==========================")
    
    if "Subscriptions:" in node_info[[i for i, s in enumerate(node_info) if 'Subscriptions' in s][0]]: 
        sub_pose = [i for i, s in enumerate(node_info) if 'Subscriptions' in s][0]
        num_sub_topics = ([i for i, s in enumerate(node_info) if 'Services' in s][0] - sub_pose)-2
        print("Number of sub topics: "+str(num_sub_topics))
        for x in range(0, num_sub_topics):
            square_braket_open =  [i for i, s in enumerate(node_info[x+sub_pose+1]) if '[' in s]
            print("name of sub topic: "+str((node_info[x+sub_pose+1].replace("*","")[:square_braket_open[0]-1])).strip())
            print("Type of sub topic: "+str((node_info[x+sub_pose+1].replace("*","")[square_braket_open[0]-1:])).strip())
               
            subscribers.append(
                {
                "Name": str((node_info[x+sub_pose+1].replace("*","")[:square_braket_open[0]-1])).strip(),
                "Type": str((node_info[x+sub_pose+1].replace("*","")[square_braket_open[0]-1:])).strip(),
                "Description": ""
            })
        
    
    if "Services:" in node_info[[i for i, s in enumerate(node_info) if 'Services' in s][0]]:
        service_pos = [i for i, s in enumerate(node_info) if 'Services' in s][0]
        print("Service pos: "+str(service_pos))
        num_services = len(node_info[service_pos:])-3 # lo que sea -2
        print("Number of services: "+str(num_services))
        for x in range(0, num_services):
            print("service name: "+str(str(node_info[x+service_pos+1]).replace("*","").strip()))
            services.append(
                {
                "Name": str(str(node_info[x+service_pos+1]).replace("*","").strip()),
                "Description": ""
            })
    
    rosNode_dict = {
    "Name": str(node_name),
    "Publications": publishers,
    "Subscriptions": subscribers,
    "Services": services
    }
    return rosNode_dict

'''
Main execution
'''

# Get ROS version
ros_distro_cmd = run_linux_command('echo $ROS_DISTRO')
print("ROS DISTRO: "+str(ros_distro_cmd))
# Allow bash script to use ROS commands.
source_ros_cmd = 'source /opt/ros/'+str(ros_distro_cmd)+'/setup.bash'
run_linux_command(source_ros_cmd)

# Get a list of ros nodes running
num_ros_nodes = run_linux_command('rosnode list | grep -c ""')
print("Number of running ROS nodes: "+str(num_ros_nodes))


# Iterate over all the ros nodes
for node_num in range(1, int(num_ros_nodes)):
    
    # Get the name of the ros node
    ros_node_name = run_linux_command('rosnode list | grep "" | sed -n '+str(node_num)+'\p')
    print("ros_node_name: ", ros_node_name)
    # Get the information about this ros node
    ros_node_info = run_linux_command('rosnode info '+str(ros_node_name), True)
    #print("ros_node_info: ", str(ros_node_info))
    # Append to python dict
    robot_onboarding_dict['ROSNodes'].append(parser_node_info(ros_node_info))
    
save_json(robot_onboarding_dict)
