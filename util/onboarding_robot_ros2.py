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
        
        cleaned_list = []
        pos = [i for i, s in enumerate(node_info) if 'Service Clients:' in s]
        print("pos: "+str(pos[0]))
        del node_info[pos[0]:] # Remove all after "contacting ..."
        #print("node_info: "+str(node_info))
        for element in node_info:
            cleaned_list.append(element.replace('\n','').strip())
            
               
        
        return cleaned_list
    publishers = []
    subscribers = []
    services = []
    node_info = clean()
    print(node_info)
    node_name = node_info[0]
    print("node name parsed: "+str(node_name))
    index_pub = node_info.index('Publishers:')
    index_service = node_info.index('Service Servers:')
    
    if 'Service Servers:' == str(node_info[index_service]):
         print('service in')
         num_service = len(node_info[index_service:])
         print('num service'+str(num_service))
         print('======================')
         for x in range(0,num_service-1):
             #print(node_info[index_service+x])
             if  num_service ==1:
               print(node_info[index_service+1])
               delimiter = node_info[index_service+1].index(':')
               print(node_info[index_service+1][:delimiter])
               print(node_info[index_service+1][delimiter+2:])
               
               services.append(
                {
                "Name": str(node_info[index_service+1][:delimiter]),
                "Type": str(node_info[index_service+1][delimiter+2:]),
                "Description": ""
                })
             else:
                    print(node_info[index_service+x+1])
                    delimiter = node_info[index_service+x+1].index(':')
                    print(node_info[index_service+x+1][:delimiter])
                    print(node_info[index_service+x+1][delimiter+2:])
                    
                    services.append(
                {
                "Name": str(node_info[index_service+x+1][:delimiter]),
                "Type": str(node_info[index_service+x+1][delimiter+2:]),
                "Description": ""
            })

         
    
    if 'Publishers:' == str(node_info[index_pub]):
         service_index = node_info.index('Service Servers:')
         print('service_index'+str(service_index))
         num_pub = service_index - index_pub -1
         print('pub num: '+str(num_pub))
         print('inicio '+str(index_pub))
         
         for x in range(0, num_pub):
            if  num_pub ==1:
               print(node_info[index_pub+1])
               delimiter = node_info[index_pub+1].index(':')
               print(node_info[index_pub+1][:delimiter])
               print(node_info[index_pub+1][delimiter+2:])
               
               publishers.append(
                {
                "Name": str(node_info[index_pub+1][:delimiter]),
                "Type": str(node_info[index_pub+1][delimiter+2:]),
                "Description": ""
            })
            else:
                    print(node_info[index_pub+x+1])
                    delimiter = node_info[index_pub+x+1].index(':')
                    print(node_info[index_pub+x+1][:delimiter])
                    print(node_info[index_pub+x+1][delimiter+2:])
                    
                    publishers.append(
                {
                "Name": str(node_info[index_pub+x+1][:delimiter]),
                "Type": str(node_info[index_pub+x+1][delimiter+2:]),
                "Description": ""
            })

    
    
    if 'Subscribers:' == str(node_info[1]).strip():
        
        pub_index = node_info.index('Publishers:')
        num_sub_topics = pub_index -2
        print(num_sub_topics)
        
        print("Number of sub topics: "+str(num_sub_topics))
        for x in range(0, num_sub_topics):
            if  num_sub_topics ==1:
               print(node_info[num_sub_topics+1])
               delimiter = node_info[num_sub_topics+1].index(':')
               print(node_info[num_sub_topics+1][:delimiter])
               print(node_info[num_sub_topics+1][delimiter+2:])
               subscribers.append(
                {
                "Name": str(node_info[num_sub_topics+1][:delimiter]),
                "Type": str(node_info[num_sub_topics+1][delimiter+2:]),
                "Description": ""
            })
            else:
                    print(node_info[num_sub_topics+x])
                    delimiter = node_info[num_sub_topics+x].index(':')
                    print(node_info[num_sub_topics+x][:delimiter])
                    print(node_info[num_sub_topics+x][delimiter+2:])
                    subscribers.append(
                {
                "Name": str(node_info[num_sub_topics+x][:delimiter]),
                "Type": str(node_info[num_sub_topics+x][delimiter+2:]),
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
num_ros_nodes = run_linux_command('ros2 node list -c')
print("Number of running ROS nodes: "+str(num_ros_nodes))


# Iterate over all the ros nodes
for node_num in range(1, int(num_ros_nodes)+1):
    time.sleep(3)
    # Get the name of the ros node
    ros_node_name = run_linux_command('ros2 node list | grep "" | sed -n '+str(node_num)+'\p')
    if 'WARNING' in ros_node_name:
        
        index_error = ros_node_name.index('effects')
        ros_node_name = ros_node_name[index_error+8:].strip()
   	
    print(ros_node_name)
    print("ros_node_name: ", ros_node_name)
    
    # Get the information about this ros node
    ros_node_info = run_linux_command('ros2 node info '+str(ros_node_name), True)
    print("ros_node_info: ", str(ros_node_info))
    if len(ros_node_info)==0:
         print('rerun query')
         ros_node_info = run_linux_command('ros2 node info '+str(ros_node_name), True)
    # Append to python dict
    robot_onboarding_dict['ROSNodes'].append(parser_node_info(ros_node_info))
    
save_json(robot_onboarding_dict)
