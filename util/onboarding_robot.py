#!/usr/bin/env python3

__author__ = "Adrian Lendinez Ibañez"
__licence__ = "GPL"
__version__ = "1.0"
__manteiner__ = "Adrian Lendinez Ibañez"
__email__ = "adrian.lendinez@outlook.com"
__status__ = "Testing"
__date__ = "19/01/2023"

import json
import os
import subprocess
import time

# Json file content:

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
Save a dictionary to json file format
'''
def save_json():
    try:
        with open('robot_onboarding_v1.json') as outfile:
            json.dump(robot_onboarding_dict, outfile)
    except Exception as e:
        print("Could not save to json file. "+str(e)) 

'''
Run a linux command and save content to file if desired.
'''
def run_linux_command(command, save = False):
    if save == False:
        output = subprocess.Popen([command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = output.communicate()
        if stderr != "": raise Exception('The Following command: ' + str(command) + "is giving the error: " +str(stderr))
        return stdout[0]
    else:
        cmd = 'command '+' > output.txt'
        os.system(cmd) 
        time.sleep(1)
        content = read_file(output.txt)
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
        del node_info[node_info.find("contacting"):] # Remove all after "contacting ..."
        return node_info
    publishers = []
    subscribers = []
    services = []
    node_info = clean()
    node_name = node_info[0][node_info[0].find("["):node_info[0].find("]")]
    # Complete publications topics info.
    if "Publications:" in node_info[1]:
        num_pub_topics = node_info.find("Subscriptions") - 3
        for x in range(0, num_pub_topics):
            publishers.append(
                publisher_dict = {
                "Name": str(str(node_info[x][:node_info[x].find("[")])).replace("*",""),
                "Type": str(node_info[x][node_info[x].find("["):node_info[x].find("]")]).replace("*",""),
                "Description": ""
            })
    if "Subscriptions:" in node_info:
        num_sub_topics = node_info.find("Services") -1
        for x in range(0, num_sub_topics):
            subscribers.append(
                subscriber_dict = {
                "Name": str(str(node_info[x][:node_info[x].find("[")])).replace("*",""),
                "Type": str(node_info[x][node_info[x].find("["):node_info[x].find("]")]).replace("*",""),
                "Description": ""
            })
    if "Services:" in node_info:
        num_services = node_info[-1]-1
        for x in range(0, num_services):
            services.append(
                service_dict = {
                "Name": str(node_info[x]).replace("*",""),
                "Description": ""
            })

    rosNode_dict = {
    "Name": str(node_name),
    "Publications": [publishers],
    "Subscriptions": [subscribers],
    "Services": [services]
    }
    return rosNode_dict

'''
Main execution
'''

# Get ROS version
ros_distro_cmd = run_linux_command('echo $ROS_DISTRO')

# Allow bash script to use ROS commands.
source_ros_cmd = 'source /opt/ros/'+str(ros_distro_cmd)+'/setup.bash'
run_linux_command(source_ros_cmd)

# Get a list of ros nodes running
num_ros_nodes = run_linux_command('rosnode list | grep -c ""')

# Iterate over all the ros nodes
for node_num in range(0, int(num_ros_nodes)):

    # Get the name of the ros node
    ros_node_name = run_linux_command('rosnode list | grep "" | sed -n '+node_num+'\p')

    # Get the information about this ros node
    ros_node_info = run_linux_command('rosnode info '+str(ros_node_name), True)

    # Append to python dict
    robot_onboarding_dict['ROSNodes'].append(parser_node_info(ros_node_info))
    
save_json(robot_onboarding_dict)