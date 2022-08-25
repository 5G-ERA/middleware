#IMPORTS
import requests
from requests.exceptions import HTTPError
import json
import os
#GLOBAL VARIABLES 
global username
username='97f4059f-acfa-48f9-b22c-165ecbc51ed1'
#username='a9fdafe6-9b90-48bc-a286-3f08ed0a78aa'
global password
password='5g-era'
global Token
Token=''
global plan
plan=''
global ActionPlanId
ActionPlanId = ''

#task_id='204edfa7-ca2b-43d2-9298-f5b3883aaf27'
#distributed
#task_id='a9fdafe6-9b90-48bc-a286-3f08ed0a78aa'
#standalone
task_id='72d9e9ee-e261-41f8-8135-1e5cc0db13f4'

#Last update: 16-05-2022

class ActionClient_5g():
    def __init__(self):
        print("5G-ERA MIDDLEWARE INTERFACE DEFINITION")
        print("======================================")
        # This interface usese the python library request. If it is not installed in your system, do so by launching this command in the terminal --> pip install requests

    def login(self): # Login function to middleware
        try:
            global Token
            #r = requests.post('http://10.64.140.43/Login', json={"Id": username, "Password": password})
            r = requests.post('http://192.168.20.220/Login', json={"Id": username, "Password": password})
            print(f"Status Code: {r.status_code}, Response: {r.json()}")
            Token = jsondata = r.json()
            newToken=(Token['token'])
            Token=newToken

        except HTTPError as e:
            print(e.response.status_code)
        
    def getPlan(self,taskId): # Get a plan created by middleware for a task
        try:
            global Token
            hed = {'Authorization': 'Bearer ' + str(Token)}
            data = {"TaskId": taskId}
            #url = "http://10.64.140.43/Task/Plan"
            url = "http://192.168.20.220/Task/Plan"
            response = requests.post(url, json=data, headers=hed)
            print(response.json())

            Action_Sequence = jsondata = response.json()
            actionseq = (Action_Sequence['ActionSequence'])
            global ActionPlanId
            ActionPlanId = (Action_Sequence['ActionPlanId'])

        except HTTPError as e:
            print(e.response.status_code)

    def getResourceStatus(self,ActionPlanId): # Get the status of the resources deployed.
        try:
            global Token
            hed = {'Authorization': 'Bearer ' + str(Token)}
            #url = "http://10.64.140.43/orchestrate/orchestrate/plan/"+str(ActionPlanId)
            url = "http://192.168.20.220/orchestrate/orchestrate/plan/"+str(ActionPlanId)
            response = requests.get(url, headers=hed)
            print(response.json())
        except HTTPError as e:
            print(e.response.status_code)

    def DeleteResources(self,ActionPlanId): # Delete all the resources alocated to a particular ActionPLan
        try:
            global Token
            hed = {'Authorization': 'Bearer ' + str(Token)}
            #url = "http://10.64.140.43/orchestrate/orchestrate/plan/"+str(ActionPlanId)
            url = "http://192.168.20.220/orchestrate/orchestrate/plan/"+str(ActionPlanId)
            response = requests.delete(url, headers=hed)
            if '200' in str(response):
                print("Deleted succesfully")
            else:
                print("Could not delete the resource. ",response)
        except HTTPError as e:
            print(e.response.status_code)
   

            
def main():
    actionClient=ActionClient_5g() # Create an object of the class
    print("=============LOGIN==================")
    actionClient.login() # Login
    print("=============GET PLAN===============")
    actionClient.getPlan(task_id) # Get plan
    print("==========GET RESOURCE STATUS=======")
    actionClient.getResourceStatus(ActionPlanId) # Get resource update
    print("===========DELETE RESOURCES=========")
    #actionClient.DeleteResources(ActionPlanId) # Delete resources
    print("====================================")
    


if __name__ == '__main__':
    main()

    
