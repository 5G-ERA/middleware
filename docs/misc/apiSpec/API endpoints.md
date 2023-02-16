# External Interface
This document presents the endpoints that are accessible in the middleware. This document covers the path to the endpoints, what is their functionality, what parameters they take and what information is returned alongside the status codes. 

All the endpoints except the authentication ones require the client to be authenticated. The authentication scheme utilizes the JWT Bearer token. The token is retrieved after logging in using the `/Login` endpoint. The token has to be passed in the `Authentication` header for each request to be allowed.

## Authentication

### POST /Register
Registers the robot into the Middleware system. Task to be performed by the Administrator.

Param:
```json 
{
   "Id":"3cace17e-a091-4f91-ad98-7d0c371b3a25",
   "Password":"password"
}
```
Return:
* 201 - after successful registration
* 400 - when parameters are not specified
* 500 - when the server experiences an error


### POST /Login
Authenticates the Robot in the system. When authenticated, the Robot is given the JWT Token to be used as a Bearer authentication scheme thought the usage of the system.
Param:
```json 
{
   "Id":"3cace17e-a091-4f91-ad98-7d0c371b3a25",
   "Password":"password"
}
```
Return:
* 200 - user has ben authenticated. Return body:
```json
{
     "token":"JWTBearerToken",
     "expirationDate": "2022-05-09T22:35:56.8472623Z"
}
```
* 400 - Credentials were not provided
* 401 - User have not ben authorized, wrong credentials

## Task planner
Task planner is responsible for the creation of the task and its initialization in the Middleware. It handles the full pipeline, from the given taskId for the execution to the deployment of the task.

### GET /PLAN/{param}

Asking: ROBOT ANSWER: 5G-ERA.

(Assumptions: robot knows the task_id for a high level action for example `grab the medicine`).

The ROBOT asks for help from 5G-ERA system. 

Param: 
```json
{
  "Id": "guid",
  "TaskId" : "task_id",
  "Questions": [
    {
    "Id": "guid",
    "Question": "Do you have a map?",
    "Answer": "True/false"
    }
  ]
}
```

Return: 
* 200 - the status of the plan to be deployed, including the latest deployment information of the actions in the task. The example body:

```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "Id": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
        "ServiceType": "Object detection/SLAM",   
        "ServiceUrl": "https://...../......",
        "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
   } 
  ]
}
```
* 404 - the plan definition has not been found


## Orchestrator

This is the proposed endpoint structure for the Orchestrator. The endpoints should allow for the communication with the necessary features of the OSM and apply the plans for the 5G-ERA Middleware.

Actions in the understanding of the 5G-ERA are the steps that the robot will have to conduct to finish the specified task.

### POST /orchestrate/orchestrate/plan/{plan}

Executes resources for the specified plan
Asking: Action Planner Response: Orchestrator

**Param**: plan - List of resources to be instantiated
```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "Id": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]      
   } 
  ]
}

```

**Return**:
* 200 - plan has ben updated and instantiated. Returns the body:
```json
{
  "Id": "TASK_NUMBER",
  "TaskPriority": "HIGH/MEDIUM/LOW",
  "ActionPlanId": "guid",
  "ActionSequence": [
   {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
   } 
  ]
}
```
* 400 - the plan to be deployed have not been specified
* 500 - The error occurred while deploying the services

### GET orchestrate/orchestrate/plan/{plan_id}

Get the status of services deployed with this plan

**Param**: id - GUID id of the plan

**Return**:
* 200 - the status of the services have been retrieved. The body with the statuses:
```json
{
    "ActionPlanId":"50a85398-21b5-4604-8c6f-d510d2abbfd3",
    "ActionSequence":[
    {
      "ActionId": 2,
      "Order": 0,
      "Placement": "EDGE/CLOUD",
      "ActionPriority": "1/2/3",
      "Services": [
        {
          "Id": "guid",
          "Image name": "Object detection service",
          "ServiceInstanceId": "guid",
          "ServiceType": "Object detection/SLAM",
          "IsReusable": true,
          "DesiredStatus": "created",        
          "ServiceUrl": "https://...../......",
          "ServiceStatus": "Active/Down/Instantiating/Idle/Terminating"
        }
      ]
    }]
}
```
* 400 - the plan to be deployed have not been specified
* 404 - the plan with the specified Id have not been found
* 500 - The error occurred while deploying the services

Status: 200 (OK), 404(Not Found)

### DELETE /orchestrate/action/{id}

Terminate services for specified action plan.

Param:
* **id** - the Identifier of the action plan to be deleted

Returns:
* 200 - plan has been deleted
* 400 - the Id of the plan has not been specified
* 404 - The plan has not been found
* 500 - There was an error while deleting the plan


## RedisInterface API

RedisInterface API allows full CRUD operations (GET/POST/PUT/PATCH/DELETE) on all data models that are embebed in the system. 

### GET Data/Action
Get all the actions.

Return:

* 200 - Success, list of Actions.
```json 
[
  {
    "Id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "Name": "move-base",
    "ActionFamily": "movment",
    "Order": 10,
    "Placement": "cloud",
    "ActionPriority": "medium"
  },
  {
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "edge",
    "ActionPriority": "low"
  }
  
]
```
* 404 - Actions were not found.
* 500 - An error occurred.

### POST Data/Action
Add new action entity.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "ActionFamily": "string",
  "Order": 0,
  "Placement": "string",
  "ActionPriority": "string"
}
```

Return:

* 200 - Success, newly created action entity.
```json 
{
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "edge",
    "ActionPriority": "low"
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/{id}
Get action entity by id.

Param:

* id -  provide the guid id for the desired action entity to be retrieved ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

Return:

* 200 - Success, the specified action entity.
```json
{
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "edge",
    "ActionPriority": "low"
}
```
* 404 - Action was not found.
* 500 - An error occurred.

### PATCH Data/Action/{id}
Modify an existing action entity.

Param:

* id - provide the guid id for the desired action entity to be modified ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

```json
{
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "cloud",
    "ActionPriority": "high"
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
    "Id": "425b46eb-eaea-45e0-8769-4926159f9c87",
    "Name": "detect-botle",
    "ActionFamily": "detection",
    "Order": 11,
    "Placement": "cloud",
    "ActionPriority": "high"
}
```
* 404 - Action entity was not found.
* 500 - An error occurred.

### DELETE Data/Action/{id}
Delete an action entity.

Param:

* id -  provide the guid id for the desired action entity to be deleted ex. 425b46eb-eaea-45e0-8769-4926159f9c87.

Return:

* 200 - Success, true.
* 404 - Action entity was not found.
* 500 - An error occurred.

### POST Data/Action/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```
* 400 - Bad Request parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired action entity relation to be retrieved ex. e5ebd4bbe-9c75-46df-a4f4-93c7dea10373.
* name - provide the relation name ex. NEEDS.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "5ebd4bbe-9c75-46df-a4f4-93c7dea10373",
    "type": "Action",
    "name": "move-base"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Action/relations/{firstName}/{secondName}
Currently, Action has single relation use case scenario.

### GET Data/Action/plan
Get all the plans.

Return:

* 200 - Success, list of plans.
```json
[
  {
    "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
    "name": "TestTask",
    "actionSequence": [
      {
        "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
        "name": "move-base",
        "actionFamily": "movement",
        "order": 1,
        "placement": "cloud",
        "actionPriority": "low",
        "services": null,
        "relations": []
      },
      {
        "id": "7e085e54-2c67-473a-9da9-732469d265bf",
        "name": "detect-botle",
        "actionFamily": "detection",
        "order": 2,
        "placement": "cloud",
        "actionPriority": "medium",
        "services": null,
        "relations": []
      }
    ],
    "relations": []
  }
]
```
* 404 - Plans were not found.
* 500 - An error occurred.

### POST Data/Action/plan
Add new plan.

Param:

```json
{
  "Id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "Name": "TestTask",
  "ActionSequence": [
    {
      "Id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "Name": "move-base",
      "ActionFamily": "movement",
      "Order": 1,
      "Placement": "cloud",
      "ActionPriority": "low"
    },
     {
      "Id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "Name": "detect-botle",
      "ActionFamily": "detection",
      "Order": 2,
      "Placement": "cloud",
      "ActionPriority": "medium"
    }
  ]
}
```

Return:

* 200 - Success, plan was created.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Action/plan/{id}
Get plan by id.

Param:

* id - provide the guid id for the desired action plan to be retrieved ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

Return:

* 200 - Success, the specified plan.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 404 - Plan was not found.
* 500 - An error occurred.

### DELETE Data/Action/plan/{id}
Delete plan by id.

Param:

* id - provide the guid id for the desired action plan to be deleted ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

Return:

* 200 Success, plan was deleted.
* 404 Plan was not found.
* 500 An error occurred.

### PUT Data/Action/plan/{id}
Update an existing plan.

Param:

* id - provide the guid id for the desired action plan to be updated ex. 97bb19b1-042c-408e-b1d9-2a2ebbe05b66.

```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```

Return:

* 200 Success, the plan was updated.
```json
{
  "id": "97bb19b1-042c-408e-b1d9-2a2ebbe05b66",
  "name": "TestTask",
  "actionSequence": [
    {
      "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
      "name": "move-base",
      "actionFamily": "movement",
      "order": 1,
      "placement": "cloud",
      "actionPriority": "low",
      "services": null,
      "relations": []
    },
    {
      "id": "7e085e54-2c67-473a-9da9-732469d265bf",
      "name": "detect-botle",
      "actionFamily": "detection",
      "order": 2,
      "placement": "cloud",
      "actionPriority": "medium",
      "services": null,
      "relations": []
    }
  ],
  "relations": []
}
```
* 400 Bad Request, parameters were not specified correctly.
* 404 The plan was not found.
* 500 An error occurred.

### GET Data/Cloud
Get all the clouds.

Return:

* 200 - Success, list of Clouds.
```json
[
  {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "name": "Cloud_1",
    "cloudStatus": "Running",
    "cloudIp": "192.168.1.1",
    "relations": []
  },
  {
    "id": "186e9c60-4682-4ec0-85b4-9284f39a381d",
    "name": "Cloud_2",
    "cloudStatus": "Running",
    "cloudIp": "192.168.1.2",
    "relations": []
  }
]
```
* 404 - Clouds were not found.
* 500 - An error occurred.

### POST Data/Cloud
Add new cloud.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "CloudStatus": "string",
  "CloudIp": "string"
}
```

Return:

* 200 Success, newly created cloud entity
```json
{
  "id": "186e9c60-4682-4ec0-85b4-9284f39a381d",
  "name": "Cloud_2",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.2",
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Cloud/{id}
Get cloud by id.

Param:

* id - provide the guid id for the desired cloud entity to be retrieved ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.1",
  "relations": []
}
```
* 404 - Cloud was not found.
* 500 - An error occurred.

### PATCH Data/Cloud/{id}
Modify an existing cloud entity.

Param:

* id - provide the guid id for the desired cloud entity to be modified ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

```json
{
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.1",
  "relations": []
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
  "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
  "name": "Cloud_1",
  "cloudStatus": "Running",
  "cloudIp": "192.168.1.5",
  "relations": []
}
```
* 404 - Cloud was not found.
* 500 - An error occurred.

### DELETE Data/Cloud/{id}
Delete a cloud entity.

Param:

* id - provide the guid id for the desired cloud entity to be deleted ex. e56e1928-03a0-43e9-b999-318198c6d7ca.

Return:

* 200 - Success, cloud was deleted.
* 404 - Cloud was not found.
* 500 - An error occurred.

### POST Data/Cloud/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Cloud/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired Cloud entity relation to be retrieved ex. e56e1928-03a0-43e9-b999-318198c6d7ca.
* name - provide the relation name ex. WORKING_FOR.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "e56e1928-03a0-43e9-b999-318198c6d7ca",
    "type": "Cloud",
    "name": "Cloud_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Cloud/relations/{firstName}/{secondName}
Currently, Cloud has single relation use case scenario.

### GET Data/ContainerImage
Get all containers.

Return:

* 200 - Success, list of containers.
```json
[
  {
    "id": "1ae73495-01f3-4273-8fd7-003c941a8491",
    "name": "my-image-8",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-8",
    "k8SDeployment": "my-deployment-8",
    "k8SService": "mk8s",
    "relations": []
  },
  {
    "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
    "name": "my-image-9",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-9",
    "k8SDeployment": "my-deployment-9",
    "k8SService": "mk8s",
    "relations": []
  }
]
```
* 404 - Containers were not found.
* 500 - An error occurred.

### POST Data/ContainerImage
Create a new container entity.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "Timestamp": "2022-05-12T14:22:17.702Z",
  "Description": "string",
  "K8SDeployment": "string",
  "K8SService": "string"
}
```

Return:

* 200 - Success, newly created container entity.
```json 
{
  "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "mk8s",
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/ContainerImage/{id}
Get container by id.

Param:

* id - provide the guid id for the desired container entity to be retrieved ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "mk8s",
  "relations": []
}
```
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### PATCH Data/ContainerImage/{id}
Modify an existing container entity.

Param:

* id - provide the guid id for the desired container entity to be modified ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

```json
{
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "kubernetes",
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
   "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
  "name": "my-image-9",
  "timestamp": "2022-03-11T12:09:43.5122616+00:00",
  "description": "my-description-9",
  "k8SDeployment": "my-deployment-9",
  "k8SService": "kubernetes",
  "relations": []
}
```
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### DELETE Data/ContainerImage/{id}
Delete a container entity.

Param:

* id -  provide the guid id for the desired container entity to be deleted ex. 92e97d88-d7f1-4d17-959f-b7e03083a547.

Return:

* 200 - Success, container was deleted.
* 404 - ContainerImage entity was not found.
* 500 - An error occurred.

### POST Data/ContainerImage/AddRelation
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/relation/{name}
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/relations/{firstName}/{secondName}
Currently the ContainerImage does not support relations to initiate from.

### GET Data/ContainerImage/instace/{id}
Get the container images associated with the specified instance.

Param:

* id - provide the guid id for the instance entity associated with the container images ex. d3f31c7e-8b6c-43d7-9cb5-99838ed27880.

Return:

* 200 Success list of container images
```json
[
  {
    "id": "92e97d88-d7f1-4d17-959f-b7e03083a547",
    "name": "my-image-9",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "description": "my-description-9",
    "k8SDeployment": "my-deployment-9",
    "k8SService": "mk8s",
    "relations": []
  }
]
```
* 404 - Container images were not found.

### GET Data/Edge
Get all the edges.

Return:

* 200 - Success, list of Edges.
```json
[
  {
  "Id": "f686d326-ca3b-4e69-a687-5176232b4668",
  "Name": "Edge_1",
  "EdgeStatus": "Running",
  "EdgeIp": "192.168.1.2",
  "MacAddress": "9B9656CCEE6C",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 90,
  "NumberOfCores": 3
  },
  {
  "Id": "bfcf00e1-1044-44b5-abca-50d28bc6af44",
  "Name": "Edge_2",
  "EdgeStatus": "WithoutBattery",
  "EdgeIp": "192.168.1.3",
  "MacAddress": "56ECE51DB5ED",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 80,
  "NumberOfCores": 2
  }
]
```
* 404 - Edges were not found.
* 500 - An error occurred.

### POST Data/Edge
Add new edge.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "EdgeStatus": "string",
  "EdgeIp": "string",
  "MacAddress": "string",
  "CPU": 0,
  "RAM": 0,
  "VirtualRam": 0,
  "DiskStorage": 0,
  "NumberOfCores": 0
}
```

Return:

* 200 Success, newly created edge entity
```json
{
  "Id": "f686d326-ca3b-4e69-a687-5176232b4668",
  "Name": "Edge_1",
  "EdgeStatus": "Running",
  "EdgeIp": "192.168.1.2",
  "MacAddress": "9B9656CCEE6C",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 90,
  "NumberOfCores": 3
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Edge/{id}
Get Edge by id.

Param:

* id - provide the guid id for the desired Edge entity to be retrieved ex. f686d326-ca3b-4e69-a687-5176232b4668.

Return:

* 200 - Success, the specified entity.
```json
{
  "Id": "f686d326-ca3b-4e69-a687-5176232b4668",
  "Name": "Edge_1",
  "EdgeStatus": "Running",
  "EdgeIp": "192.168.1.2",
  "MacAddress": "9B9656CCEE6C",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 90,
  "NumberOfCores": 3
}
```
* 404 - Edge was not found.
* 500 - An error occurred.

### PATCH Data/Edge/{id}
Modify an existing edge entity.

Param:

* id - provide the guid id for the desired edge entity to be modified ex. f686d326-ca3b-4e69-a687-5176232b4668.

```json
{
  "Name": "Edge_1",
  "EdgeStatus": "Running",
  "EdgeIp": "192.168.1.2",
  "MacAddress": "9B9656CCEE6C",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 100,
  "NumberOfCores": 5
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
  "Id": "f686d326-ca3b-4e69-a687-5176232b4668",
  "Name": "Edge_1",
  "EdgeStatus": "Running",
  "EdgeIp": "192.168.1.2",
  "MacAddress": "9B9656CCEE6C",
  "CPU": 90,
  "RAM": 90,
  "VirtualRam": 90,
  "DiskStorage": 100,
  "NumberOfCores": 5
}
```
* 404 - Edge was not found.
* 500 - An error occurred.

### DELETE Data/Edge/{id}
Delete an Edge entity.

Param:

* id - provide the guid id for the desired edge entity to be deleted ex. f686d326-ca3b-4e69-a687-5176232b4668.

Return:

* 200 - Success, edge was deleted.
* 404 - Edge was not found.
* 500 - An error occurred.

### POST Data/Edge/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "f686d326-ca3b-4e69-a687-5176232b4668",
    "type": "Edge",
    "name": "Edge_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "f686d326-ca3b-4e69-a687-5176232b4668",
    "type": "Edge",
    "name": "Edge_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Edge/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired Edge entity relation to be retrieved ex. f686d326-ca3b-4e69-a687-5176232b4668.
* name - provide the relation name ex. WORKING_FOR.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "f686d326-ca3b-4e69-a687-5176232b4668",
    "type": "Edge",
    "name": "Edge_1"
  },
  "relationName": "WORKING_FOR",
  "pointsTo": {
    "id": "5d82d55d-03ab-4ee1-ba4e-529c4949cbdc",
    "type": "Action",
    "name": "move-base"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Edge/relations/{firstName}/{secondName}
Currently, Edge has single relation use case scenario.

### GET Data/Health
Check for the functionality of the RedisInterface API.

Return:

* 200 Success, RedisInterface API is functional.

### GET Data/Instance
Get all the instances.

Return:

* 200 - Success, list of Instances.
```json
[
 {
  "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
  "name": "my-test-instance-8",
  "imageName": "my-image-name-8",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-8.rdu",
  "serviceStatus": "up",
  "containerImage": null,
  "relations": []
 },
 {
  "id": "d3f31c7e-8b6c-43d7-9cb5-99838ed27880",
  "name": "my-test-instance-9",
  "imageName": "my-image-name-9",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-9.rdu",
  "serviceStatus": "up",
  "containerImage": null,
  "relations": []
}
]
```
* 404 - Instances were not found.
* 500 - An error occurred.

### POST Data/Instance
Add new instance.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "ImageName": "string",
  "ServiceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "ServiceType": "string",
  "IsReusable": true,
  "DesiredStatus": "string",
  "ServiceUrl": "string",
  "ServiceStatus": "string"
}
```

Return:

* 200 Success, newly created instance entity
```json
{
  "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
  "name": "my-test-instance-8",
  "imageName": "my-image-name-8",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-8.rdu",
  "serviceStatus": "up",
  "containerImage": null,
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Instance/{id}
Get Instance by id.

Param:

* id - provide the guid id for the desired Instance entity to be retrieved ex. 7865bd0f-62c6-421d-a82d-1d9a16223b64.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
  "name": "my-test-instance-8",
  "imageName": "my-image-name-8",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-8.rdu",
  "serviceStatus": "up",
  "containerImage": null,
  "relations": []
}
```
* 404 - Instance was not found.
* 500 - An error occurred.

### PATCH Data/Instance/{id}
Modify an existing Instance entity.

Param:

* id - provide the guid id for the desired instance entity to be modified ex. 7865bd0f-62c6-421d-a82d-1d9a16223b64.

```json
{
  "name": "my-test-instance-8",
  "imageName": "my-image-name-8",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-8.rdu",
  "serviceStatus": "down",
  "containerImage": null,
  "relations": []
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
  "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
  "name": "my-test-instance-8",
  "imageName": "my-image-name-8",
  "serviceInstanceId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "serviceType": "web-api",
  "isReusable": true,
  "desiredStatus": "up",
  "serviceUrl": "http://my-test-url-8.rdu",
  "serviceStatus": "down",
  "containerImage": null,
  "relations": []
}
```
* 404 - Instance was not found.
* 500 - An error occurred.

### DELETE Data/Instance/{id}
Delete an Instance entity.

Param:

* id - provide the guid id for the desired instance entity to be deleted ex. 7865bd0f-62c6-421d-a82d-1d9a16223b64.

Return:

* 200 - Success, instance was deleted.
* 404 - Instance was not found.
* 500 - An error occurred.

### POST Data/Instance/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "1ae73495-01f3-4273-8fd7-003c941a8491",
    "type": "Container",
    "name": "my-image-8"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
    "type": "Instance",
    "name": "my-test-instance-8"
  },
  "relationName": "NEEDS",
  "pointsTo": {
    "id": "1ae73495-01f3-4273-8fd7-003c941a8491",
    "type": "Container",
    "name": "my-image-8"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Instance/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired Instance entity relation to be retrieved ex. 7865bd0f-62c6-421d-a82d-1d9a16223b64.
* name - provide the relation name ex. NEEDS.

Return:

* 200 - Success, relation was found.
```json
{
    "initiatesFrom": {
      "id": "7865bd0f-62c6-421d-a82d-1d9a16223b64",
      "type": "INSTANCE",
      "name": "my-test-instance-8"
    },
    "relationName": "NEEDS",
    "pointsTo": {
      "id": "1ae73495-01f3-4273-8fd7-003c941a8491",
      "type": "CONTAINER",
      "name": "my-image-8"
    }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Instance/relations/{firstName}/{secondName}
Currently, Instance has single relation use case scenario.

### GET Data/Policy/{id}
Get policy by id.

Param:

* id - provide the guid id for the desired Policy entity to be retrieved ex. a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204.

Return:

* 200 Success, the specified policy entity.
```json
{
    "id": "a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204",
    "name": "QoS",
    "timestamp": "2022-03-09T10:09:43.5122616+00:00",
    "isActive": true,
    "description": "Look for Quality of Service, Quality of Experience, and Historical data available",
    "relations": []
}
```
* 404 - Policy was not found.
* 500 - An error occurred.

### PATCH Data/Policy/{id}
Modifiy an existing policy entity.

Param:

id - provide the guid id for the desired policy entity to be modified ex. a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204.

```json
{
  "name": "QoS",
  "timestamp": "2022-03-09T10:09:43.5122616+00:00",
  "isActive": true,
  "description": "Look for Quality of Service, Quality of Experience, and Historical data available",
}
```

Return:

```json
{
    "id": "a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204",
    "name": "QoS",
    "timestamp": "2022-03-09T10:09:43.5122616+00:00",
    "isActive": false,
    "description": "Look for Quality of Service, Quality of Experience, and Historical data available",
    "relations": []
}
```
* 404 - Policy was not found.
* 500 - An error occurred.

### GET Data/Policy
Get all the policies.

Return:

* 200 - Success, list of Policies.
```json
[
  {
    "id": "a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204",
    "name": "QoS",
    "timestamp": "2022-03-09T10:09:43.5122616+00:00",
    "isActive": true,
    "description": "Look for Quality of Service, Quality of Experience, and Historical data available",
    "relations": []
  },
  {
    "id": "7e982d41-1d67-4761-a902-5cafaabbc8de",
    "name": "QoS",
    "timestamp": "2022-03-11T12:09:43.5122616+00:00",
    "isActive": true,
    "description": "Try to use closest physical machine in the topolgy",
    "relations": []
  }
]
```
* 404 - Policies were not found.
* 500 - An error occurred.

### GET Data/Policy/current
Get the currently active policies.

Return:

* 200 - Success, list of active Policies.
```json
[
  {
    "id": "a0c4b9e2-00e0-4aaa-97de-c06c9f6a1204",
    "policyName": "QoS",
    "policyDescription": "Look for Quality of Service, Quality of Experience, and Historical data available"
  },
  {
    "id": "7e982d41-1d67-4761-a902-5cafaabbc8de",
    "policyName": "QoS",
    "policyDescription": "Try to use closest physical machine in the topolgy"
  }
]
```
* 404 - Active policies were not found.
* 500 - An error occurred.

### GET Data/Robot
Get all the robots.

Return:

* 200 - Success, list of Robots.
```json
[
  {
      "Id": "1f095f82-3e19-4952-b000-886a94d3b416",
      "Name": "Robot_1",
      "Manufacturer": "RobotNik",
      "RobotModel": "Summit-xl",
      "RobotStatus": "Running",
      "TaskList": [
          "Task_2"
      ],
      "BatteryStatus": 90,
      "MacAddress": "00:00:5e:00:53:af",
      "LocomotionSystem": "differential_Drive",
      "Sensors": [
          "lidar",
          "camera",
          "IMU"
      ],
      "CPU": 90,
      "RAM": 90,
      "VirtualRam": 90,
      "StorageDisk": 90,
      "NumberCores": 3,
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00",
      "Questions": [
          {
              "Id": "452d7766-aeed-488c-9fc3-06f378bbfb30",
              "Question": "Do you want slicing?",
              "IsSingleAnswer": true,
              "Answer": [
                  {
                      "key": true
                  }
              ],
              "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
          },
          {
              "Id": "46e3662a-f94c-4770-8a5a-ef3ba4e49122",
              "Question": "What types of maps do you have?",
              "IsSingleAnswer": false,
              "Answer": [
                  {
                      "2ac61d8b-896d-4428-867b-d3483bcb5c88": "2d_Geometry"
                  },
                  {
                      "4dd2464f-0215-4bd5-9f2f-07c32f1f5eaa": "2d_Semantic"
                  }
              ],
              "Timestamp": "2022-03-10T12:09:43.5122616+00:00"
          }  
      ]
  }
]
```
* 404 - Robots were not found.
* 500 - An error occurred.

### POST Data/Robot
Add new robot.

Param:

```json
{
  "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "Manufacturer": "string",
  "RobotModel": "string",
  "RobotStatus": "string",
  "CurrentTaskID": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "TaskList": [
    "string"
  ],
  "BatteryStatus": 0,
  "MacAddress": "string",
  "LocomotionSystem": "string",
  "Sensors": [
    "string"
  ],
  "CPU": 0,
  "RAM": 0,
  "VirtualRam": 0,
  "StorageDisk": 0,
  "NumberCores": 0,
  "Questions": [
    {
      "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
      "Question": "string",
      "IsSingleAnswer": true,
      "Answer": [
        {
          "key": "string",
          "value": "string"
        }
      ]
    }
  ]
}
```

Return:

* 200 Success, newly created robot entity
```json
{
      "Id": "1f095f82-3e19-4952-b000-886a94d3b416",
      "Name": "Robot_1",
      "Manufacturer": "RobotNik",
      "RobotModel": "Summit-xl",
      "RobotStatus": "Running",
      "TaskList": [
          "Task_2"
      ],
      "BatteryStatus": 90,
      "MacAddress": "00:00:5e:00:53:af",
      "LocomotionSystem": "differential_Drive",
      "Sensors": [
          "lidar",
          "camera",
          "IMU"
      ],
      "CPU": 90,
      "RAM": 90,
      "VirtualRam": 90,
      "StorageDisk": 90,
      "NumberCores": 3,
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00",
      "Questions": [
          {
              "Id": "452d7766-aeed-488c-9fc3-06f378bbfb30",
              "Question": "Do you want slicing?",
              "IsSingleAnswer": true,
              "Answer": [
                  {
                      "key": true
                  }
              ],
              "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
          },
          {
              "Id": "46e3662a-f94c-4770-8a5a-ef3ba4e49122",
              "Question": "What types of maps do you have?",
              "IsSingleAnswer": false,
              "Answer": [
                  {
                      "2ac61d8b-896d-4428-867b-d3483bcb5c88": "2d_Geometry"
                  },
                  {
                      "4dd2464f-0215-4bd5-9f2f-07c32f1f5eaa": "2d_Semantic"
                  }
              ],
              "Timestamp": "2022-03-10T12:09:43.5122616+00:00"
          }  
      ]
  }
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Robot/{id}
Get robot by id.

Param:

* id - provide the guid id for the desired robot entity to be retrieved ex. 1f095f82-3e19-4952-b000-886a94d3b416.

Return:

* 200 - Success, the specified entity.
```json
{
      "Id": "1f095f82-3e19-4952-b000-886a94d3b416",
      "Name": "Robot_1",
      "Manufacturer": "RobotNik",
      "RobotModel": "Summit-xl",
      "RobotStatus": "Running",
      "TaskList": [
          "Task_2"
      ],
      "BatteryStatus": 90,
      "MacAddress": "00:00:5e:00:53:af",
      "LocomotionSystem": "differential_Drive",
      "Sensors": [
          "lidar",
          "camera",
          "IMU"
      ],
      "CPU": 90,
      "RAM": 90,
      "VirtualRam": 90,
      "StorageDisk": 90,
      "NumberCores": 3,
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00",
      "Questions": [
          {
              "Id": "452d7766-aeed-488c-9fc3-06f378bbfb30",
              "Question": "Do you want slicing?",
              "IsSingleAnswer": true,
              "Answer": [
                  {
                      "key": true
                  }
              ],
              "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
          },
          {
              "Id": "46e3662a-f94c-4770-8a5a-ef3ba4e49122",
              "Question": "What types of maps do you have?",
              "IsSingleAnswer": false,
              "Answer": [
                  {
                      "2ac61d8b-896d-4428-867b-d3483bcb5c88": "2d_Geometry"
                  },
                  {
                      "4dd2464f-0215-4bd5-9f2f-07c32f1f5eaa": "2d_Semantic"
                  }
              ],
              "Timestamp": "2022-03-10T12:09:43.5122616+00:00"
          }  
      ]
  }
```
* 404 - Robot was not found.
* 500 - An error occurred.

### PATCH Data/Robot/{id}
Modify an existing robot entity.

Param:

* id - provide the guid id for the desired robot entity to be modified ex. 1f095f82-3e19-4952-b000-886a94d3b416.

```json
{
      "Name": "Robot_1",
      "Manufacturer": "RobotNik",
      "RobotModel": "Summit-xl",
      "RobotStatus": "Running",
      "TaskList": [
          "Task_2"
      ],
      "BatteryStatus": 90,
      "MacAddress": "00:00:5e:00:53:af",
      "LocomotionSystem": "differential_Drive",
      "Sensors": [
          "lidar",
          "camera",
          "IMU"
      ],
      "CPU": 90,
      "RAM": 90,
      "VirtualRam": 90,
      "StorageDisk": 90,
      "NumberCores": 3,
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00",
      "Questions": [
          {
              "Id": "452d7766-aeed-488c-9fc3-06f378bbfb30",
              "Question": "Do you want slicing?",
              "IsSingleAnswer": true,
              "Answer": [
                  {
                      "key": true
                  }
              ],
              "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
          },
          {
              "Id": "46e3662a-f94c-4770-8a5a-ef3ba4e49122",
              "Question": "What types of maps do you have?",
              "IsSingleAnswer": false,
              "Answer": [
                  {
                      "2ac61d8b-896d-4428-867b-d3483bcb5c88": "2d_Geometry"
                  },
                  {
                      "4dd2464f-0215-4bd5-9f2f-07c32f1f5eaa": "2d_Semantic"
                  }
              ],
              "Timestamp": "2022-03-10T12:09:43.5122616+00:00"
          }  
      ]
  }
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
      "Id": "1f095f82-3e19-4952-b000-886a94d3b416",
      "Name": "Robot_1",
      "Manufacturer": "RobotNik",
      "RobotModel": "Summit-xl",
      "RobotStatus": "Running",
      "TaskList": [
          "Task_2"
      ],
      "BatteryStatus": 90,
      "MacAddress": "00:00:5e:00:53:af",
      "LocomotionSystem": "differential_Drive",
      "Sensors": [
          "lidar",
          "camera",
          "IMU"
      ],
      "CPU": 90,
      "RAM": 90,
      "VirtualRam": 90,
      "StorageDisk": 90,
      "NumberCores": 3,
      "TimeStamped": "2022-03-11T12:09:43.5122616+00:00",
      "Questions": [
          {
              "Id": "452d7766-aeed-488c-9fc3-06f378bbfb30",
              "Question": "Do you want slicing?",
              "IsSingleAnswer": true,
              "Answer": [
                  {
                      "key": true
                  }
              ],
              "Timestamp": "2022-03-11T12:09:43.5122616+00:00"
          },
          {
              "Id": "46e3662a-f94c-4770-8a5a-ef3ba4e49122",
              "Question": "What types of maps do you have?",
              "IsSingleAnswer": false,
              "Answer": [
                  {
                      "2ac61d8b-896d-4428-867b-d3483bcb5c88": "2d_Geometry"
                  },
                  {
                      "4dd2464f-0215-4bd5-9f2f-07c32f1f5eaa": "2d_Semantic"
                  }
              ],
              "Timestamp": "2022-03-10T12:09:43.5122616+00:00"
          }  
      ]
  }
```
* 404 - Robot was not found.
* 500 - An error occurred.

### DELETE Data/Robot/{id}
Delete an Robot entity.

Param:

* id - provide the guid id for the desired robot entity to be deleted ex. 1f095f82-3e19-4952-b000-886a94d3b416.

Return:

* 200 - Success, robot was deleted.
* 404 - Robot was not found.
* 500 - An error occurred.

### POST Data/Robot/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
    "type": "Robot",
    "name": "Robot_1"
  },
  "relationName": "OWNS",
  "pointsTo": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
    "type": "Robot",
    "name": "Robot_1"
  },
  "relationName": "OWNS",
  "pointsTo": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Robot/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired robot entity relation to be retrieved ex. 73b43f02-0a95-41f8-a1b6-b4c90d5acccf.
* name - provide the relation name ex. OWNS.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
    "type": "Robot",
    "name": "Robot_1"
  },
  "relationName": "OWNS",
  "pointsTo": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Robot/relations/{firstName}/{secondName}
Get multiple relations by names.

Param:

* id - provide the guid id for the desired robot entity relation to be retrieved ex. 73b43f02-0a95-41f8-a1b6-b4c90d5acccf.
* firstName - provide the relation name ex. OWNS.
* secondName - provide the relation name ex. CAN_REACH.

Return:

* 200 - Success, relations were found.
```json
[
  {
    "initiatesFrom": {
      "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
      "type": "Robot",
      "name": "Robot_1"
    },
    "relationName": "OWNS",
    "pointsTo": {
      "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
      "type": "Task",
      "name": "Task_1"
    }
  },
  {
    "initiatesFrom": {
      "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
      "type": "Robot",
      "name": "Robot_1"
    },
    "relationName": "CAN_REACH",
    "pointsTo": {
      "id": "bfcf00e1-1044-44b5-abca-50d28bc6af44",
      "type": "Edge",
      "name": "Edge_1"
    }
  },
  {
    "initiatesFrom": {
      "id": "73b43f02-0a95-41f8-a1b6-b4c90d5acccf",
      "type": "Robot",
      "name": "Robot_1"
    },
    "relationName": "CAN_REACH",
    "pointsTo": {
      "id": "186e9c60-4682-4ec0-85b4-9284f39a381d",
      "type": "Cloud",
      "name": "Cloud_1"
    }
  }
]
```
* 404 - Relations were not found.
* 500 - An error occurred.

### GET Data/Task
Get all the tasks.

Return:

* 200 - Success, list of Tasks.
```json
[
  {
    "id": "0a37089e-8dae-4574-895b-af837dc2e96f",
    "name": "task-test",
    "taskPriority": 5,
    "actionPlanId": "138970b3-c05e-40d4-8729-7dc7fca871c7",
    "actionSequence": null,
    "relations": []
  },
  {
    "id": "c712ea4a-a83f-49b6-a6ce-36682d6418e5",
    "name": "task-test",
    "taskPriority": 13,
    "actionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
    "actionSequence": null,
    "relations": []
  },
]
```
* 404 - Tasks were not found.
* 500 - An error occurred.

### POST Data/Task
Add new task.

Param:

```json
{
  "Id": "5fa85f64-5717-4562-b3fc-2c963f66afa6",
  "Name": "string",
  "TaskPriority": 0,
  "ActionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6"
}
```

Return:

* 200 Success, newly created task entity
```json
{
  "id": "c712ea4a-a83f-49b6-a6ce-36682d6418e5",
  "name": "task-test",
  "taskPriority": 13,
  "actionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "actionSequence": null,
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Task/{id}
Get Task by id.

Param:

* id - provide the guid id for the desired Task entity to be retrieved ex. c712ea4a-a83f-49b6-a6ce-36682d6418e5.

Return:

* 200 - Success, the specified entity.
```json
{
  "id": "c712ea4a-a83f-49b6-a6ce-36682d6418e5",
  "name": "task-test",
  "taskPriority": 13,
  "actionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "actionSequence": null,
  "relations": []
}
```
* 404 - Task was not found.
* 500 - An error occurred.

### PATCH Data/Task/{id}
Modify an existing Task entity.

Param:

* id - provide the guid id for the desired task entity to be modified ex. c712ea4a-a83f-49b6-a6ce-36682d6418e5.

```json
{
  "name": "task-test",
  "taskPriority": 11,
  "actionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "actionSequence": null,
  "relations": []
}
```

Return:

* 200 - Success, the specified entity was modified.
```json
{
  "id": "c712ea4a-a83f-49b6-a6ce-36682d6418e5",
  "name": "task-test",
  "taskPriority": 11,
  "actionPlanId": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
  "actionSequence": null,
  "relations": []
}
```
* 404 - Task was not found.
* 500 - An error occurred.

### DELETE Data/Task/{id}
Delete an Task entity.

Param:

* id - provide the guid id for the desired Task entity to be deleted ex. c712ea4a-a83f-49b6-a6ce-36682d6418e5.

Return:

* 200 - Success, task was deleted.
* 404 - Task was not found.
* 500 - An error occurred.

### POST Data/Task/AddRelation
Create a relation between two entities.

Param:

```json
{
  "initiatesFrom": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  },
  "relationName": "EXTENDS",
  "pointsTo": {
    "id": "ce45d487-3cc1-4029-9166-fb81524c9d61",
    "type": "Action",
    "name": "Action_3"
  }
}
```

Return:

* 200 - Success, relation was created.
```json
{
  "initiatesFrom": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  },
  "relationName": "EXTENDS",
  "pointsTo": {
    "id": "ce45d487-3cc1-4029-9166-fb81524c9d61",
    "type": "Action",
    "name": "Action_3"
  }
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.

### GET Data/Task/relation/{name}
Get relation by name.

Param:

* id - provide the guid id for the desired Task entity relation to be retrieved ex. 4225e56e-4b68-4372-9d34-66bba1a633b3.
* name - provide the relation name ex. EXTENDS.

Return:

* 200 - Success, relation was found.
```json
{
  "initiatesFrom": {
    "id": "4225e56e-4b68-4372-9d34-66bba1a633b3",
    "type": "Task",
    "name": "Task_1"
  },
  "relationName": "EXTENDS",
  "pointsTo": {
    "id": "ce45d487-3cc1-4029-9166-fb81524c9d61",
    "type": "Action",
    "name": "Action_3"
  }
}
```
* 404 - Relation was not found.
* 500 - An error occurred.

### GET Data/Task/relations/{firstName}/{secondName}
Currently, Task has single relation use case scenario.

### POST Data/Task/ImportTask
Imports the complete task definition for the user to incorporate services to be used in the middleware.

This endpoint will add new items in redis, for the following entities : **Task**, **Action**, **Instance**, **ContainerImage**. 

Graph representation with relations is also created when accessing this endpoint.

New GUID Id's will be automatically generated for each new entiy.

The Id of the newly created **Task** will be used as input for the **ActionClient**.

Task Definition **JSON** structure breakdown : 
* Each Task contains a list of Actions named ActionSequence. 
* Each Action contains a list of Instances named Services. 
* Each Instance contains a ContainerImage.

Param:

```json
{
    "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
    "Name": "Task_1",
    "TaskPriority": 3,
    "ActionPlanId": "00000000-0000-0000-0000-000000000000",
    "ActionSequence": [
      {
        "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
        "Name": "move-base",
        "ActionFamily": "Movement",
        "Order": 1,
        "Placement": "Edge_1",
        "ActionPriority": "high",
            "Services": [
               {
                "Id": "e8f9b8a3-af9c-49dd-bf64-bd741b3e835c",
                "Name": "Instance_1",
                "ServiceInstanceId": "00000000-0000-0000-0000-000000000000",
                "ServiceType": "Web-API",
                "IsReusable": true,
                "DesiredStatus": "running",
                "ServiceUrl": "http://localhost:4091",
                "ServiceStatus": "running",
                    "ContainerImage": {
                    "Id": "431e2134-cfbf-4383-b0d8-55c8184709ff",
                    "Name": "394603622351.dkr.ecr.eu-west-1.amazonaws.com/redis-interface-api:latest",
                    "Timestamp": "2022-03-11T12:09:43.5122616+00:00",
                    "Description": "Redis API Client",
                    "K8SDeployment": "apiVersion: apps/v1\nkind: Deployment\nmetadata:\n  name: redis-interface-api\nspec:\n  selector:\n    matchLabels:\n      name: redis-interface-api\n  template:\n    metadata:\n      labels:\n        name: redis-interface-api\n    spec:\n      nodeSelector:\n        kubernetes.io/os: linux\n      containers:        \n      - name: redis-interface-api\n        image: 394603622351.dkr.ecr.eu-west-1.amazonaws.com/redis-interface-api:latest\n        imagePullPolicy: Always\n        resources: {}\n        env:\n        - name: REDIS_HOSTNAME\n          value: ec2-18-133-117-215.eu-west-2.compute.amazonaws.com # to be changed with real value\n        - name: REDIS_PORT\n          value: '6309' # to be changed with real value",
                    "K8SService": "apiVersion: v1\nkind: Service\nmetadata:\n  name: redis-interface-api\nspec:\n  type: ClusterIP\n  selector:\n    name: redis-interface-api\n  ports:\n  - port: 80\n    targetPort: 80\n    name: http\n  - port: 433\n    targetPort: 433\n    name: https"
                }
               }
            ]
       },
       {
        "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
        "Name": "detect-botle",
        "ActionFamily": "Detection",
        "Order": 2,
        "Placement": "Cloud_1",
        "ActionPriority": "high",
            "Services": [
               {
                "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
                "Name": "Instance_2",
                "ServiceInstanceId": "00000000-0000-0000-0000-000000000000",
                "ServiceType": "Web-API",
                "IsReusable": true,
                "DesiredStatus": "running",
                "ServiceUrl": "http://localhost:4143",
                "ServiceStatus": "running",
                    "ContainerImage": {
                    "Id": "3fa85f64-5717-4562-b3fc-2c963f66afa6",
                    "Name": "394603622351.dkr.ecr.eu-west-1.amazonaws.com/task-planner-api:latest",
                    "Timestamp": "2022-03-11T12:09:43.5122616+00:00",
                    "Description": "Task Planner API",
                    "K8SDeployment": "apiVersion: apps/v1\nkind: Deployment\nmetadata:\n  name: task-planner-api\nspec:\n  selector:\n    matchLabels:\n      name: task-planner-api\n  template:\n    metadata:\n      labels:\n        name: task-planner-api\n    spec:\n      nodeSelector:\n        kubernetes.io/os: linux\n      containers:        \n      - name: task-planner-api\n        image: 394603622351.dkr.ecr.eu-west-1.amazonaws.com/task-planner-api:latest\n        imagePullPolicy: Always\n        resources: {}\n        env:\n        - name: REDIS_INTERFACE_ADDRESS\n          value: http://redis-interface-api\n        - name: ORCHESTRATOR_ADDRESS\n          value: http://orchestrator-api",
                    "K8SService": "apiVersion: v1\nkind: Service\nmetadata:\n  name: redis-interface-api\nspec:\n  type: ClusterIP\n  selector:\n    name: redis-interface-api\n  ports:\n  - port: 80\n    targetPort: 80\n    name: http\n  - port: 433\n    targetPort: 433\n    name: https"
                }
               }
            ]
       }
    ]
}
```

Return:

* 200 - Success, Task was imported
```json
{
  "id": "5bbd201c-a581-4385-9581-b6e1d60a766c",
  "name": "Task_1",
  "taskPriority": 3,
  "actionPlanId": "00000000-0000-0000-0000-000000000000",
  "actionSequence": [
    {
      "id": "f25ca20f-5ffc-450e-9e39-1287b304b3fa",
      "name": "move-base",
      "actionFamily": "Movement",
      "order": 1,
      "placement": "Edge_1",
      "actionPriority": "high",
      "services": [
        {
          "id": "e1a3a51b-d3cf-4a3f-a245-a33a9444fc9d",
          "name": "Instance_1",
          "serviceInstanceId": "00000000-0000-0000-0000-000000000000",
          "serviceType": "Web-API",
          "isReusable": true,
          "desiredStatus": "running",
          "serviceUrl": "http://localhost:4091",
          "serviceStatus": "running",
          "containerImage": {
            "id": "0c57e635-4698-4cc0-ad65-331cdba1d609",
            "name": "394603622351.dkr.ecr.eu-west-1.amazonaws.com/redis-interface-api:latest",
            "timestamp": "2022-03-11T12:09:43.5122616+00:00",
            "description": "Redis API Client",
            "k8SDeployment": "apiVersion: apps/v1\nkind: Deployment\nmetadata:\n  name: redis-interface-api\nspec:\n  selector:\n    matchLabels:\n      name: redis-interface-api\n  template:\n    metadata:\n      labels:\n        name: redis-interface-api\n    spec:\n      nodeSelector:\n        kubernetes.io/os: linux\n      containers:        \n      - name: redis-interface-api\n        image: 394603622351.dkr.ecr.eu-west-1.amazonaws.com/redis-interface-api:latest\n        imagePullPolicy: Always\n        resources: {}\n        env:\n        - name: REDIS_HOSTNAME\n          value: ec2-18-133-117-215.eu-west-2.compute.amazonaws.com # to be changed with real value\n        - name: REDIS_PORT\n          value: '6309' # to be changed with real value",
            "k8SService": "apiVersion: v1\nkind: Service\nmetadata:\n  name: redis-interface-api\nspec:\n  type: ClusterIP\n  selector:\n    name: redis-interface-api\n  ports:\n  - port: 80\n    targetPort: 80\n    name: http\n  - port: 433\n    targetPort: 433\n    name: https",
            "relations": []
          },
          "relations": []
        }
      ],
      "relations": []
    },
    {
      "id": "a0a24247-1c20-440f-be2d-d7d82981dbae",
      "name": "detect-botle",
      "actionFamily": "Detection",
      "order": 2,
      "placement": "Cloud_1",
      "actionPriority": "high",
      "services": [
        {
          "id": "e431f75c-7609-4503-9450-b46647c90f52",
          "name": "Instance_2",
          "serviceInstanceId": "00000000-0000-0000-0000-000000000000",
          "serviceType": "Web-API",
          "isReusable": true,
          "desiredStatus": "running",
          "serviceUrl": "http://localhost:4143",
          "serviceStatus": "running",
          "containerImage": {
            "id": "5da96f1b-6f87-4647-a2fa-6f166776a139",
            "name": "394603622351.dkr.ecr.eu-west-1.amazonaws.com/task-planner-api:latest",
            "timestamp": "2022-03-11T12:09:43.5122616+00:00",
            "description": "Task Planner API",
            "k8SDeployment": "apiVersion: apps/v1\nkind: Deployment\nmetadata:\n  name: task-planner-api\nspec:\n  selector:\n    matchLabels:\n      name: task-planner-api\n  template:\n    metadata:\n      labels:\n        name: task-planner-api\n    spec:\n      nodeSelector:\n        kubernetes.io/os: linux\n      containers:        \n      - name: task-planner-api\n        image: 394603622351.dkr.ecr.eu-west-1.amazonaws.com/task-planner-api:latest\n        imagePullPolicy: Always\n        resources: {}\n        env:\n        - name: REDIS_INTERFACE_ADDRESS\n          value: http://redis-interface-api\n        - name: ORCHESTRATOR_ADDRESS\n          value: http://orchestrator-api",
            "k8SService": "apiVersion: v1\nkind: Service\nmetadata:\n  name: redis-interface-api\nspec:\n  type: ClusterIP\n  selector:\n    name: redis-interface-api\n  ports:\n  - port: 80\n    targetPort: 80\n    name: http\n  - port: 433\n    targetPort: 433\n    name: https",
            "relations": []
          },
          "relations": []
        }
      ],
      "relations": []
    }
  ],
  "relations": []
}
```
* 400 - Bad Request, parameters were not specified correctly.
* 500 - An error occurred.