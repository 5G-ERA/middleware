# Resource Planning and allocation
Resource Planning is based on resource allocation and service provisioning. Resource allocation plays a vital role in executing the required tasks. Regardless of allocating the resources locally or globally, there are numerous steps that go into it. 5G-ERA Middleware provides a framework that helps cloud-native resources to be allocated and deployed effortlessly. To complete a particular task, a Robot has to complete some steps, and a Resource Planner helps in executing these tasks smoothly. When the Robot asks middleware for the deployment of a certain task, middleware then asks the Resource Planner for the availability of resources. Depending on the availability of resources whether the task needs to be done locally or globally, resources are allocated respectively. The Orchestrator also plays a crucial component in the Middleware. It delivers the integration between the planning mechanism and the Kubernetes platform that allows for the automated deployment of the Network Applications. The orchestrator is responsible for the state and lifecycle management of the Network Applications and enables their management in the Kubernetes cluster. The orchestrator coordinates the deployment and manages the applications’ lifecycle under the Middleware. When the Orchestrator receives the new plan has been instantiated, it starts orchestrating all the resources needed. Furthermore, the network application developers have set priorities and parameter requirements for cloud and edge onboarding that are discussed below. The signal mapping is through color-coding (green, yellow, and red) which has set range by the network application developers as follows:

```json

"Latency":
  [
    {
      "Expected": "100",
      "Green": " = 100",
      "Yellow": "GREATER than 100 and less than 150", 
      "Red": "Greater than 150"
      "Priority": 1
    }
  ]
,
  "Throughput": 
 [
    {
      "Expected": "150",
      "Green": 150,
      "Yellow": "Greater than 140 and less than 150", 
      "Red": "LESS THAN 140"
      "Priority": 2
    }
  ]
,
"Minimum cores":
[
    {
     "Expected": "4",
     "Green": 4,
     "Yellow": "Greater than 3 and less than 4",
     "Red": "less than 3"
     "Priority": 3
  }
]
,
"Minimum ram":
[
   {
    "Expected": "16",
    "Green": 16,
    "Yellow": "Greater than 12 and less than 16",
    "Red": "less than 12"
    "Priority": 4
   }
]
,
"Disk Storage":
[ 
   {
    "Expected": "20",
    "Green": 20,
    "Yellow": "Greater than 16 and less than 20",
    "Red": "less than 16"
    "Priority": 5
   }
]
,
"Is reusable":
[
  {
   "Expected": "True",
   "Green": "True",
   "Red": "False",
   "Priority": 6
   }
]

"
```



