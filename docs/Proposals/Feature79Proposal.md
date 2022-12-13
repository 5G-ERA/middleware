# Technical feature proposal: 5G-ERA PROJECT

| Author | Partner | Last edit time |
|--------|---------|----------------|
|    Adrian Lendinez    |     University of Bedfordshire    |       06-12-2022         |


- **Addressed a github issue?** Yes
- (*If not, include the issue definition here:*) ...

- **Github issue Id:** #79

- **System/s affected:** Middleware and reference netApp.
- **Partners involved:** BED, BUT

___

- **Technical proposal specifications:** 

## *Background:*

5G-ERA Middleware can essentially support 2 types of deployments.
1) **Context free low level** single/multiple action sequence netApps.
2) **Semantic-complex high level** task action sequence netApps.

This two paradigms are extremelly different and represent two use-cases.
1) Deployment type **context-free** askes for tasks that either prepare single or multiple netApps to use as specific algorithms for specific problems to be solved. I.E *Face recognition* problem in IA can be solved by running a YOLO algorithm. *Object tracking* can be addressed by using an algorithm specific to that. All of this represent low level actions required by the robot to accomplish a high level task that the middleware does not know and **DOESNT** need to know. In this sense, this type of deployment is like an netApp eShop. User's ask for a single/multiple netApp encapsulated in a task and middleware deployes it without any complex semantic planning included. *The author of this proposal believes this the approach to be mostly used by the end-user/developers as normally, high level task design is accomodated in the robot and not shared with the middleware or cloud infraestructure in the classic approaches.* Also, ontologies for robot tasks are not common.

2) **Semantic high level task** deployments require the robot to inform the middleware of the high level tasks that the robot needs to accomplish. In this case, both systems need to understand **the same** and coordinate. Assuming this to be the case, action planner component inside the middleware prepares an action sequence with multiple or single netApp's inside the ordered steps (known as actions). The robot executes the task by order and the middleware in terms of resource planning, may decide to deploy only action 1 netApps instead of the whole plan actions. Obviously, this case is much more complex that the first one presented and innovate at the same time. There is a need to define a **Robot task ontology** for the robot and middleware to understand the task ahead. This, by all means is very big topic.

## **Extra considerations:**
1)  Each task that the robot wants to execute either as a low level or high level task, should be think of as a **ticket**. This means that the usual attributes for a ticket should be avaialble. I.E Id, description, status, historical change, and time tracking. I suggest thinking of it as a ticketing system that a company may use for their employee to ask for things or inform about incidents. Example of such a ticketing system include Remedy BMC and Service now or even Redmine. In the IT world these are known as ITSM systems. The middleware should handle this information accordingly. Such a data include relationships between users and incidents types, groups of users etc. This means that data storage should accomodate SQL like tech (git issue #71).

2) **Custom tasks:**
Users can create their custom action sequence meaning that they can select which netApps should run at each step (action). Consequently, custom tasks will come with custom taks names (even scope control) and randomly generated Ids.

## *Technical proposal:*

## A) Context free
Robot asking for a deployment of type **context free** have two scenarios:

1) Robot wants specific netApp/s.
2) Robot wants a netApp/s of specific family.

The new interface **planning endpoint** should have: *taskId*, *taskName*, *taskDescription* and *priority* as main parameters. The robot/developer should be given the option to ask for (scenario 1.) a task by name or by Guid. For scenario 2. by netApp family or name or Guid.

Description field will be accomodated to include custom 5g-era commands. I.E  

```
TaskId: None
TaskName: None
Description: \run \netApp --name "ComputerVision_Yolo" --version 1.0
Priority: Low
```

(*Naming conventions for netApps will be addresed later, same for task naming conventions and ontology of robot tasks.*)

In this case, as it can be seen, a netApp is been asked for and not a plan. What will happend behind the scene is a task with a single action in action sequence (the actual netApp) will be run. When onboarding a netApp, middleware will automatically create a taskID with a single action that been the netApp deployment. Additiaonylly, the command includes version control for the netApp. Additional ways to run this very same experiment:

```
TaskId: None
TaskName: None
Description: \run \task --id "cfcb6637-f064-4e74-8131-f764af624f44" 
Priority: Low
```

This is reminiscene of the way middleware version 1 works. Even if it is not human readble, the author of this document recommends keep in it as an option. 

More ways to run this netApp:


```
TaskId: None
TaskName: None
Description: \run \netApp --netAppFamily "ComputerVision"
Priority: Low
```

This will run a netApp of the corresponding family and with the robot avaialble specs. 

```
TaskId: cfcb6637-f064-4e74-8131-f764af624f44
TaskName: None
Description: None
Priority: Low
```

Other ways:

```
TaskId: cfcb6637-f064-4e74-8131-f764af624f44
TaskName: None
Description: None
Priority: Low
```

```
TaskId: None
TaskName: "ComputerVision_Yolo"
Description: None
Priority: Low
```

In a very advanced version of the middleware, NLP should be supported. So full description should be able to be interpreted by the middleware and create the task. I.E

```
TaskId: None
TaskName: None
Description: Deploy an algorithm to do computer vision to detect my pet.
Priority: Low
```

### Naming conventions:

#### NetApp: 
It should include the netApp family name it is part of and a descriptive name of the functioanlity or name of the algorithm.

```
netAppFamily_descriptiveName
```

**Important**: for custom tasks the user still has to choose the netApp family the algorithm is part of as a compulsory field. Therefore, netApp family names must be public and easily accesible. Recommended as a scroll list in the dashboard and also wiki page.

If the user creates a custom task, it can use TaskId or TaskName fields and even Description to let the middleware know about the deployment.

#### Tasks: 
An already stablished ontology for robot tasks or a new ontology needs to be proposed. This ontology should be public and easy to find. It must accomodate all partners neccesities and usecases.
TODO: Check if avaible ontology available. Git issue: #82

## B) High level complex semantic task.

This requires issue #82 (robot task ontology) to be completed. It means that task names and task id's represent an already known complex high level task agreed between robot and middleware. This for a simplest scenario would just require passing id or name of task to the middleware. 

Complex case scenario: doing a task may require some standard procedure (action sequence),  but it can change depending on the context the robot is in. If a room has a very dark lighting condition, maybe there is a better algorithm to run in the dark. Such a context specific data is required for the middleware to perform analogy between past robot experiences and newly to come robots asking for a task or running a current task. 

So far, this context specific details can be accomodated using the task tags in the API definition and task model in the middleware code. However, then again, tags can mean many things depending on the context. For this, I suggest using normal tags in any language for flexibility but trying the robots/users to use the following standard: [wikiData](https://www.wikidata.org/wiki/Wikidata:Main_Page) #Q values. This is an open source project that uniquely tags each thing. Example, R2D2 is is #Q value: Q51788, instance of "astromech droid", creator "George Lucas" etc. WikiData supports query language under SPARQL. I recommend using a tuple of (#Q value, human readble name) I.E (Q51788, R2D2)

 
