using Middleware.Common.MessageContracts;
using Middleware.Common.Models;
using Middleware.Common.Services;
using Middleware.TaskPlanner.Exceptions;
using Middleware.TaskPlanner.Publishers;

namespace Middleware.TaskPlanner.Services;

public class ActionPlanner : IActionPlanner
{
    private readonly IRedisInterfaceClientService _redisInterfaceClient;
    private readonly IPublisher<DeployPlanMessage> _deployPlanPublisher;

    public List<ActionModel> ActionSequence { get; set; }
    public DateTime CurrentTime { get; set; }
    public string InferingProcess { get; set; }

    public List<Common.Models.KeyValuePair> Answer { get; set; }

    public ActionPlanner(IRedisInterfaceClientService redisInterfaceClient,
        IPublisher<DeployPlanMessage> deployPlanPublisher)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _deployPlanPublisher = deployPlanPublisher;
        InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
    }

    /// <summary>
    /// BB: this function is not needed
    /// </summary>
    /// <param name="actionSequence"></param>
    /// <param name="currentTime"></param>
    public void Initialize(List<ActionModel> actionSequence, DateTime currentTime)
    {
        ActionSequence = actionSequence; //Empty at the begining
        CurrentTime = currentTime;
    }

    /// <summary>
    /// Create basic plan based on the configured action sequence
    /// </summary>
    /// <param name="taskId"></param>
    /// <param name="robotId"></param>
    /// <returns></returns>
    /// <exception cref="NotImplementedException"></exception>
    public async Task<Tuple<TaskModel, RobotModel>> Plan(Guid taskId, Guid robotId)
    {
        if (taskId == Guid.Empty)
            throw new ArgumentException(nameof(taskId));
        if (robotId == Guid.Empty)
            throw new ArgumentException(nameof(robotId));


        var robot = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        if (robot is null)
            throw new ArgumentException("The specified robot is not present in the Middleware database", nameof(robot));
        var task = await _redisInterfaceClient.TaskGetByIdAsync(taskId);
        if (task is null)
            throw new ArgumentException("The specified task is not present in the Middleware database", nameof(robot));

        List<RelationModel> relations = await _redisInterfaceClient.GetRelationAsync(task, "EXTENDS");

        List<Guid> actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

        foreach (Guid actionId in
                 actionGuids) //Iterate over the pre-defined action sequence of the knowledge redis graph.
        {
            ActionModel actionItem = await _redisInterfaceClient.ActionGetByIdAsync(actionId);
            if (actionItem is null)
                continue;
            
            task.ActionSequence.Add(actionItem);
        }
        
        return new Tuple<TaskModel, RobotModel>(task, robot);
    }

    /// <summary>
    /// Find an alternative action because replanning.
    /// </summary>
    /// <param name="action"></param>
    /// <returns>ActionModel</returns>
    /// 
    protected async Task<ActionModel> FindAlternativeAction(ActionModel action)
    {
        //A new action entity will be created with most of the attributes from the previous one but the services.
        ActionModel newAction = new ActionModel();
        newAction.Id = action.Id;
        newAction.Name = action.Name;
        newAction.Order = action.Order;
        newAction.ActionPriority = action.ActionPriority;
        newAction.Relations = action.Relations;

        foreach (InstanceModel instance in action.Services)
        {
            InstanceModel candidate = await _redisInterfaceClient.GetInstanceAlternativeAsync(instance.Id);
            newAction.Services.Add(candidate);
        }

        return null;
    }

    /// <summary>
    /// Get a list of other "actions" that are markovian to the given action
    /// </summary>
    /// <param name="actionId"></param>
    /// <returns>List<RelationModel></returns>
    protected async Task<List<RelationModel>> GetMarkovianActions(Guid actionId)
    {
        List<RelationModel> dependsOnAction =
            await _redisInterfaceClient.GetRelationForActionAsync(actionId, "DEPENDS_ON");
        return dependsOnAction;
    }

    /// <summary>
    /// Check if an action is Markovian (depends on other actions) or not.
    /// </summary>
    /// <param name="actionId"></param>
    /// <returns></returns>
    protected async Task<bool> IsMarkovianAction(Guid actionId)
    {
        var actions = await GetMarkovianActions(actionId);

        return actions.Count != 0;
    }

    /// <summary>
    /// Sort action sequence by descending order of the action attribute order.
    /// </summary>
    /// <param name="unsortedActionSequence"></param>
    /// <returns>List<ActionModel></returns>
    protected List<ActionModel> SortActionSequence(List<ActionModel> unsortedActionSequence)
    {
        List<ActionModel> SortedList = new List<ActionModel>();
        Dictionary<ActionModel, int> myDict = new Dictionary<ActionModel, int>();

        foreach (ActionModel action in unsortedActionSequence)
        {
            myDict.Add(action, action.Order);
        }

        //sort diccionary by decending order of action attribute order.
        var sortedDict = from entry in myDict orderby entry.Value descending select entry;

        foreach (KeyValuePair<ActionModel, int> entry in sortedDict)
        {
            SortedList.Add(entry.Key);
        }

        return SortedList;
    }

    /// <summary>
    /// Get the markovian (dependant) actions Guid's of a single action.
    /// </summary>
    /// <param name="failedActions"></param>
    /// <param name="completeActionSeq"></param>
    /// <returns>List<Guid></returns>
    protected async Task<List<Guid>> getMarkovianCandidates(List<ActionModel> failedActions,
        List<ActionModel> completeActionSeq)
    {
        List<Guid> FinalCandidates = new List<Guid>();
        List<Guid> sharedFailedCandidates = new List<Guid>(); //markovian actions that are already in the failed list.

        //For every single action failed, there are X amount of possible markovian actions.
        foreach (ActionModel tempFailedAction in failedActions)
        {
            List<RelationModel> dependsOnAction = await GetMarkovianActions(tempFailedAction.Id);

            //Check if these markovian actions are in the failed actions list
            foreach (RelationModel relationData in dependsOnAction)
            {
                foreach (ActionModel failedAction in failedActions)
                {
                    if (failedAction.Id == relationData.PointsTo.Id)
                    {
                        sharedFailedCandidates.Add(failedAction.Id);
                    }
                }
            }

            //Which of dependsOnAction is not in sharedFailedCandidates? --> succeded actions required
            foreach (RelationModel dependsOnActionData in dependsOnAction)
            {
                bool found = sharedFailedCandidates.Contains(dependsOnActionData.PointsTo.Id);
                //Add the succesful actions that are required by the failed actions.
                if (found == false) FinalCandidates.Add(dependsOnActionData.PointsTo.Id);
            }
        }

        return FinalCandidates;
    }

    /// <summary>
    /// Validation of robot and netApp match.
    /// </summary>
    /// <param name="robot"></param>
    /// <param name="actionItem"></param>
    /// <exception cref="IncorrectROSDistroException"></exception>
    /// <exception cref="IncorrectROSVersionException"></exception>
    protected async Task ValidateRobotVsNetApp(RobotModel robot, ActionModel actionItem)
    {
        var instances = new List<InstanceModel>();
        List<RelationModel> relations = await _redisInterfaceClient.GetRelationAsync(actionItem, "NEEDS");
        foreach (RelationModel tempRelation in relations)
        {
            var instanceTemp = await _redisInterfaceClient.InstanceGetByIdAsync(tempRelation.PointsTo.Id);
            instances.Add(instanceTemp);
        }

        // Check if the netApp ROS distro is equal to the ROS distro of robot.
        foreach (InstanceModel instance in instances)
        {
            if (instance.RosVersion != robot.RosVersion)
            {
                throw new IncorrectROSDistroException();
            }

            if (instance.ROSDistro != robot.RosDistro)
            {
                throw new IncorrectROSVersionException();
            }
        }

        //Check if the instances (netApps) support the robot attributes, sensors and specifications.
        int numSensors = robot.Sensors.Count;
        int countTemp = 0;
        foreach (InstanceModel instance in instances)
        {
            if (instance.InstanceFamily == "ComputerVision")
            {
                foreach (SensorModel sensor in robot.Sensors)
                {
                    if (sensor.Type != "camera" | sensor.Type != "depthCamera" | sensor.Type != "rgdbCamera")
                    {
                        countTemp++;
                    }
                }
            }

            if (instance.InstanceFamily == "Manipulation")
            {
                if (robot.Manipulators.Count == 0)
                {
                    throw new InvalidOperationException(
                        "The robot does not have any proper manipulators to accomodate the neccesary one of the netApp's.");
                }
            }
        }

        if (countTemp == 0)
        {
            throw new InvalidOperationException(
                "The robot does not have any proper sensors to feed the neccesary data to the netApp.");
        }
    }

    /// <summary>
    /// Get predefined action sequence from knowledge graph given TaskId or provide partial or full replan.
    /// </summary>
    /// <param name="currentTaskId"></param>
    /// <param name="resourceLock"></param>
    /// <param name="dialogueTemp"></param>
    /// <param name="robotId"></param>
    /// <returns>TaskModel</returns>
    public async Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid currentTaskId, bool ContextKnown,
        bool resourceLock, List<DialogueModel> dialogueTemp, Guid robotId)
    {
        List<ActionModel> tempTask = new List<ActionModel>();
        List<ActionModel> tempReplanedCompleteActionSeq = new List<ActionModel>();
        if (dialogueTemp is null)
            dialogueTemp = new();
        // modify the existing plan with the candidates
        return await InferActionSequence(currentTaskId, ContextKnown, resourceLock, dialogueTemp, robotId, tempTask,
            tempReplanedCompleteActionSeq);
    }

    /// <summary>
    /// Get predefined action sequence from knowledge graph given TaskId or provide partial or full replan.
    /// </summary>
    /// <param name="currentTaskId"></param>
    /// <param name="resourceLock"></param>
    /// <param name="dialogueTemp"></param>
    /// <param name="robotId"></param>
    /// <param name="candidates"></param>
    /// <returns>TaskModel</returns>
    public async Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid currentTaskId,
        bool contextKnown,
        bool resourceLock,
        List<DialogueModel> dialogueTemp,
        Guid robotId,
        List<ActionModel> candidatesToRePlan,
        List<ActionModel> replanedCompleteActionSeq)
    {
        if (dialogueTemp == null)
            throw new ArgumentException($"{nameof(dialogueTemp)} cannot be null", nameof(dialogueTemp));
        if (robotId == Guid.Empty)
            throw new ArgumentException($"{nameof(robotId)} cannot be empty", nameof(robotId));

        //Load the robot asking for a plan from redis to middleware for infering action sequence.
        RobotModel robot = await _redisInterfaceClient.RobotGetByIdAsync(robotId);

        // Backup list of replanedCompleteActionSeq for removing items when processed inside this function.
        List<ActionModel> replanedCompleteActionSeqBK = new List<ActionModel>();
        replanedCompleteActionSeqBK.AddRange(replanedCompleteActionSeq);

        robot.Questions.AddRange(dialogueTemp); //Append the questions-answers to the robot

        if (contextKnown == false)
        {
            throw new NotImplementedException();
        }

        TaskModel task = await _redisInterfaceClient.TaskGetByIdAsync(currentTaskId); //Get action plan from Redis
        bool alreadyExist = task != null; //Check if CurrentTask is inside Redis model
        task.ActionPlanId = Guid.NewGuid(); //Generate automatic new Guid for plan ID.

        task.PartialRePlan = false; //By default
        task.FullReplan = false; //By default

        // Set the task  replanning attribute accordingly.
        if (replanedCompleteActionSeq.Count == 0 && candidatesToRePlan.Count != 0)
        {
            task.PartialRePlan = true;
            task.FullReplan = false;
        }

        if (replanedCompleteActionSeq.Count != 0)
        {
            task.PartialRePlan = false;
            task.FullReplan = true;
        }

        if (replanedCompleteActionSeq.Count == 0 && candidatesToRePlan.Count == 0)
        {
            task.PartialRePlan = false;
            task.FullReplan = false;
        }

        bool ActionToConsider = false;
        bool AllActionToConsider = false;

        // Check if the redis graph knowledge base knows about this requested task.
        if (alreadyExist == true)
        {
            // For now query graph to get action sequence. It will be modified in later iterations.
            // according to the StackOverflow this should work, if not let's map objects in the list one by one
            List<RelationModel>
                relations = await _redisInterfaceClient.GetRelationAsync(task,
                    "EXTENDS"); //returns x and y --> taskId and ActionID


            List<Guid> actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

            foreach (Guid actionId in
                     actionGuids) //Iterate over the pre-defined action sequence of the knowledge redis graph.
            {
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //
                //     PLAN ESTRATEGY A: use normal knowledge base relationships to get and add actions.
                //     REPLAN ESTRATEGY B: change the failed actions to others and add the not failed actions.
                //     REPLAN ESTRATEGY C: only add to action sequence new alternative actions and NOT succeded actions.
                //
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////                    
                ActionModel actionItem = await _redisInterfaceClient.ActionGetByIdAsync(actionId);

                await ValidateRobotVsNetApp(robot, actionItem);

                //If the partial or complete replan was not activated, add the action to the action sequence. --> Normal plan
                // PLAN ESTRATEGY A:
                if (candidatesToRePlan.Count == 0 && replanedCompleteActionSeq.Count == 0)
                {
                    //Add action to the action sequence in TaskModel
                    ActionSequence.Add(actionItem);
                }
                // Some partial or complete replan was requested.
                else
                {
                    if (replanedCompleteActionSeq.Count == 0)
                    {
                        // Check if any of the candidatesToRePlan actions is the same to the old predefined action.
                        foreach (ActionModel action in candidatesToRePlan)
                        {
                            if (actionItem.Name == action.Name)
                            {
                                ActionToConsider = true;
                            }
                        }
                    }

                    //ONLY add to action seq the actions from replanedCompleteActionSeq
                    if (replanedCompleteActionSeq.Count != 0)
                    {
                        AllActionToConsider = true;
                    }
                }

                // REPLAN ESTRATEGY B:
                // Find new action of candidate action that previously failed, because of replan.
                if (ActionToConsider == true && AllActionToConsider == false)
                {
                    ActionModel newAction = await FindAlternativeAction(actionItem);
                    //Add action to the action sequence in TaskModel
                    ActionSequence.Add(newAction);
                }

                // REPLAN ESTRATEGY B:
                // Add the other actions that did not fail and where not considered for replaning
                if (AllActionToConsider == false)
                {
                    ActionSequence.Add(actionItem);
                }

                // REPLAN ESTRATEGY C:
                // Partial replan running ONLY, the failed actions but with new candidate netApp.
                if (AllActionToConsider == true)
                {
                    ActionModel tempActionPr = replanedCompleteActionSeqBK.FirstOrDefault();
                    ActionSequence.Add(tempActionPr);
                    replanedCompleteActionSeqBK.RemoveAt(0); //Remove the item for next iteration of the action loop.
                }
            }

            // Iterate over the answers of the robot to make an action plan accordingly.
            foreach (DialogueModel entryDialog in robot.Questions)
            {
                if (entryDialog.Name == "TaskPriority")
                {
                    Common.Models.KeyValuePair answer = Answer.First();
                    task.TaskPriority = (int)answer.Value;
                }
            }
        }

        task.ActionSequence = ActionSequence;
        task.ResourceLock = resourceLock;
        return new Tuple<TaskModel, RobotModel>(task, robot);
    }

    /// <summary>
    ///  Reason for the new action plan based upon information provided by robot.
    /// </summary>
    /// <param name="oldTask"></param>
    /// <param name="CompleteReplan"></param>
    /// <param name="DialogueTemp"></param>
    /// <returns>Tuple<TaskModel, TaskModel, RobotModel</returns>
    public async Task<Tuple<TaskModel, TaskModel, RobotModel>> ReInferActionSequence(TaskModel oldTask, Guid RobotId,
        bool ContextKnown, bool CompleteReplan, List<DialogueModel> DialogueTemp)
    {
        bool MarkovianProcess = oldTask.MarkovianProcess;
        Guid currentTaskId = oldTask.Id;
        Guid currentPlanId = oldTask.ActionPlanId;
        bool resourceLock = oldTask.ResourceLock;
        bool ActionReplanLocked = oldTask.ReplanActionPlannerLocked;
        List<ActionModel> oldActionSequence = oldTask.ActionSequence;
        List<ActionModel> FinalCandidatesActions = new List<ActionModel>();
        //DialogueModel dialogues = oldTask.

        // Define some local method variables

        List<ActionModel> candidatesActions = new List<ActionModel>();
        List<ActionModel> FailedActions = new List<ActionModel>();
        List<RelationModel> dependantActions = new List<RelationModel>();
        Dictionary<Guid, string> actionStatus = new Dictionary<Guid, string>();
        RelationModel dependency = new RelationModel();
        bool InstanceError = false;

        // Prepare basic information of new plan
        TaskModel task = new TaskModel();
        task.Id = currentTaskId;
        task.ActionPlanId = Guid.NewGuid();
        // Set the plan to be a replan.
        task.PartialRePlan = false;
        task.FullReplan = true;

        // Load robot

        RobotModel robot = await _redisInterfaceClient.RobotGetByIdAsync(RobotId);
        robot.Questions = DialogueTemp; //Add the questions-answers to the robot

        //Query Redis given old plan and obtain action seq.
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(currentPlanId);

        // The robot requests to not change anything from action sequence but placement.
        if (ActionReplanLocked == false) // TODO - VALIDATION WITH CompleteReplan
        {
            //Check if there were any issues with the deployments or instances. --> if so it is a replan on the resource planner level and action plan stays the same.
            foreach (ActionModel action in actionPlan.ActionSequence)
            {
                actionStatus.Add(action.Id, action.ActionStatus);
                if (action.ActionStatus == "Failed")
                {
                    FailedActions.Add(action);
                }

                foreach (InstanceModel instance in action.Services)
                {
                    if (instance.ServiceStatus == "Problem") //maybe another one more specific? Not sure about this one.
                    {
                        InstanceError = true;
                    }
                }
            }

            // How many actions in actionSeq
            int numActions = actionStatus.Count;

            // Check if all the actions have the same status?
            var lists = actionStatus.Select(kv => kv.Value.OrderBy(x => x)).ToList();
            var first = lists.First();
            var areEqual = lists.Skip(1).All(hs => hs.SequenceEqual(first));

            // A review of the action seq is neccesary --> there were no errors from the resource perspective.
            if (InstanceError == false)
            {
                //Prepare a complete replan asked explicitely by the robot
                if (CompleteReplan == true)
                {
                    Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId, ContextKnown,
                        resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence, new List<ActionModel>());
                    return new Tuple<TaskModel, TaskModel, RobotModel>(replanedTask.Item1, oldTask, robot);
                }
                //Prepare a partial replan asked explicitely by the robot
                else
                {
                    // If the task is single action then by nature it is none-Markovian.
                    if (numActions == 1)
                    {
                        foreach (ActionModel failedAction in FailedActions)
                        {
                            Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId,
                                ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                                new List<ActionModel>());
                            return new Tuple<TaskModel, TaskModel, RobotModel>(replanedTask.Item1, oldTask, robot);
                        }
                    }
                    // If task is not single action, query for markovian relationships among actions.
                    else
                    {
                        // Modify only the actions that have failed
                        if (MarkovianProcess == false)
                        {
                            foreach (ActionModel failedAction in FailedActions)
                            {
                                Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId,
                                    ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                                    new List<ActionModel>());
                                return new Tuple<TaskModel, TaskModel, RobotModel>(replanedTask.Item1, oldTask, robot);
                            }
                        }
                        // Check if the actions that failed have a depends_on relationship to other actions. Check if there is any failed Markovian action.
                        else
                        {
                            List<Guid> FinalCandidates = await getMarkovianCandidates(FailedActions, oldActionSequence);

                            // Add failed actions to the final candidates list
                            foreach (ActionModel fAction in FailedActions)
                            {
                                FinalCandidates.Add(fAction.Id);
                                FinalCandidatesActions.Add(fAction);
                            }

                            //Ask for a new plan
                            Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId,
                                ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                                FinalCandidatesActions);
                            return new Tuple<TaskModel, TaskModel, RobotModel>(replanedTask.Item1, oldTask, robot);
                        }
                    }
                }
            }

            // Provide same action seq and let resource planner modify the placement.
            else
            {
                task.ActionSequence = actionPlan.ActionSequence;
            }

            return new Tuple<TaskModel, TaskModel, RobotModel>(task, oldTask, robot);
        }
        else
        {
            // return same action sequence but with new plan id.
            task.ActionSequence = actionPlan.ActionSequence;
            return new Tuple<TaskModel, TaskModel, RobotModel>(task, oldTask, robot);
        }
    }

    public async Task PublishPlanAsync(TaskModel task, RobotModel robot)
    {
        var location = task.ActionSequence.Select(a => a.Placement).Distinct().First();
        var message = new DeployPlanMessage()
        {
            Task = task,
            RobotId = robot.Id,
            DeploymentLocation = location
        };
        await _deployPlanPublisher.PublishAsync(message);
    }
}