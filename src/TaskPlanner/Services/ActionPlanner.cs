using Middleware.Common;
using Middleware.Common.MessageContracts;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.TaskPlanner.Exceptions;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;

namespace Middleware.TaskPlanner.Services;

public class ActionPlanner : IActionPlanner
{
    private readonly IPublisher<DeployPlanMessage> _deployPlanPublisher;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public List<ActionModel> ActionSequence { get; set; }
    public DateTime CurrentTime { get; set; }
    public string InferingProcess { get; set; }

    public List<KeyValuePair> Answer { get; set; }

    public ActionPlanner(IRedisInterfaceClient redisInterfaceClient,
        IPublisher<DeployPlanMessage> deployPlanPublisher)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _deployPlanPublisher = deployPlanPublisher;
        InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
    }

    /// <summary>
    ///     BB: this function is not needed
    /// </summary>
    /// <param name="actionSequence"></param>
    /// <param name="currentTime"></param>
    public void Initialize(List<ActionModel> actionSequence, DateTime currentTime)
    {
        ActionSequence = actionSequence; //Empty at the begining
        CurrentTime = currentTime;
    }

    /// <summary>
    ///     Create basic plan based on the configured action sequence
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
        var robotResponse = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        var robot = robotResponse.ToRobot();
        if (robot is null)
            throw new ArgumentException("The specified robot is not present in the Middleware database", nameof(robot));
        var taskResponse = await _redisInterfaceClient.TaskGetByIdAsync(taskId);
        var task = taskResponse.ToTask();
        if (task is null)
            throw new ArgumentException("The specified task is not present in the Middleware database", nameof(robot));

        task.ActionPlanId = Guid.NewGuid();
        task.ActionSequence = new();
        var relations = await _redisInterfaceClient.GetRelationAsync(task, "EXTENDS");

        var actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

        //Iterate over the pre-defined action sequence of the knowledge redis graph.
        foreach (var actionId in actionGuids)
        {
            var actionItem = (await _redisInterfaceClient.ActionGetByIdAsync(actionId)).ToAction();
            if (actionItem is null)
                continue;

            task.ActionSequence.Add(actionItem);
        }

        return new(task, robot);
    }

    /// <summary>
    ///     Get predefined action sequence from knowledge graph given TaskId or provide partial or full replan.
    /// </summary>
    /// <param name="currentTaskId"></param>
    /// <param name="resourceLock"></param>
    /// <param name="dialogueTemp"></param>
    /// <param name="robotId"></param>
    /// <returns>TaskModel</returns>
    public async Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid currentTaskId, bool ContextKnown,
        bool resourceLock, List<DialogueModel> dialogueTemp, Guid robotId)
    {
        var tempTask = new List<ActionModel>();
        var tempReplanedCompleteActionSeq = new List<ActionModel>();
        if (dialogueTemp is null)
            dialogueTemp = new();
        // modify the existing plan with the candidates
        return await InferActionSequence(currentTaskId, ContextKnown, resourceLock, dialogueTemp, robotId, tempTask,
            tempReplanedCompleteActionSeq);
    }

    /// <summary>
    ///     Reason for the new action plan based upon information provided by robot.
    /// </summary>
    /// <param name="oldTask"></param>
    /// <param name="CompleteReplan"></param>
    /// <param name="DialogueTemp"></param>
    /// <returns>Tuple<TaskModel, TaskModel, RobotModel</returns>
    public async Task<Tuple<TaskModel, TaskModel, RobotModel>> ReInferActionSequence(TaskModel oldTask, Guid RobotId,
        bool ContextKnown, bool CompleteReplan, List<DialogueModel> DialogueTemp)
    {
        var MarkovianProcess = oldTask.MarkovianProcess;
        var currentTaskId = oldTask.Id;
        var currentPlanId = oldTask.ActionPlanId;
        var resourceLock = oldTask.ResourceLock;
        var ActionReplanLocked = oldTask.ReplanActionPlannerLocked;
        var oldActionSequence = oldTask.ActionSequence;
        var FinalCandidatesActions = new List<ActionModel>();
        //DialogueModel dialogues = oldTask.

        // Define some local method variables

        var candidatesActions = new List<ActionModel>();
        var FailedActions = new List<ActionModel>();
        var dependantActions = new List<RelationModel>();
        var actionStatus = new Dictionary<Guid, string>();
        var dependency = new RelationModel();
        var InstanceError = false;

        // Prepare basic information of new plan
        var task = new TaskModel();
        task.Id = currentTaskId;
        task.ActionPlanId = Guid.NewGuid();
        // Set the plan to be a replan.
        task.PartialRePlan = false;
        task.FullReplan = true;

        // Load robot
        var robotResp = await _redisInterfaceClient.RobotGetByIdAsync(RobotId);
        var robot = robotResp.ToRobot();
        robot.Questions = DialogueTemp; //Add the questions-answers to the robot

        //Query Redis given old plan and obtain action seq.
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(currentPlanId);

        // The robot requests to not change anything from action sequence but placement.
        if (ActionReplanLocked == false) // TODO - VALIDATION WITH CompleteReplan
        {
            //Check if there were any issues with the deployments or instances. --> if so it is a replan on the resource planner level and action plan stays the same.
            foreach (var action in actionPlan.ActionSequence)
            {
                actionStatus.Add(action.Id, action.ActionStatus);
                if (action.ActionStatus == "Failed") FailedActions.Add(action);

                foreach (var instance in action.Services)
                {
                    if (instance.ServiceStatus == "Problem") //maybe another one more specific? Not sure about this one.
                        InstanceError = true;
                }
            }

            // How many actions in actionSeq
            var numActions = actionStatus.Count;

            // Check if all the actions have the same status?
            var lists = actionStatus.Select(kv => kv.Value.OrderBy(x => x)).ToList();
            var first = lists.First();
            var areEqual = lists.Skip(1).All(hs => hs.SequenceEqual(first));

            // A review of the action seq is neccesary --> there were no errors from the resource perspective.
            if (InstanceError == false)
            {
                //Prepare a complete replan asked explicitely by the robot
                if (CompleteReplan)
                {
                    var replanedTask = await InferActionSequence(currentTaskId, ContextKnown,
                        resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence, new());
                    return new(replanedTask.Item1, oldTask, robot);
                }
                //Prepare a partial replan asked explicitely by the robot

                // If the task is single action then by nature it is none-Markovian.
                if (numActions == 1)
                {
                    foreach (var failedAction in FailedActions)
                    {
                        var replanedTask = await InferActionSequence(currentTaskId,
                            ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                            new());
                        return new(replanedTask.Item1, oldTask, robot);
                    }
                }
                // If task is not single action, query for markovian relationships among actions.
                else
                {
                    // Modify only the actions that have failed
                    if (MarkovianProcess == false)
                    {
                        foreach (var failedAction in FailedActions)
                        {
                            var replanedTask = await InferActionSequence(currentTaskId,
                                ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                                new());
                            return new(replanedTask.Item1, oldTask, robot);
                        }
                    }
                    // Check if the actions that failed have a depends_on relationship to other actions. Check if there is any failed Markovian action.
                    else
                    {
                        var FinalCandidates = await getMarkovianCandidates(FailedActions, oldActionSequence);

                        // Add failed actions to the final candidates list
                        foreach (var fAction in FailedActions)
                        {
                            FinalCandidates.Add(fAction.Id);
                            FinalCandidatesActions.Add(fAction);
                        }

                        //Ask for a new plan
                        var replanedTask = await InferActionSequence(currentTaskId,
                            ContextKnown, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence,
                            FinalCandidatesActions);
                        return new(replanedTask.Item1, oldTask, robot);
                    }
                }
            }

            // Provide same action seq and let resource planner modify the placement.
            else
                task.ActionSequence = actionPlan.ActionSequence;

            return new(task, oldTask, robot);
        }

        // return same action sequence but with new plan id.
        task.ActionSequence = actionPlan.ActionSequence;
        return new(task, oldTask, robot);
    }

    /// <summary>
    ///     Find an alternative action because replanning.
    /// </summary>
    /// <param name="action"></param>
    /// <returns>ActionModel</returns>
    protected async Task<ActionModel> FindAlternativeAction(ActionModel action)
    {
        //A new action entity will be created with most of the attributes from the previous one but the services.
        var newAction = new ActionModel
        {
            Id = action.Id,
            Name = action.Name,
            Order = action.Order,
            ActionPriority = action.ActionPriority,
            Relations = action.Relations,
            Services = new()
        };

        foreach (var instance in action.Services)
        {
            var candidateResponse = await _redisInterfaceClient.GetInstanceAlternativeAsync(instance.Id);
            var candidate = candidateResponse.ToInstance();
            newAction.Services.Add(candidate);
        }

        return null;
    }

    /// <summary>
    ///     Get a list of other "actions" that are markovian to the given action
    /// </summary>
    /// <param name="actionId"></param>
    /// <returns>List<RelationModel></returns>
    protected async Task<List<RelationModel>> GetMarkovianActions(Guid actionId)
    {
        var dependsOnAction =
            await _redisInterfaceClient.GetRelationForActionAsync(actionId, "DEPENDS_ON");
        return dependsOnAction;
    }

    /// <summary>
    ///     Check if an action is Markovian (depends on other actions) or not.
    /// </summary>
    /// <param name="actionId"></param>
    /// <returns></returns>
    protected async Task<bool> IsMarkovianAction(Guid actionId)
    {
        var actions = await GetMarkovianActions(actionId);

        return actions.Count != 0;
    }

    /// <summary>
    ///     Sort action sequence by descending order of the action attribute order.
    /// </summary>
    /// <param name="unsortedActionSequence"></param>
    /// <returns>List<ActionModel></returns>
    protected List<ActionModel> SortActionSequence(List<ActionModel> unsortedActionSequence)
    {
        var SortedList = new List<ActionModel>();
        var myDict = new Dictionary<ActionModel, int?>();

        foreach (var action in unsortedActionSequence)
        {
            myDict.Add(action, action.Order);
        }

        //sort diccionary by decending order of action attribute order.
        var sortedDict = from entry in myDict orderby entry.Value descending select entry;

        foreach (var entry in sortedDict)
        {
            SortedList.Add(entry.Key);
        }

        return SortedList;
    }

    /// <summary>
    ///     Get the markovian (dependant) actions Guid's of a single action.
    /// </summary>
    /// <param name="failedActions"></param>
    /// <param name="completeActionSeq"></param>
    /// <returns>List<Guid></returns>
    protected async Task<List<Guid>> getMarkovianCandidates(List<ActionModel> failedActions,
        List<ActionModel> completeActionSeq)
    {
        var FinalCandidates = new List<Guid>();
        var sharedFailedCandidates = new List<Guid>(); //markovian actions that are already in the failed list.

        //For every single action failed, there are X amount of possible markovian actions.
        foreach (var tempFailedAction in failedActions)
        {
            var dependsOnAction = await GetMarkovianActions(tempFailedAction.Id);

            //Check if these markovian actions are in the failed actions list
            foreach (var relationData in dependsOnAction)
            {
                foreach (var failedAction in failedActions)
                {
                    if (failedAction.Id == relationData.PointsTo.Id) sharedFailedCandidates.Add(failedAction.Id);
                }
            }

            //Which of dependsOnAction is not in sharedFailedCandidates? --> succeded actions required
            foreach (var dependsOnActionData in dependsOnAction)
            {
                var found = sharedFailedCandidates.Contains(dependsOnActionData.PointsTo.Id);
                //Add the succesful actions that are required by the failed actions.
                if (found == false) FinalCandidates.Add(dependsOnActionData.PointsTo.Id);
            }
        }

        return FinalCandidates;
    }

    /// <summary>
    ///     Validation of robot and netApp match.
    /// </summary>
    /// <param name="robot"></param>
    /// <param name="actionItem"></param>
    /// <exception cref="IncorrectROSDistroException"></exception>
    /// <exception cref="IncorrectROSVersionException"></exception>
    protected async Task ValidateRobotVsNetApp(RobotModel robot, ActionModel actionItem)
    {
        var instances = new List<InstanceModel>();
        var relations = await _redisInterfaceClient.GetRelationAsync(actionItem, "NEEDS");
        foreach (var tempRelation in relations)
        {
            var instanceResponse = await _redisInterfaceClient.InstanceGetByIdAsync(tempRelation.PointsTo.Id);
            if (instanceResponse is null)
                continue;

            var instanceTemp = instanceResponse.ToInstance();

            instances.Add(instanceTemp);
        }

        // Check if the netApp ROS distro is equal to the ROS distro of robot.
        foreach (var instance in instances)
        {
            if (instance.RosVersion != robot.RosVersion) throw new IncorrectROSDistroException();

            if (instance.RosDistro != robot.RosDistro) throw new IncorrectROSVersionException();
        }

        //Check if the instances (netApps) support the robot attributes, sensors and specifications.
        var numSensors = robot.Sensors.Count;
        var countTemp = 0;
        foreach (var instance in instances)
        {
            if (instance.InstanceFamily == "ComputerVision")
            {
                foreach (var sensor in robot.Sensors)
                {
                    if ((sensor.Type != "camera") | (sensor.Type != "depthCamera") | (sensor.Type != "rgdbCamera"))
                        countTemp++;
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
    ///     Get predefined action sequence from knowledge graph given TaskId or provide partial or full replan.
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
        var robotResponse = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        var robot = robotResponse.ToRobot();

        // Backup list of replanedCompleteActionSeq for removing items when processed inside this function.
        var replanedCompleteActionSeqBk = new List<ActionModel>();
        replanedCompleteActionSeqBk.AddRange(replanedCompleteActionSeq);

        robot.Questions.AddRange(dialogueTemp); //Append the questions-answers to the robot

        if (contextKnown == false) throw new NotImplementedException();

        var taskResponse = await _redisInterfaceClient.TaskGetByIdAsync(currentTaskId);
        var task = taskResponse.ToTask();
        var alreadyExist = task != null; //Check if CurrentTask is inside Redis model
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

        var ActionToConsider = false;
        var AllActionToConsider = false;

        // Check if the redis graph knowledge base knows about this requested task.
        if (alreadyExist)
        {
            // For now query graph to get action sequence. It will be modified in later iterations.
            // according to the StackOverflow this should work, if not let's map objects in the list one by one
            var
                relations = await _redisInterfaceClient.GetRelationAsync(task,
                    "EXTENDS"); //returns x and y --> taskId and ActionID


            var actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

            foreach (var actionId in
                     actionGuids) //Iterate over the pre-defined action sequence of the knowledge redis graph.
            {
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //
                //     PLAN ESTRATEGY A: use normal knowledge base relationships to get and add actions.
                //     REPLAN ESTRATEGY B: change the failed actions to others and add the not failed actions.
                //     REPLAN ESTRATEGY C: only add to action sequence new alternative actions and NOT succeded actions.
                //
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                var actionResp = await _redisInterfaceClient.ActionGetByIdAsync(actionId);
                var actionItem = actionResp.ToAction();

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
                        foreach (var action in candidatesToRePlan)
                        {
                            if (actionItem.Name == action.Name) ActionToConsider = true;
                        }
                    }

                    //ONLY add to action seq the actions from replanedCompleteActionSeq
                    if (replanedCompleteActionSeq.Count != 0) AllActionToConsider = true;
                }

                // REPLAN ESTRATEGY B:
                // Find new action of candidate action that previously failed, because of replan.
                if (ActionToConsider && AllActionToConsider == false)
                {
                    var newAction = await FindAlternativeAction(actionItem);
                    //Add action to the action sequence in TaskModel
                    ActionSequence.Add(newAction);
                }

                // REPLAN ESTRATEGY B:
                // Add the other actions that did not fail and where not considered for replaning
                if (AllActionToConsider == false) ActionSequence.Add(actionItem);

                // REPLAN ESTRATEGY C:
                // Partial replan running ONLY, the failed actions but with new candidate netApp.
                if (AllActionToConsider)
                {
                    var tempActionPr = replanedCompleteActionSeqBk.FirstOrDefault();
                    ActionSequence.Add(tempActionPr);
                    replanedCompleteActionSeqBk.RemoveAt(0); //Remove the item for next iteration of the action loop.
                }
            }

            // Iterate over the answers of the robot to make an action plan accordingly.
            foreach (var entryDialog in robot.Questions)
            {
                if (entryDialog.Name == "TaskPriority")
                {
                    var answer = Answer.First();
                    task.TaskPriority = (int)answer.Value;
                }
            }
        }

        task.ActionSequence = ActionSequence;
        task.ResourceLock = resourceLock;
        return new(task, robot);
    }
}