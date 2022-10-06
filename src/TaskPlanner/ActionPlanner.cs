using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using AutoMapper;
using Middleware.Common.Models;
using Middleware.TaskPlanner.ApiReference;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.TaskPlanner
{
    public interface IActionPlanner
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);
       
        Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid id, bool lockResource, List<Common.Models.DialogueModel> dialogueTemp, Guid robotId);
    }

    public class ActionPlanner : IActionPlanner
    {
        private readonly RedisInterface.RedisApiClient _apiClient;
        private readonly IMapper _mapper;
        public List<ActionModel> ActionSequence { get; set; }
        public DateTime CurrentTime { get; set; }
        public string InferingProcess { get; set; }

        public List<Common.Models.KeyValuePair> Answer { get; set; }

        public ActionPlanner(IApiClientBuilder apiBuilder, IMapper mapper)
        {
            _apiClient = apiBuilder.CreateRedisApiClient();
            _mapper = mapper;
            
            InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
        }

        public void Initialize(List<ActionModel> actionSequence, DateTime currentTime)
        {
            ActionSequence = actionSequence; //Empty at the begining
            CurrentTime = currentTime;
        }

        /// <summary>
        /// Find an alternative action because replanning.
        /// </summary>
        /// <param name="action"></param>
        /// <returns>ActionModel</returns>
        /// 
        protected async Task<ActionModel> FindAlternativeAction(ActionModel action)
        {
            //List<EdgeModel> edgeData = await ExecuteLuaQueryAsync("GetResourceEdgeData", testValues)
            //ActionModel edge = (await GetAllAsync()).Where(x => x.Name == name).FirstOrDefault();
            return null;
        }

        /// <summary>
        /// Get a list of other "actions" that are markovian to the given action
        /// </summary>
        /// <param name="actionId"></param>
        /// <returns>List<RelationModel></returns>
        protected async Task<List<RelationModel>> markovianActions(Guid actionId)
        {
            List<RedisInterface.RelationModel> dependsOnRelation = (await _apiClient.ActionGetRelationByNameAsync(actionId, "DEPENDS_ON"))?.ToList();
            List<RelationModel> dependsOnAction = _mapper.Map<List<RelationModel>>(dependsOnRelation);
            return dependsOnAction;
        }

        /// <summary>
        /// Check if an action is Markovian or not
        /// </summary>
        /// <param name="actionId"></param>
        /// <returns></returns>
        protected async Task<bool> CheckIfMarkovianAction(Guid actionId)
        {
            List<RedisInterface.RelationModel> dependsOnRelation = (await _apiClient.ActionGetRelationByNameAsync(actionId, "DEPENDS_ON"))?.ToList();
            List<RelationModel> dependsOnAction = _mapper.Map<List<RelationModel>>(dependsOnRelation);
            if (dependsOnAction.Count == 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        protected List<ActionModel> SortActionSequence(List<ActionModel> unsortedActionSequence)
        {
            return null;
        }

        protected async Task<List<Guid>> getMarkovianCandidates(List<ActionModel> failedActions, List<ActionModel> completeActionSeq)
        {
            List<Guid> FinalCandidates= new List<Guid>();
            List<Guid> sharedFailedCandidates = new List<Guid>(); //markovian actions that are already in the failed list.

            //For every single action failed, there are X amount of possible markovian actions.
            foreach (ActionModel tempFailedAction in failedActions)
            {
                List<RelationModel> dependsOnAction = await markovianActions(tempFailedAction.Id);

                //Check if these markovian actions are in the failed actions list
                foreach(RelationModel relationData in dependsOnAction)
                {
                    foreach(ActionModel failedAction in failedActions)
                    {
                        if(failedAction.Id == relationData.PointsTo.Id)
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
        /// Check that the robot can run the netApp from HW, sensor & actuators perspective 
        /// </summary>
        /// <param name="robot"></param>
        /// <param name="actionItem"></param>
        /// <exception cref="InvalidOperationException"></exception>
        protected void CheckInstanceByRobotHw(RobotModel robot, ActionModel actionItem)
        {
            //Check if the instances (netApps) support the robot attributes, sensors and specifications.
            int numSensors = robot.Sensors.Count;
            int countTemp = 0;
            foreach (InstanceModel instance in actionItem.Services)
            {
                if (instance.InstanceFamily == "ComputerVision")
                {
                    foreach (SensorModel sensor in robot.Sensors)
                    {
                        if ((sensor.SensorType != "camera") | (sensor.SensorType != "depthCamera") | (sensor.SensorType != "rgdbCamera"))
                        {
                            countTemp++;
                        }
                    }
                }
                if (instance.InstanceFamily == "Manipulation")
                {
                    if (robot.Actuator.Count == 0)
                    {
                        throw new InvalidOperationException("The robot does not have any proper actuators to accomodate the neccesary one of the netApp's.");
                    }
                }
            }

            if (numSensors == countTemp)
            {
                throw new InvalidOperationException("The robot does not have any proper sensors to feed the neccesary data to the netApp.");
            }
        }

        /// <summary>
        /// Get predefined action sequence from knowledge graph given TaskId
        /// </summary>
        /// <param name="currentTaskId"></param>
        /// <param name="resourceLock"></param>
        /// <param name="DialogueTemp"></param>
        /// <param name="robotId"></param>
        /// <returns>TaskModel</returns>
        public async Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid currentTaskId, bool resourceLock, List<Common.Models.DialogueModel> DialogueTemp, Guid robotId)
        {
         
            List<ActionModel>tempTask = new List<ActionModel>();
            List<ActionModel> tempReplanedCompleteActionSeq = new List<ActionModel>();
            // modify the existing plan with the candidates
            return await InferActionSequence(currentTaskId,resourceLock, DialogueTemp, robotId, tempTask, tempReplanedCompleteActionSeq);
        }
        /// <summary>
        /// Get predefined action sequence from knowledge graph given TaskId.
        /// Or get replaned action sequence given partial or full replan.
        /// </summary>
        /// <param name="currentTaskId"></param>
        /// <param name="resourceLock"></param>
        /// <param name="DialogueTemp"></param>
        /// <param name="robotId"></param>
        /// <param name="candidates"></param>
        /// <returns>TaskModel</returns>
        public async Task<Tuple<TaskModel,RobotModel>> InferActionSequence(Guid currentTaskId, bool resourceLock, List<Common.Models.DialogueModel> DialogueTemp, Guid robotId, List<ActionModel> candidatesToRePlan, List<ActionModel> replanedCompleteActionSeq)
        {
            //Load the robot asking for a plan from redis to middleware for infering action sequence.
            RedisInterface.RobotModel robotRedis = (await _apiClient.RobotGetByIdAsync(robotId)); 
            RobotModel robot = _mapper.Map<RobotModel>(robotRedis);

            // Backup list of replanedCompleteActionSeq for removing items when processed inside this function.
            List<ActionModel> replanedCompleteActionSeqBK = new List<ActionModel>();
            replanedCompleteActionSeqBK.AddRange(replanedCompleteActionSeq);

            robot.Questions = DialogueTemp; //Add the questions-answers to the robot
            RedisInterface.TaskModel tmpTask = await _apiClient.TaskGetByIdAsync(currentTaskId); //Get action plan from Redis
            TaskModel task = _mapper.Map<TaskModel>(tmpTask);
            bool alreadyExist = task != null; //Check if CurrentTask is inside Redis model
            task.ActionPlanId = Guid.NewGuid();//Generate automatic new Guid for plan ID.

            task.PartialRePlan = false; //By default
            task.FullReplan = false; //By default

            // Set the task  replanning attribute accordingly.
            if ((replanedCompleteActionSeq.Count == 0) && (candidatesToRePlan.Count != 0)) { task.PartialRePlan = true; task.FullReplan = false; }
            if (replanedCompleteActionSeq.Count != 0) { task.PartialRePlan = false; task.FullReplan = true; }
            if ((replanedCompleteActionSeq.Count == 0) && (candidatesToRePlan.Count == 0)) { task.PartialRePlan = false; task.FullReplan = false; }

            bool ActionToConsider = false;
            bool AllActionToConsider = false;

            // Check if the redis graph knowledge base knows about this requested task.
            if (alreadyExist == true)
            {
                // For now query graph to get action sequence. It will be modified in later iterations.
                List<RedisInterface.RelationModel> tempRelations = (await _apiClient.TaskGetRelationByNameAsync(currentTaskId, "EXTENDS"))?.ToList(); //returns x and y --> taskId and ActionID

                // according to the StackOverflow this should work, if not let's map objects in the list one by one
                List<RelationModel> relations = _mapper.Map<List<RelationModel>>(tempRelations);

                //Here is the list of the Ids of actions retrieved from the relation
                List<Guid> actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

                foreach (Guid actionId in actionGuids) //Iterate over the pre-defined action sequence of the knowledge redis graph.
                {
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    //
                    //     PLAN ESTRATEGY A: use normal knowledge base relationships to get and add actions.
                    //     REPLAN ESTRATEGY B: change the failed actions to others and add the not failed actions.
                    //     REPLAN ESTRATEGY C: only add to action sequence new alternative actions and NOT succeded actions.
                    //
                    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

                    //Call to retrieve specific action
                    RedisInterface.ActionModel tempAction = await _apiClient.ActionGetByIdAsync(actionId);
                    ActionModel actionItem = _mapper.Map<ActionModel>(tempAction);

                    // Check if the robot have the neccesary sensors and actuators for the netapp to correctly function.
                    CheckInstanceByRobotHw(robot,actionItem);

                    //If the partial or complete replan was not activated, add the action to the action sequence. --> Normal plan
                    // PLAN ESTRATEGY A:
                    if ((candidatesToRePlan.Count==0) && (replanedCompleteActionSeq.Count == 0))
                    {
                        //Add action to the action sequence in TaskModel
                        ActionSequence.Add(actionItem);
                    }
                    // Some partial or complete replan was requested.
                    else
                    {
                        //Check if the action is a candidatesToRePlan and later on find a better instance (netApp).
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
                    if ((ActionToConsider == true) && (AllActionToConsider == false))
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
            return new Tuple<TaskModel, RobotModel>(task,robot);
        }
        private async Task<Tuple<TaskModel,TaskModel, RobotModel>> ReInferActionSequence(TaskModel oldTask, bool CompleteReplan, List<Common.Models.DialogueModel> DialogueTemp)
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
            RedisInterface.RobotModel robotRedis = (await _apiClient.RobotGetByIdAsync(currentTaskId)); //TODO: change currentTaskId to robot actual Guid from Api call.
            RobotModel robot = _mapper.Map<RobotModel>(robotRedis);
            robot.Questions = DialogueTemp; //Add the questions-answers to the robot

            //Query Redis given old plan and obtain action seq.
            RedisInterface.ActionPlanModel riActionPLan = (await _apiClient.ActionPlanGetByIdAsync(currentPlanId));
            var actionPlan = _mapper.Map<ActionPlanModel>(riActionPLan);

            // The robot requests to not change anything from action sequence but placement.
            if (ActionReplanLocked == false)
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
                        if (instance.ServiceStatus == "Problem")
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
                        Tuple<TaskModel, RobotModel>  replanedTask = await InferActionSequence(currentTaskId, resourceLock, DialogueTemp,robot.Id, oldTask.ActionSequence,new List<ActionModel>());
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
                                Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence, new List<ActionModel>());
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
                                    Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence, new List<ActionModel>());
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
                                    RedisInterface.ActionModel riTempActionmodel = await _apiClient.ActionGetByIdAsync(fAction.Id);
                                    ActionModel tempActionmodel = _mapper.Map<ActionModel>(riTempActionmodel);
                                    FinalCandidatesActions.Add(tempActionmodel);
                                }

                                //Ask for a new plan
                                Tuple<TaskModel, RobotModel> replanedTask = await InferActionSequence(currentTaskId, resourceLock, DialogueTemp, robot.Id, oldTask.ActionSequence, FinalCandidatesActions);
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


                return new Tuple<TaskModel, TaskModel, RobotModel>(task, oldTask,  robot);
            }
            else
            {
                // return same action sequence but with new plan id.
                task.ActionSequence = actionPlan.ActionSequence;
                return new Tuple<TaskModel, TaskModel, RobotModel>(task, oldTask, robot);
            }
        }

            
    }

}



