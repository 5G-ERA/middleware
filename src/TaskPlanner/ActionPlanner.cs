using System.Linq;
using AutoMapper;
using Middleware.Common.Models;
using Middleware.TaskPlanner.ApiReference;
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

        public async Task<Tuple<TaskModel,RobotModel>> InferActionSequence(Guid currentTaskId, bool resourceLock, List<Common.Models.DialogueModel> DialogueTemp, Guid robotId)
        {
            //Load the robot asking for a plan from redis to middleware for infering action sequence.
            RedisInterface.RobotModel robotRedis = (await _apiClient.RobotGetByIdAsync(robotId)); //TODO: change currentTaskId to robot actual Guid from Api call.
            RobotModel robot = _mapper.Map<RobotModel>(robotRedis);

            robot.Questions = DialogueTemp; //Add the questions-answers to the robot
            RedisInterface.TaskModel tmpTask = await _apiClient.TaskGetByIdAsync(currentTaskId); //Get action plan from Redis
            TaskModel task = _mapper.Map<TaskModel>(tmpTask);
            bool alreadyExist = task != null; //Check if CurrentTask is inside Redis model
            task.ActionPlanId = Guid.NewGuid();//Generate automatic new Guid for plan ID.
            task.PartialRePlan = false;
            task.FullReplan = false;

            if (alreadyExist == true)
            {
                // For now query graph to get action sequence. It will be modified in later iterations.
                List<RedisInterface.RelationModel> tempRelations = (await _apiClient.TaskGetRelationByNameAsync(currentTaskId, "EXTENDS"))?.ToList(); //returns x and y --> taskId and ActionID

                // according to the StackOverflow this should work, if not let's map objects in the list one by one
                List<RelationModel> relations = _mapper.Map<List<RelationModel>>(tempRelations);

                //Here is the list of the Ids of actions retrieved from the relation
                List<Guid> actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

                foreach (Guid actionId in actionGuids)
                {
                    //Call to retrieve specific action
                    RedisInterface.ActionModel tempAction = await _apiClient.ActionGetByIdAsync(actionId);
                    ActionModel actionItem = _mapper.Map<ActionModel>(tempAction);

                    // Check if the robot have the neccesary sensors and actuators for the netapp to correctly function.
                    CheckInstanceByRobotHw(robot,actionItem);

                    //Add action to the action sequence in TaskModel
                    ActionSequence.Add(actionItem);
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
                // Define some local method variables
                List<ActionModel> FailedActions = new List<ActionModel>();
                List<Guid> dependantActions = new List<Guid>();
                Dictionary<Guid, string> actionStatus = new Dictionary<Guid, string>();
                RelationModel dependency = new RelationModel();
                bool InstanceError = false;              

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
                        //TODO
                    }
                    //Prepare a partial replan asked explicitely by the robot
                    else
                    {
                        // If the task is single action then by nature it is none-Markovian.
                        if (numActions == 1)
                        {
                            foreach (ActionModel failedAction in FailedActions)
                            {
                               // string family = failedAction.ActionFamily;
                                List<string> tags = failedAction.Tags;
                                // Query Redis for another action with the same family and tags --> Run LUA query
                                //TODO
                            }
                        }
                        // If task is not single action
                        else
                        {
                            // Modify only the actions that have failed
                            if (MarkovianProcess == false)
                            {
                                foreach (ActionModel failedAction in FailedActions)
                                {
                                    //string family = failedAction.ActionFamily;
                                    List<string> tags = failedAction.Tags;
                                    // Query Redis for another action with the same family and tags --> Run LUA query
                                    //TODO
                                }
                            }
                            // Check if the actions that failed have a depends_on relationship to other actions. Check if there is any failed Markovian action.
                            else // TODO: REDIS GRAPH create depends_on relationships.
                            {
                                foreach (ActionModel failedAction in FailedActions)
                                {
                                    List<RedisInterface.RelationModel> dependsOnRelationship = (await _apiClient.ActionGetRelationByNameAsync(failedAction.Id, "DEPENDS_ON"))?.ToList();
                                    RelationModel dependsOnAction = _mapper.Map<RelationModel>(dependsOnRelationship);
                                    dependantActions.Add(dependsOnAction.PointsTo.Id);
                                }
                                //TODO
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



