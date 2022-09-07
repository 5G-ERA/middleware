using System.Linq;
using AutoMapper;
using Middleware.Common.Models;
using Middleware.TaskPlanner.ApiReference;
namespace Middleware.TaskPlanner
{
    public interface IActionPlanner
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);
       
        Task<Tuple<TaskModel, RobotModel>> InferActionSequence(Guid id, bool lockResource, List<Common.Models.DialogueModel> dialogueTemp);
    }

    public class ActionPlanner : IActionPlanner
    {

        /// <summary>
        /// Client to access Redis Interface API
        /// </summary>
        private readonly RedisInterface.RedisApiClient _apiClient;

        private readonly IMapper _mapper;

        private TaskModel _taskModel;
        private RobotModel _robotModel;
        private ActionModel _actionModel;
        private InstanceModel _instanceModel;

        public Dictionary<string, string> rosMapper = new Dictionary<string, string>();
        public List<Guid> TasksIDs { get; set; } //List with all tasks ids registered in Redis
        public Guid ActionPlanId { get; set; } //Pregenerated Id for task planner request
        public List<ActionModel> ActionSequence { get; set; }
        public DateTime CurrentTime { get; set; }
        public string LocomotionSystem { get; set; }
        public List<string> Sensors { get; set; }
        public Guid Id { get; set; }
        public int TaskPriority { get; set; }
        public string InferingProcess { get; set; }
        public Guid RobotId { get; set; }
        public string RobotName { get; set; }
        public Guid QuestionId { get; set; }
        public string QuestionName { get; set; }
        public bool IsSingleAnswer { get; set; }
        //public RobotModel robot { get; set; }
        public List<Common.Models.KeyValuePair> Answer { get; set; }

        public ActionPlanner(IApiClientBuilder apiBuilder, IMapper mapper)
        {
            _apiClient = apiBuilder.CreateRedisApiClient();
            _mapper = mapper;
            
            InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
            //string robotName = _robotModel.RobotName;
            //Guid RobotId = _robotModel.Id;
        }

        public void Initialize(List<ActionModel> actionSequence, DateTime currentTime)
        {
            ActionSequence = actionSequence; //Empty at the begining
            CurrentTime = currentTime;
        }

        public void mapRobotTopicsToNetApp(ActionModel action, RobotModel tempRobot) //TOBECOMPLETED
        { // The robot topics and the netApps topics are different. Here they get mapped automatically.
           
            List<InstanceModel> tempInstance = _mapper.Map<List<InstanceModel>>(action.Services);

            foreach (InstanceModel Instance in tempInstance)
            {
                foreach (RosTopicModel InstaceTopics in Instance.RosTopicsSub)
                {
                    foreach (SensorModel sensor in tempRobot.Sensors)
                    {
                        foreach (RosTopicModel sensorTopic in sensor.RosTopicPub)
                        {
                            if (sensorTopic.Type == InstaceTopics.Type)
                            {
                                rosMapper.Add(sensorTopic.Name, InstaceTopics.Name); //Add a mapper
                            }
                        }
                    }
                }
            }          
        }


        public async Task<Tuple<TaskModel,RobotModel>> InferActionSequence(Guid currentTaskId, bool resourceLock, List<Common.Models.DialogueModel> DialogueTemp)
        {
            //Load the robot asking from a plan from redis to middleware for infering action sequence.
            RedisInterface.RobotModel robotRedis = (await _apiClient.RobotGetByIdAsync(currentTaskId)); //TODO: change currentTaskId to robot actual Guid from Api call.
            RobotModel robot = _mapper.Map<RobotModel>(robotRedis);

            robot.Questions = DialogueTemp; //Add the questions-answers to the robot
            RedisInterface.TaskModel tmpTask = await _apiClient.TaskGetByIdAsync(currentTaskId); //Get action plan from Redis
            TaskModel task = _mapper.Map<TaskModel>(tmpTask);
            bool alreadyExist = task != null; //Check if CurrentTask is inside Redis model
            task.ActionPlanId = Guid.NewGuid();//Generate automatic new Guid for plan ID.
            task.Replan = false;

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

                    //Add action to the action sequence in TaskModel
                    ActionSequence.Add(actionItem);

                    //Map the topics of the robot to the topics of the NetApp;
                    mapRobotTopicsToNetApp(actionItem, robot);
                }
              
            //Iterate over the answers of the robot to make an action plan accordingly.
            foreach (Common.Models.DialogueModel entryDialog in robot.Questions)
            {
                QuestionId = entryDialog.Id;
                QuestionName = entryDialog.Name;
                IsSingleAnswer = entryDialog.IsSingleAnswer;
                Answer = entryDialog.Answer;

                if (QuestionName == "Whats the priority of this task?")
                {
                    Common.Models.KeyValuePair answer = Answer.First();
                    task.TaskPriority = (int)answer.Value;
                }


                if (QuestionName == "Whats your battery status?")
                {
                    Common.Models.KeyValuePair answer = Answer.First();
                    robot.BatteryStatus = (long)answer.Value;
                }
            }
            }
            task.ActionSequence = ActionSequence;
            task.ResourceLock = resourceLock;
            //robot.CurrentTaskId = task.Id; //Add the task to the robot internally in the middleware <-- not done like this.
            return new Tuple<TaskModel, RobotModel>(task,robot);
        }
        private async Task<Tuple<TaskModel, RobotModel>> ReInferActionSequence(Guid currentTaskId, Guid currentPlanId, int CompleteReplan, bool MarkovianProcess, List<Common.Models.DialogueModel> DialogueTemp, bool resourceLock)
        {
            // Prepare basic information of new plan
            TaskModel task = new TaskModel();
            task.Id = currentTaskId;
            task.ActionPlanId = Guid.NewGuid();
            task.Replan = true;

            // Load robot
            RedisInterface.RobotModel robotRedis = (await _apiClient.RobotGetByIdAsync(currentTaskId)); //TODO: change currentTaskId to robot actual Guid from Api call.
            RobotModel robot = _mapper.Map<RobotModel>(robotRedis);
            robot.Questions = DialogueTemp; //Add the questions-answers to the robot

            // Define some local method variables
            List<ActionModel> FailedActions = new List<ActionModel>();
            List<Guid> dependantActions = new List<Guid>();
            Dictionary<Guid, string> actionStatus = new Dictionary<Guid, string>();
            RelationModel dependency = new RelationModel();
            bool InstanceError = false;

            //Query Redis given curerntPLanId and obtain action seq.
            RedisInterface.ActionPlanModel riActionPLan = (await _apiClient.ActionPlanGetByIdAsync(currentPlanId));
            var actionPlan = _mapper.Map<ActionPlanModel>(riActionPLan);

            //Check if there was any issues with the deployments or instances. --> if so it is a replan on the resource planner level and action plan stays the same.
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

            // All the actions have the same status?
            var lists = actionStatus.Select(kv => kv.Value.OrderBy(x => x)).ToList();
            var first = lists.First();
            var areEqual = lists.Skip(1).All(hs => hs.SequenceEqual(first));

            // A review of the action seq is neccesary
            if (InstanceError == false)
            {
                //Prepare a complete replan asked explicitely by the robot
                if (CompleteReplan == 1)
                {
                    //TODO
                }
                //Prepare a partial replan asked explicitely by the robot
                if (CompleteReplan == 2)
                {
                    // Modify only the actions that have failed
                    if (MarkovianProcess == false)
                    {
                        foreach (ActionModel failedAction in FailedActions)
                        {
                            string family = failedAction.ActionFamily;
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

                //ActionPlanner decides if partial or complete replan is neccesary as the robot did not select either explicitely.
                if (CompleteReplan == 0)
                {
                    // The robot executed the complete action sequence and failed all of them => Task failed.               
                     if ((areEqual==true) && actionStatus.ContainsValue("Failed")){
                        //TODO
                        }

                     // The robot decided to call for replan before finishing the whole task.
                     if (actionStatus.ContainsValue("Waiting"))
                        {
                            //TODO
                        }
                }
            }

            // Provide same action seq and let resource planner modify the placement.
            else
            {
                task.ActionSequence = actionPlan.ActionSequence;
            }
            

            return new Tuple<TaskModel, RobotModel>(task, robot);
        }
    }

}



