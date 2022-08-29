using System.Linq;
using AutoMapper;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.RedisInterface;
using ActionModel = Middleware.Common.Models.ActionModel;
using RobotModel = Middleware.Common.Models.RobotModel;
using TaskModel = Middleware.Common.Models.TaskModel;
using InstanceModel = Middleware.Common.Models.InstanceModel;
using RosTopicModel = Middleware.Common.Models.RosTopicModel;
using SensorModel = Middleware.Common.Models.SensorModel;

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
        private readonly RedisApiClient _apiClient;

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

        public void mapRobotTopicsToNetApp(ActionModel action, RobotModel tempRobot)
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

        //create new action mapper container to redis

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
        private static void ReInferActionSequence(Guid CurrentTaskId)
        {

            //Replan

        }

    }

}

