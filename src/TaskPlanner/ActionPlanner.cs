using System.Linq;
using AutoMapper;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.RedisInterface;
using ActionModel = Middleware.Common.Models.ActionModel;
using RobotModel = Middleware.Common.Models.RobotModel;
using TaskModel = Middleware.Common.Models.TaskModel;

namespace Middleware.TaskPlanner
{
    public interface IActionPlanner
    {
        void Initialize(List<ActionModel> actionSequence, DateTime currentTime);
        Task<TaskModel> InferActionSequence(Guid currentTaskId, bool resourceLock);
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

        public async Task<TaskModel> InferActionSequence(Guid currentTaskId, bool resourceLock)
        {
            // TasksIDs = GetAllTasksID.lua
            RedisInterface.TaskModel tmpTask = await _apiClient.TaskGetByIdAsync(currentTaskId);
            TaskModel task = _mapper.Map<TaskModel>(tmpTask);

            bool alreadyExist = task != null; //Check if CurrentTask is inside Redis model

            task.ActionPlanId = Guid.NewGuid();
            // Use . after task to access the properties of the task
            //task.TaskPriority
            if (alreadyExist == true)
            {
                // BB: 2022-03-30
                // For now query graph to get action sequence. It will be modified in later iterations.
                List<RedisInterface.RelationModel> tempRelations = (await _apiClient.TaskGetRelationByNameAsync(currentTaskId, "EXTENDS"))?.ToList(); //returns x and y --> taskId and ActionID

                // according to the StackOverflow this should work, if not let's map objects in the list one by one
                List<RelationModel> relations = _mapper.Map<List<RelationModel>>(tempRelations);

                //here is the list of the Ids of actions retrieved from the relation
                List<Guid> actionGuids = relations.Select(r => r.PointsTo.Id).ToList();

                foreach (Guid actionId in actionGuids)
                {
                    //here call to retrieve specific action
                    //add action to the action sequence in TaskModel
                    RedisInterface.ActionModel tempAction = await _apiClient.ActionGetByIdAsync(actionId);
                    ActionModel actionItem = _mapper.Map<ActionModel>(tempAction);

                    ActionSequence.Add(actionItem);
                    //ActionModel actionItem = new ActionModel(actionId);
                    //ActionSequence.Add(actionItem.Id(actionId));
                }

            }

            task.ActionSequence = ActionSequence;
            task.resourceLock = resourceLock;
            return task;
            
            //    TaskModel tempActionSequence = _mapper.Map<TaskModel>(tempAction);
            //    object p = tempActionSequence.ActionSequence;
            //    //return p




            //    //manual action Sequence with minimum config from dialogues table.

            //    //TaskId maps with preDefined ActionPlan --> Redis query to get PlanId by TaskId

            //    List<ActionModel> ActionSequence = new List<ActionModel>(); //Simulate for now the output of lua query to get actionSequence predefined by TaskID. 
            //    for(int i=0; i < ActionSequence.Count;i++)

            //    {
            //        ActionModel action = _mapper.Map<ActionModel>(ActionSequence[i]);
            //        string family = action.ActionFamily;
            //        if (family == "Navigation")
            //        {
            //            //Execute Redis query: Give me back the result of question 73b43f02-0a95-41f8-a1b6-b4c90d5acccf registerd for robot with Guid ...
            //            //452d7946-aeed-488c-9fc3-06f378bbfb30 --> Do you have a map
            //        }
            //        if (family == "Manipulation")
            //        {
            //            //Execute Redis query: Give me back the result of param ArticulationAvailable registerd for robot with Guid ...

            //        }
            //        if (family == "Perception")
            //        {
            //            //Execute Redis query: Give me back the result of param Sensors registerd for robot with Guid ...

            //        }

            //    }

            //    //Loop over each action in actionSequence.
            //    // If navigationFamily --> get answer do you have map, what types of maps, what sensors do you have.
            //    //if no map, add a new action before this one with SLAM. --> Check if timelimit exists.
            //    //Check the ROS version and distro for SLAM based upon dialogues table. --> run LUA script with search parameters.
            //    //if map, Check the ROS version and distro for SLAM based upon dialogues table

            //        //If manipulationFamily --> get answer, do you have neccesary articulations. --> run LUA script with search parameters.

            //        //If perception --> get answer, what sensors do you have sensor --> run LUA script with search parameters.

            //        //Return ActionSequence
            //}
            //else
            //{

            //    //Activate flexible planner, infer possible action sequence.

            //}


        }

        private static void ReInferActionSequence(Guid CurrentTaskId)
        {

            //Replan

        }

    }

}

