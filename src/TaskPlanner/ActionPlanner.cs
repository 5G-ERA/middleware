using System.Linq;
using AutoMapper;
using Middleware.TaskPlanner.RedisInterface;
using ActionModel = Middleware.Common.Models.ActionModel;
using RobotModel = Middleware.Common.Models.RobotModel;
using TaskModel = Middleware.Common.Models.TaskModel;

namespace Middleware.TaskPlanner
{
    public class ActionPlanner
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

        public ActionPlanner(RedisApiClient apiClient, IMapper mapper, Guid ActionPlanningId, List<ActionModel> SequenceActions, DateTime Currenttime)
        {
            _apiClient = apiClient;
            _mapper = mapper;
            ActionPlanId = ActionPlanningId; //Automatically generated Guid.
            ActionSequence = SequenceActions; //Empty at the begining
            CurrentTime = Currenttime;
            InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
            string robotName = _robotModel.RobotName;
        }

        public async Task InferActionSequence(Guid currentTaskId)
        {
            // TasksIDs = GetAllTasksID.lua
            RedisInterface.TaskModel tmpTask = await _apiClient.TaskGetByIdAsync(currentTaskId);
            TaskModel task = _mapper.Map<TaskModel>(tmpTask);

            bool alreadyExist = task != null; //TasksIDs.Contains(currentTaskId);

            // Use . after task to access the properties of the task
            //task.TaskPriority
            if (alreadyExist == true)
            {
                //manual action Sequence with minimum config from dialogues table.

                //TaskId maps with preDefined ActionPlan --> Redis query to get PlanId by TaskId

                //Loop over each action in actionSequence.
                // If navigationFamily --> get answer do you have map, what types of maps, what sensors do you have.
                //if no map, add a new action before this one with SLAM. --> Check if timelimit exists.
                //Check the ROS version and distro for SLAM based upon dialogues table. --> run LUA script with search parameters.
                //if map, Check the ROS version and distro for SLAM based upon dialogues table

                //If manipulationFamily --> get answer, do you have neccesary articulations. --> run LUA script with search parameters.

                //If perception --> get answer, what sensors do you have sensor --> run LUA script with search parameters.

                //Return ActionSequence
            }
            else
            {

                //Activate flexible planner, infer possible action sequence.

            }


        }

        private static void ReInferActionSequence(Guid CurrentTaskId)
        {

            //Replan

        }

    }

}

