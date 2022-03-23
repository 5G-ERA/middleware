using System.Linq;
using Middleware.Common.Models;

namespace Middleware.TaskPlanner
{
    public class ActionPlanner
    {

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
        public Guid RobotId  { get; set; }      
        public string RobotName { get; set; }

        public ActionPlanner (Guid ActionPlanningId, List<ActionModel> SequenceActions, DateTime Currenttime)
        {
            ActionPlanId = ActionPlanningId; //Automatically generated Guid.
            ActionSequence = SequenceActions; //Empty at the begining
            CurrentTime = Currenttime;
            InferingProcess = ""; //Predefined actionsequence by id or IA infering based on new task.
            string robotName = _robotModel.RobotName;
        }

        public void InferActionSequence (Guid CurrentTaskId)
            {
            // TasksIDs = GetAllTasksID.lua
            bool alreadyExist = TasksIDs.Contains(CurrentTaskId);

            if (alreadyExist==true)  
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

