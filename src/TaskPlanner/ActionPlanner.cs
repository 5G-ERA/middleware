using System.Linq;
using Middleware.Common.Models;

namespace Middleware.TaskPlanner
{
    public class ActionPlanner
    {
        //**Get all the data neccesary to plan**
        //============================================
        // --> get task_id
        // --> get task_priority
        // --> Get all questions and answers
        // --> get list of all task_ids registered in the system
        //--> get robot model
        //--> get LocomotionSystem
        //--> get "ArticulationAvailable": "false",
        //--> get "NumberOfArticulation": 0,
        //-->  "ArticulationDof": [],
        //-->  "Sensors": [ "lidar", "camera", "IMU" ],
        //============================================

        // taskModel
        // robot model
        private TaskModel _taskModel;
        private RobotModel _robotModel;
        private ActionModel _actionModel;

        
     


        public Guid ActionPlanId { get; set; }
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
            string rotobName = _robotModel.RobotName;
        }

        public void InferActionSequence ()
            {
            // query redis for single ID.

            // if taskId not in all TasksId registered: 
            //Activate flexible planner, infer possible action sequence.

            //else:
            //manual action Sequence with minimum config from dialogues table.
             }

        
    }

    

}

