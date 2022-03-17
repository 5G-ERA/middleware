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


        public Guid ActionPlanId { get; set; }
        public List<ActionModel> ActionSequence { get; set; }
        public TimeOnly TimeOnly { get; set; }  
        
        public ActionPlanner (Guid ActionPlanningId, List<ActionModel> SequenceActions, TimeOnly CurrentTime)
        {
            ActionPlanId = ActionPlanningId;
            ActionSequence = SequenceActions;
            TimeOnly = CurrentTime;

        }
    }
}

//first get task and robot parameters from redis table Robot and task

// select to use manual or AI model.



// taskid -1

//
