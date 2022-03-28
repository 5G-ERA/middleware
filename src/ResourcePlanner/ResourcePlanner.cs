using System.Text.Json.Serialization;
using Middleware.Common.Models;
namespace Middleware.ResourcePlanner
{
    public class ResourcePlanner
    {
        private object closestmachine;
        private Guid ActionSequence;

        public string ActionName { get; }
        public Guid ActionId { get; }
        public List<Priority> ActionPriority { get; }
        public string Placement { get; }

        private TaskModel _taskModel;
        public Guid GetTaskId()
        {
            if (_taskModel != null)
            {
                return _taskModel.Id;
            }
            return Guid.Empty;
        }

        public List<ActionModel> GetActionId(List<ActionModel> actionId) => actionId;

        public async Task<TaskModel> Plan(TaskModel taskModel, List<ActionModel> actionId)
        {
            _taskModel = taskModel;
            // actionPlanner will give resource planner the actionSequence. 

            // Get action sequence 
            List<ActionModel> actionSequence = taskModel.ActionSequence;
            if (actionSequence == null || actionSequence.Count == 0)
                throw new ArgumentException("Action sequence cannot be empty");
            //
            // 
            //
            //
            // iterate throught actions in actionSequence
            List<ActionModel> sequence = new List<ActionModel>
            { new ActionModel() {
                ActionPriority = "3", Order = 2, Id = Guid.NewGuid() } };

            List<ActionModel> sortedSequence = sequence.OrderBy(s=>s.Order).ThenBy(s=>int.Parse(s.ActionPriority)).ToList();


            if (actionSequence.Count == 1)
                actionSequence = actionId;
            else
                actionSequence = sortedSequence;


            // "ActionId": 2,
            //    "Order": 0,
            //    "ActionPriority": "1/2/3", 
            // We get the answer here. in Action Sequence 

            // for each action get required services (call redis API?)#
            RedisInterface.ActionModel actionModel = new RedisInterface.ActionModel();
            ActionModel actionModel1 = new ActionModel();


            // //get active policies: get closet machine.
            List<PolicyModel> policy = new List<PolicyModel>();
            policy.Add(new PolicyModel() { Description = (string)closestmachine } as PolicyModel);


            //query graph for resources to perform on task 
            //slam //object detection 
            //return back to action

            List<ActionModel> policySequence = new List<ActionModel>();


            new List<ActionModel>(){ new ActionModel() { ImageName = "SLAM", Id = Guid.NewGuid() },
            new ActionModel() { ImageName = "Object detection", Id = Guid.NewGuid() } };

            //what actions needs to be instancaited to complete the action from RADU###############
            //needs the redis query for the instances for the specified actions // give action id - and return the instnaces that are connected to the graph




           
            // // query redis for best placement.
            // Get BEST Placement
            // if (EDGE_1 = closest machine and/or not occupied with any Task)

            // return "Edge_1 is free to perform task"
            //else if
            // (Cloud_1 = closest machine and/or not occupied with any Task)

            //return "Cloud_1 is free to perfrom task"
            //


            // // the actors that are free to perform task 



            // // [edge1,cloud1]
            // // call redis API

            // //  


            // set services for action from result of the redis


            return taskModel;
            

        }





        public ResourcePlanner (Guid ActionPlanId, String ActionTaskDescription, List<Priority> PrioritySequence, String BestPlacement)
        {
            ActionSequence = ActionPlanId;
            ActionName = ActionTaskDescription;
            //actionId = ActionPlanId;
            ActionPriority = PrioritySequence;
            Placement = BestPlacement;

        }

        //
    }

    public class Priority
    {
    }

    public class Order
    {
    }
}