using System.Text.Json.Serialization;
using Middleware.Common.Models;
namespace Middleware.ResourcePlanner
{
    public class ResourcePlanner
    {
      private object closestmachine;
        private Guid ActionSequence;

        public string ActionName { get; }
        public Guid actionId { get; }
        public List<Priority> ActionPriority { get; }
        public string Placement { get; }

        public List<ActionModel> GetActionId()
        {
            return actionId;
        }

        public async Task<TaskModel> Plan(TaskModel taskModel, List<ActionModel> actionId)
        {
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
            List<Func<ActionModel>> sequence = new List<Func<ActionModel>> { () => actionId(1), () => Order(2), () => ActionPriority(3) };
            if (actionSequence.Count == 1)
                actionSequence = actionId; 


            // "ActionId": 2,
            //    "Order": 0,
            //    "ActionPriority": "1/2/3", 

            // for each action get required services (call redis API?)#


            // //get active policies: get closet machine.
            List<PolicyModel> policy = new List<PolicyModel>();
            policy.Add(new PolicyModel() { Description = (string)closestmachine } as PolicyModel);



            // // query redis for best placement.


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
            actionId = ActionPlanId;
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