using System.Net;
using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;


namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class RePlanController : ControllerBase
    {
        [HttpPost] //http get replan 
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetReplan([FromBody] TaskReplanModel inputModel)
        {
            var taskModel = new TaskModel(inputModel.Id, 1)
            {
                ActionPlanId = Guid.NewGuid(),
                ActionSequence = new List<ActionModel>() { new ActionModel( ) { } },
                TaskPriority = 1,
                Id = Guid.NewGuid(),

            };
            
            //var plan = new TaskReplanModel(inputModel.Id,inputModel.ActionPlanId);  
 //           var policyRecord = new PolicyTrimmedRecord(policy.Id, policy.PolicyName, policy.Description); 
            return Ok(taskModel);
        }
        


    }
}
/*

{
    "TaskId": "guid"
  "ActionPlanId": "guid",
  "ActionSequence": [
    {
        "ActionId": 2,
      "Status": "Done/In progress/Failed/Unable to execute",
      "Timestamp": "dd/MM/yyyy HH24:ss.mmm"
    }
  ],
  
}
*/