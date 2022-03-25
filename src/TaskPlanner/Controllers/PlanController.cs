using System.Net;
using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;


namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class PlanController : ControllerBase
    {
       

        [HttpPost] //http get request
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetPlan([FromBody]TaskPlannerInputModel inputModel)
        {
            var plan =  new TaskModel();

            //ActionPlanner Plan = new ActionPlanner(Guid.NewGuid(), new List<ActionModel>() /*ActionSequence*/, DateTime.UtcNow);
            //Plan.InferActionSequence();


            return Ok(plan);
        }

      
    }
}
