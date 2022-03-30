using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        private readonly IResourcePlanner _resourcePlanner;


        public ResourceController(IResourcePlanner resourcePlanner)
        {
            _resourcePlanner = resourcePlanner;
        }

        [HttpPost(Name = "GetResourcePlan")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetResource([FromBody] TaskModel inputmodel)
        {

            TaskModel updatedTask =  await _resourcePlanner.Plan(inputmodel);

            
            return Ok(updatedTask);
        }

        
    }
}
