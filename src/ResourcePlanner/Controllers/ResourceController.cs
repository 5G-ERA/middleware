using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        [HttpPost]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetResource([FromBody] TaskModel inputmodel)
        {
            var resource = new TaskModel();
            return Ok(resource);
        }

        
    }
}
