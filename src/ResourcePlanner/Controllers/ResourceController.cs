using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        private readonly RedisInterface.RedisApiClient _redisApiClient;

        public ResourceController(RedisInterface.RedisApiClient redisApiClient)
        {
            _redisApiClient = redisApiClient;
        }

        [HttpPost(Name = "GetResourcePlan")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetResource([FromBody] TaskModel inputmodel)
        {
            var resource = new TaskModel();
            return Ok(resource);
        }

        
    }
}
