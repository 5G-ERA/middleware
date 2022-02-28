using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class TaskInstanceController : ControllerBase
    {
        [HttpGet]
        [Route("[action]", Name = "GetTaskInstance")]
        [ProducesResponseType(typeof(TaskInstance), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskInstance>> GetTaskInstance()
        {
            var taskInstance =  new TaskInstance();  
            return Ok(taskInstance);
        }
    }
}
