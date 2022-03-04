using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class TaskController : ControllerBase
    {
        [HttpGet]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetTask()
        {
            var task =  new TaskModel();  
            return Ok(task);
        }
    }
}
