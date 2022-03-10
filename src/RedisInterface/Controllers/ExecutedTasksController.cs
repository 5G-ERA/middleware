using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ExecutedTasksController : ControllerBase
    {
        [HttpGet]
        [ProducesResponseType(typeof(ExecutedTasksModel), (int)HttpStatusCode.OK)]
        public async Task<List<ExecutedTasksModel>> GetAllExecutedTasksAsync()
        {
            List<ExecutedTasksModel> executedTasks = new List<ExecutedTasksModel>();
            return executedTasks;
        }
    }
}
