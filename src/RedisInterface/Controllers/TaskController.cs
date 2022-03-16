using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class TaskController : ControllerBase
    {
        private readonly ITaskRepository _taskRepository;

        public TaskController(ITaskRepository repository)
        {
            _taskRepository = repository ?? throw new ArgumentNullException(nameof(repository));
        }


        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            TaskModel model = await _taskRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpGet]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<TaskModel>>> GetAllAsync()
        {
            List<TaskModel> models = await _taskRepository.GetAllAsync();

            return Ok(models);
        }






    }
}
