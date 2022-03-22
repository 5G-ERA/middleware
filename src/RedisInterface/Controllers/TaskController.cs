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


        [HttpGet(Name = "TaskGetAll")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<TaskModel>>> GetAllAsync()
        {
            List<TaskModel> models = await _taskRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "TaskGetById")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            TaskModel model = await _taskRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "TaskAdd")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> AddAsync([FromBody] TaskModel model)
        {
            await _taskRepository.AddAsync(model);
            return Ok(model);
        }


        [HttpDelete]
        [Route("{id}", Name = "TaskDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _taskRepository.DeleteByIdAsync(id);
            return Ok();
        }






    }
}
