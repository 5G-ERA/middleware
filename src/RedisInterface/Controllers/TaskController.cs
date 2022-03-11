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
        private readonly ITaskRepository _repository;

        

        public TaskController(ITaskRepository repository)
        {
            _repository = repository ?? throw new ArgumentNullException(nameof(repository));
        }

       
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<List<TaskModel>> GetAllTasksAsync(Guid id)
        {
            List<TaskModel> tasks = await _repository.GetAllTasksAsync(id);
            return new();
        }


        [HttpPost]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> PostTaskAsync([FromBody] TaskModel taskModel)
        {
            TaskModel task = new TaskModel();
            return Ok(task);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteTaskAsync(Guid id)
        {
            //Delete Task by id
            return Ok();
        }


        
    }
}
