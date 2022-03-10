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
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<List<TaskModel>> GetAllTasksAsync(Guid id)
        {
            List<TaskModel> tasks = await _repository.GetAllTasksAsync(id);
            return tasks;    
        }
    }
}
