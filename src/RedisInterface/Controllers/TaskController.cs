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
        private readonly IPolicyRepository _policyRepository;

        public TaskController(ITaskRepository taskRepository, IPolicyRepository policyRepository)
        {
            _taskRepository = taskRepository ?? throw new ArgumentNullException(nameof(taskRepository));
            _policyRepository = policyRepository;
        }

        [HttpGet]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<List<TaskModel>> GetAllTasksAsync(Guid id)
        {
            List<PolicyModel> tasks = await _policyRepository.GetAllPoliciesAsync();
            return new();
        }
    }
}
