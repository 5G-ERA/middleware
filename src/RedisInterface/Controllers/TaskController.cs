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

        /// <summary>
        /// Get all the TaskModel entities
        /// </summary>
        /// <returns> the list of TaskModel entities </returns>
        [HttpGet(Name = "TaskGetAll")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<TaskModel>>> GetAllAsync()
        {
            List<TaskModel> models = await _taskRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get a TaskModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the TaskModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "TaskGetById")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            TaskModel model = await _taskRepository.GetByIdAsync(id);

            return Ok(model);
        }

        /// <summary>
        /// Add a new TaskModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created TaskModel entity </returns>
        [HttpPost(Name = "TaskAdd")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> AddAsync([FromBody] TaskModel model)
        {
            await _taskRepository.AddAsync(model);
            return Ok(model);
        }


        /// <summary>
        /// Partially update an existing InstanceModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "TaskPatch")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchTaskAsync([FromBody] TaskModel patch, [FromRoute] Guid id)
        {

            TaskModel model = await _taskRepository.PatchTaskAsync(id, patch);
            return Ok(model);
        }


        /// <summary>
        /// Delete an TaskModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "TaskDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _taskRepository.DeleteByIdAsync(id);
            return Ok();
        }


        [HttpGet]
        [Route("relation/{name}", Name = "GetTaskRelationByName")]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations = await _taskRepository.GetRelation(id, name);
            return Ok(relations);
        }




    }
}
