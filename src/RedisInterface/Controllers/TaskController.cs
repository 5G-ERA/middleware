using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;
using System.Net;
using System.Text.Json;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class TaskController : ControllerBase
    {
        private readonly ITaskRepository _taskRepository;
        private readonly IActionRepository _actionRepository;
        private readonly IInstanceRepository _instanceRepository;
        private readonly IContainerImageRepository _containerImageRepository;
        private readonly ILogger _logger;

        public TaskController(ITaskRepository taskRepository, IActionRepository actionRepository, IInstanceRepository instanceRepository, IContainerImageRepository containerImageRepository, ILogger<TaskController> logger)
        {
            _taskRepository = taskRepository ?? throw new ArgumentNullException(nameof(taskRepository));
            _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
            _instanceRepository = instanceRepository ?? throw new ArgumentNullException(nameof(instanceRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _containerImageRepository = containerImageRepository ?? throw new ArgumentNullException(nameof(containerImageRepository));  
        }

        /// <summary>
        /// Get all the TaskModel entities
        /// </summary>
        /// <returns> the list of TaskModel entities </returns>
        [HttpGet(Name = "TaskGetAll")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<TaskModel>>> GetAllAsync()
        {
            try
            {
                List<TaskModel> models = await _taskRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound("Objects were not found.");
                }
                return Ok(models);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Get a TaskModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the TaskModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "TaskGetById")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                TaskModel model = await _taskRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound("Object was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Add a new TaskModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created TaskModel entity </returns>
        [HttpPost(Name = "TaskAdd")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<TaskModel>> AddAsync([FromBody] TaskModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _taskRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
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
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchTaskAsync([FromBody] TaskModel patch, [FromRoute] Guid id)
        {
            try
            {
                TaskModel model = await _taskRepository.PatchTaskAsync(id, patch);
                if (model == null)
                {
                    return NotFound("Object to be updated was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        /// <summary>
        /// Delete an TaskModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "TaskDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _taskRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();
        }


        [HttpPost]
        [Route("AddRelation", Name = "TaskAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                bool isValid = await _taskRepository.AddRelationAsync(model);
                if (!isValid) 
                {
                    return Problem("The relation was not created");
                }
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }

        [HttpGet]
        [Route("relation/{name}", Name = "TaskGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _taskRepository.GetRelation(id, name);
                if (!relations.Any())
                {
                    return NotFound("Relations were not found.");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "TaskGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _taskRepository.GetRelations(id, relationNames);
                if (!relations.Any())
                {
                    return NotFound("Relations were not found");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        [HttpPost]
        [Route("ImportTask", Name = "ImportTaskAsync")]
        [ProducesResponseType(typeof(ActionResult<TaskModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<TaskModel>> ImportTaskAsync([FromBody] TaskModel model) 
        {
            if (model == null)
            {
                BadRequest("Parameters for TaskModel were not specified corectly.");
            }
            if (!model.ActionSequence.Any()) { return BadRequest("Parameters for ActionSequence were not specified."); }
            foreach (ActionModel actionModel in model.ActionSequence) 
            {
                await _actionRepository.AddAsync(actionModel);

                if (!actionModel.Services.Any()) { return BadRequest("Parameters for Services were not specified."); }
                foreach (InstanceModel instanceModel in actionModel.Services) 
                { 
                    await _instanceRepository.AddAsync(instanceModel);

                    if (instanceModel.ContainerImage == null) { return BadRequest("Parameters for ContainerImage were not specified."); }
                    await _containerImageRepository.AddAsync(instanceModel.ContainerImage);

                    //RELATIONSHIP--NEEDS (INSTANCE-IMAGE)
                    GraphEntityModel initatesFrom, pointsTo;
                    string relationname = "NEEDS";
                    initatesFrom = new GraphEntityModel(instanceModel.Id, instanceModel.Name, RedisDbIndexEnum.Instance);
                    pointsTo = new GraphEntityModel(instanceModel.ContainerImage.Id, instanceModel.ContainerImage.Name, RedisDbIndexEnum.Container);
                    RelationModel imageRelation = new RelationModel(initatesFrom, pointsTo, relationname);
                    bool isImageValid = await _containerImageRepository.AddRelationAsync(imageRelation);
                    if (!isImageValid) { return Problem("The relation was not created"); }
                }
                //RELATIONSHIP--NEEDS (ACTION-INSTANCE)
                foreach (var instance in actionModel.Services)
                {
                    GraphEntityModel initiatesFrom, pointsTo;
                    string relationname = "NEEDS";
                    initiatesFrom = new GraphEntityModel(actionModel.Id, actionModel.Name, RedisDbIndexEnum.Action);
                    pointsTo = new GraphEntityModel(instance.Id, instance.Name, RedisDbIndexEnum.Instance);
                    RelationModel instanceRelation = new RelationModel(initiatesFrom, pointsTo, relationname);
                    bool isInstanceValid = await _instanceRepository.AddRelationAsync(instanceRelation);
                    if (!isInstanceValid) { return Problem("The relation was not created"); }
                }  
            }
            TaskModel importModel = await _taskRepository.AddAsync(model);
            foreach (var action  in model.ActionSequence) 
            {
                //RELATIONSHIP--EXTENDS (TASK->ACTION)
                GraphEntityModel initiatesFrom, pointsTo;
                string relationName = "EXTENDS";
                initiatesFrom = new GraphEntityModel(importModel.Id, importModel.Name, RedisDbIndexEnum.Task);
                pointsTo = new GraphEntityModel(action.Id, action.Name, RedisDbIndexEnum.Action);
                RelationModel taskRelation = new RelationModel(initiatesFrom, pointsTo, relationName);
                bool isTaskValid = await _taskRepository.AddRelationAsync(taskRelation);
                if (!isTaskValid) { return Problem("The relation was not created"); } 
            }
            return Ok(importModel);
        }



    }
}
