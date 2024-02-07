using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Attributes;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class TaskController : ControllerBase
{
    private readonly IActionRepository _actionRepository;
    private readonly IContainerImageRepository _containerImageRepository;
    private readonly IInstanceRepository _instanceRepository;
    private readonly ILogger _logger;
    private readonly ITaskRepository _taskRepository;
    private readonly ITaskService _taskService;

    public TaskController(ITaskRepository taskRepository, IActionRepository actionRepository,
        IInstanceRepository instanceRepository, IContainerImageRepository containerImageRepository,
        ITaskService taskService,
        ILogger<TaskController> logger)
    {
        _taskRepository = taskRepository ?? throw new ArgumentNullException(nameof(taskRepository));
        _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
        _instanceRepository = instanceRepository ?? throw new ArgumentNullException(nameof(instanceRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        _containerImageRepository = containerImageRepository ??
                                    throw new ArgumentNullException(nameof(containerImageRepository));
        _taskService = taskService;
    }

    /// <summary>
    ///     Get all the TaskModel entities
    /// </summary>
    /// <returns> the list of TaskModel entities </returns>
    [HttpGet(Name = "TaskGetAll")]
    [ProducesResponseType(typeof(GetTasksResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var models = await _taskRepository.GetAllAsync();
            if (models.Any() == false)
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));

            var response = models.ToTasksResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Get a TaskModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the TaskModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "TaskGetById")]
    [ProducesResponseType(typeof(TaskResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        if (id == Guid.Empty)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            var task = await _taskRepository.GetByIdAsync(id);
            if (task == null) return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));

            var response = task.ToTaskResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Add a new TaskModel entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created TaskModel entity </returns>
    [HttpPost(Name = "TaskAdd")]
    [ProducesResponseType(typeof(TaskResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] TaskRequest request)
    {
        if (request == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            var model = request.ToTask();
            var existingTask = await _taskRepository.FindSingleAsync(t=>t.Name == model.Name);
            if (existingTask is not null)
            {
                return StatusCode((int)HttpStatusCode.BadRequest,
                    new ApiResponse((int)HttpStatusCode.BadRequest,
                        "Task with specified name already exists"));
            }
            var task = await _taskRepository.AddAsync(model);
            if (task is null)
            {
                return StatusCode((int)HttpStatusCode.InternalServerError,
                    new ApiResponse((int)HttpStatusCode.InternalServerError,
                        "Could not add a task to the data store"));
            }

            var response = task.ToTaskResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    /// <summary>
    ///     Partially update an existing InstanceModel entity
    /// </summary>
    /// <returns> the modified InstanceModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "TaskPatch")]
    [ProducesResponseType(typeof(TaskResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchTaskAsync([FromMultiSource] UpdateTaskRequest request)
    {
        try
        {
            var model = request.ToTask();
            var exists = await _taskRepository.GetByIdAsync(model.Id);
            if (exists is null)
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
            await _taskRepository.UpdateAsync(model);
            var response = model.ToTaskResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    /// <summary>
    ///     Delete an TaskModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "TaskDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult> DeleteByIdAsync(Guid id)
    {
        if (id == Guid.Empty)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            var exists = await _taskRepository.GetByIdAsync(id);
            if (exists is null)
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
            await _taskRepository.DeleteByIdAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Retrieves two relations by their names
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("{id}/full", Name = "TaskGetFullDefinition")]
    [ProducesResponseType(typeof(FullTaskResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id)
    {
        try
        {
            var task = await _taskService.GetFullTaskDefinitionAsync(id);

            if (task is null)
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Task definition was not found"));
            return Ok(task.ToFullTaskResponse());
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "TaskAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
    {
        if (model == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.NotFound, "Parameters were not specified."));
        try
        {
            var isValid = await _taskRepository.AddRelationAsync(model);
            if (!isValid)
            {
                return StatusCode((int)HttpStatusCode.InternalServerError,
                    new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
            }
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }

        return Ok(model);
    }

    /// <summary>
    ///     Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "TaskGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation name not specified"));
        }
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation ID not specified"));
        }
        RelationDirection directionEnum;
        if (Enum.TryParse<RelationDirection>(direction, out directionEnum) == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Wrong Relation direction specified"));
        }
        var inputDirection = directionEnum;
        try
        {
            var relations = await _taskRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
            return Ok(relations);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Retrieves two relations by their names
    /// </summary>
    /// <param name="id"></param>
    /// <param name="firstName"></param>
    /// <param name="secondName"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relations/{firstName}/{secondName}", Name = "TaskGetRelationsByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
    {
        try
        {
            var relationNames = new List<string> { firstName, secondName };
            var relations = await _taskRepository.GetRelations(id, relationNames);
            if (!relations.Any())
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found"));
            return Ok(relations);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    /// <summary>
    ///     Imports the complete task definition for the user to incorporate services to be used in the middleware
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("ImportTask", Name = "ImportTaskAsync")]
    [ProducesResponseType(typeof(ActionResult<TaskModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<TaskModel>> ImportTaskAsync([FromBody] TaskModel model)
    {
        if (model == null)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.NotFound,
                "Parameters for TaskModel were not specified correctly."));
        }

        if (await CheckExistingTask(model.Id))
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "This task ID has already been used, please specify a new Task ID."));
        }
        var existingTask = await _taskRepository.FindSingleAsync(t=>t.Name == model.Name);
        if (existingTask is not null)
        {
            return StatusCode((int)HttpStatusCode.BadRequest,
                new ApiResponse((int)HttpStatusCode.BadRequest,
                    "Task with specified name already exists"));
        }

        if (!model.ActionSequence!.Any())
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Parameters for ActionSequence were not specified."));
        }

        try
        {
            foreach (var actionModel in model.ActionSequence)
            {
                var tmpServices = actionModel.Services;

                if (await CheckExistingAction(actionModel.Id) == false)
                    await _actionRepository.AddAsync(actionModel);
                else
                    continue;
                actionModel.Services = tmpServices;
                if (!actionModel.Services.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                        "Parameters for Services were not specified."));
                }

                foreach (var instanceModel in actionModel.Services)
                {
                    var tmpImage = instanceModel.ContainerImage;

                    if (await CheckExistingInstance(instanceModel.Id) == false)
                        await _instanceRepository.AddAsync(instanceModel);
                    else
                        continue;
                    instanceModel.ContainerImage = tmpImage;
                    if (instanceModel.ContainerImage == null)
                    {
                        return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                            "Parameters for ContainerImage were not specified."));
                    }

                    if (await CheckExistingContainerImage(instanceModel.Id) == false)
                        await _containerImageRepository.AddAsync(instanceModel.ContainerImage);
                    else
                        continue;
                    //RELATIONSHIP--NEEDS (INSTANCE-IMAGE)
                    var imageRelation = CreateGraphRelation(instanceModel, RedisDbIndexEnum.Instance,
                        instanceModel.ContainerImage, RedisDbIndexEnum.ContainerImage);
                    var isImageValid = await _containerImageRepository.AddRelationAsync(imageRelation);
                    if (!isImageValid)
                    {
                        return StatusCode((int)HttpStatusCode.InternalServerError,
                            new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                    }
                }

                //RELATIONSHIP--NEEDS (ACTION-INSTANCE)
                foreach (var instance in actionModel.Services)
                {
                    var instanceRelation = CreateGraphRelation(actionModel, RedisDbIndexEnum.Action, instance,
                        RedisDbIndexEnum.Instance);
                    var isInstanceValid = await _instanceRepository.AddRelationAsync(instanceRelation);
                    if (!isInstanceValid)
                    {
                        return StatusCode((int)HttpStatusCode.InternalServerError,
                            new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                    }
                }
            }

            var tmpSequence = model.ActionSequence;
            var importModel = await _taskRepository.AddAsync(model);
            model.ActionSequence = tmpSequence;
            foreach (var action in model.ActionSequence)
            {
                //RELATIONSHIP--EXTENDS (TASK->ACTION)
                var taskRelation =
                    CreateGraphRelation(importModel, RedisDbIndexEnum.Task, action, RedisDbIndexEnum.Action);
                var isTaskValid = await _taskRepository.AddRelationAsync(taskRelation);
                if (!isTaskValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                }
            }

            return Ok(importModel);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Creates the new Relation between 2 objects for the import of the task
    /// </summary>
    /// <param name="initiatesModel"></param>
    /// <param name="initiatesType"></param>
    /// <param name="pointsToModel"></param>
    /// <param name="pointsToType"></param>
    /// <returns></returns>
    private RelationModel CreateGraphRelation(BaseModel initiatesModel,
        RedisDbIndexEnum initiatesType,
        BaseModel pointsToModel,
        RedisDbIndexEnum pointsToType)
    {
        var initiatesFrom = new GraphEntityModel(initiatesModel.Id, initiatesModel.Name, initiatesType);
        var pointsTo = new GraphEntityModel(pointsToModel.Id, pointsToModel.Name, pointsToType);
        var relation = initiatesType == RedisDbIndexEnum.Task ? "EXTENDS" : "NEEDS";

        return new(initiatesFrom, pointsTo, relation);
    }

    private async Task<bool> CheckExistingAction(Guid guidToCheck)
    {
        var actionModel = await _actionRepository.GetByIdAsync(guidToCheck);
        if (actionModel == null)
            return false;
        return true;
    }

    private async Task<bool> CheckExistingInstance(Guid guidToCheck)
    {
        var instanceModel = await _instanceRepository.GetByIdAsync(guidToCheck);
        if (instanceModel == null)
            return false;
        return true;
    }

    private async Task<bool> CheckExistingContainerImage(Guid guidToCheck)
    {
        var containerImageModel = await _containerImageRepository.GetByIdAsync(guidToCheck);
        if (containerImageModel == null)
            return false;
        return true;
    }

    private async Task<bool> CheckExistingTask(Guid guidToCheck)
    {
        var taskModel = await _taskRepository.GetByIdAsync(guidToCheck);
        if (taskModel == null)
            return false;
        return true;
    }
}