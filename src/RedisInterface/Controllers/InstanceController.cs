using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Attributes;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class InstanceController : MiddlewareController
{
    private readonly IInstanceRepository _instanceRepository;
    private readonly ILogger _logger;

    public InstanceController(IInstanceRepository instanceRepository, ILogger<InstanceController> logger)
    {
        _instanceRepository = instanceRepository ?? throw new ArgumentNullException(nameof(instanceRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }

    /// <summary>
    ///     Get all the InstanceModel entities
    /// </summary>
    /// <returns> the list of InstanceModel entities </returns>
    [HttpGet(Name = "InstanceGetAll")]
    [ProducesResponseType(typeof(GetInstancesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var models = await _instanceRepository.GetAllAsync();
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "instance", $"No instances found");

            var response = models.ToInstancesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get an InstanceModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the InstanceModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id:guid}", Name = "InstanceGetById")]
    [ProducesResponseType(typeof(InstanceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            var model = await _instanceRepository.GetByIdAsync(id);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Instance with id {id} not found");

            var response = model.ToInstanceResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Add a new InstanceModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created InstanceModel entity </returns>
    [HttpPost(Name = "InstanceAdd")]
    [ProducesResponseType(typeof(InstanceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] InstanceRequest request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified.");
        try
        {
            var model = request.ToInstance();
            var existingInstance = await _instanceRepository.FindSingleAsync(i => i.Name == model.Name);
            if (existingInstance is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name),
                    "Instance with specified name already exists");
            }

            var instance = await _instanceRepository.AddAsync(model);
            if (instance is null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"Could not add the instance to the data store.");
            }

            var response = instance.ToInstanceResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Partially update an existing InstanceModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified InstanceModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "InstancePatch")]
    [ProducesResponseType(typeof(InstanceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchInstanceAsync([FromMultiSource] UpdateInstanceRequest request)
    {
        try
        {
            var model = request.ToInstance();
            var exists = await _instanceRepository.GetByIdAsync(model.Id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id),
                    $"Instance with id {request.Id} not found");

            await _instanceRepository.UpdateAsync(model);
            var response = model.ToInstanceResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Delete an InstanceModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "InstanceDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            var exists = await _instanceRepository.GetByIdAsync(id);
            if (exists is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Instance with id {id} not found");

            await _instanceRepository.DeleteByIdAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "InstanceAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var isValid = await _instanceRepository.AddRelationAsync(request);
            if (!isValid)
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"The relation was not created.");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }

        return Ok(request);
    }


    /// <summary>
    ///     Deletes a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("DeleteRelation", Name = "InstanceDeleteRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var isValid = await _instanceRepository.DeleteRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"The relation was not deleted");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }

        return Ok();
    }

    /// <summary>
    ///     Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "InstanceGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), $"The relation was not specified.");
        }

        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Instance ID not specified.");
        }

        if (Enum.TryParse<RelationDirection>(direction, out var directionEnum) == false)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction),
                $"Wrong Relation direction specified");
        }

        var inputDirection = directionEnum;
        try
        {
            var relations = await _instanceRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", $"The relation was not found.");

            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    [Route("relations/{firstName}/{secondName}", Name = "InstanceGetRelationsByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
    {
        try
        {
            var relationNames = new List<string> { firstName, secondName };
            var relations = await _instanceRepository.GetRelations(id, relationNames);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relations", $"The relations were not found.");

            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return alternative instance of the same instance family of the provided Instance Id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns>InstanceModel</returns>
    [HttpGet]
    [Route("alternative/{id}", Name = "InstanceGetAlternative")]
    [ProducesResponseType(typeof(InstanceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> FindAlternativeInstance(Guid id)
    {
        if (id == Guid.Empty)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Instance ID not specified.");

        try
        {
            var alternativeInstance = await _instanceRepository.FindAlternativeInstance(id);
            if (alternativeInstance == null)
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"No alternative instance found.");

            var response = alternativeInstance.ToInstanceResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}