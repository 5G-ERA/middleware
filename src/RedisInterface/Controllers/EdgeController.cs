using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Attributes;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class EdgeController : MiddlewareController
{
    private readonly ILocationRepository _locationRepository;
    private readonly ILogger _logger;

    public EdgeController(ILocationRepository locationRepository, ILogger<EdgeController> logger)
    {
        _locationRepository = locationRepository ?? throw new ArgumentNullException(nameof(locationRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }

    /// <summary>
    ///     Get all the Edge entities
    /// </summary>
    /// <returns> the list of Edge entities </returns>
    [HttpGet(Name = "EdgeGetAll")]
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var edgeType = LocationType.Edge.ToString();
            var models = await _locationRepository.FindAsync(e => e.Type == edgeType);
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "edge", "No Edges were found");

            var response = models.ToEdgesResponse();
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
    ///     Get an Edge entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the Edge entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "EdgeGetById")]
    [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            var idStr = id.ToString();
            var type = LocationType.Edge.ToString();
            var model = await _locationRepository.FindSingleAsync(e =>
                e.Id == idStr && e.Type == type);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Edge with specified id: {id} was not found");

            var response = model.ToEdgeResponse();
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
    ///     Add a new Edge entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created EdgeModel entity </returns>
    [HttpPost(Name = "EdgeAdd")]
    [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] EdgeRequest request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request), "Request body was not specified");

        try
        {
            var model = request.ToLocation();
            var existingLoc = await _locationRepository.GetByNameAsync(model.Name);
            if (existingLoc is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name), "Location with specified name already exists");
            }
            var edge = await _locationRepository.AddAsync(model);
            if (edge is null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Problem while adding the Edge to the data store");
            }

            var response = edge.ToEdgeResponse();
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
    ///     Partially update an existing Edge entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified Edge entity </returns>
    [HttpPut]
    [Route("{id}", Name = "EdgePatch")]
    [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchEdgeAsync([FromMultiSource] UpdateEdgeRequest request)
    {
        if (request is null)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");
        }

        try
        {
            var edge = request.ToLocation();
            var exists = await _locationRepository.GetByIdAsync(edge.Id);
            if (exists is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id), "Object to be updated was not found.");
            }

            await _locationRepository.UpdateAsync(edge);
            var response = edge.ToEdgeResponse();
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
    ///     Delete an Edge entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "EdgeDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "The id cannot be empty");
            }

            var exists = await _locationRepository.GetByIdAsync(id);
            if (exists is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "The specified Edge has not been found.");
            }

            await _locationRepository.DeleteByIdAsync(id);
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
    [Route("AddRelation", Name = "EdgeAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");

        try
        {
            var isValid = await _locationRepository.AddRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "The relation was not created");
            }
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
    ///     Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "EdgeGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), "Relation name not specified");
        
        if (id == Guid.Empty)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Edge Id was not specified");
        
        if (Enum.TryParse(direction, out RelationDirection directionEnum) == false)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction), "Wrong Relation direction specified");
        var inputDirection = directionEnum;
        try
        {
            var relations = await _locationRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", "Relation was not found");

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
    [Route("relations/{firstName}/{secondName}", Name = "EdgeGetRelationsByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
    {
        try
        {
            var relationNames = new List<string> { firstName, secondName };
            var relations = await _locationRepository.GetRelations(id, relationNames);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", "Relations were not found");

            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("free", Name = "GetFreeEdgesIds")] //edges
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetFreeEdgesIdsAsync(List<EdgeModel> request)
    {
        try
        {
            if (!request.Any())
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");

            var edgeFree = await _locationRepository.FilterFreeLocationsAsync(request.ToLocations());
            if (!edgeFree.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request), "There are no edges connected to the Robot");
            }

            var response = edgeFree.ToEdgesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("lessBusy", Name = "GetLessBusyEdges")]
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLessBusyEdgesAsync(List<EdgeModel> request)
    {
        try
        {
            if (!request.Any())
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");

            var lessBusyEdge = await _locationRepository.OrderLocationsByUtilizationAsync(request.ToLocations());
            if (!lessBusyEdge.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "edges", "There are no busy edges");

            var response = lessBusyEdge.ToEdgesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("name/{name}", Name = "EdgeGetDataByName")]
    [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetEdgeResourceDetailsByNameAsync(string name)
    {
        try
        {
            var edgeResource = await _locationRepository.GetByNameAsync(name);
            if (edgeResource is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(name), "Edge with specified name was not found.");

            var response = edgeResource.ToEdgeResponse();
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
    ///     Check if a edge is busy by Id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{id}/busy", Name = "IsBusyEdgeById")]
    [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyEdgeById(Guid id)
    {
        try
        {
            var busy = await _locationRepository.IsBusyAsync(id);
            return Ok(busy);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Check if a edge is busy by Name.
    /// </summary>
    /// <param name="name"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{name}/busy", Name = "IsBusyEdgeByName")]
    [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyEdgeByName(string name)
    {
        try
        {
            var busy = await _locationRepository.IsBusyAsync(name);
            return Ok(busy);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }


    /// <summary>
    ///     Returns the number of containers that are deployed in a cloud entity base on cloud Id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{id}/containers/count", Name = "GetNumEdgeContainersById")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumEdgeContainersById(Guid id)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(id);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }


    /// <summary>
    ///     Returns the number of containers that are deployed in a cloud entity base on cloud Name.
    /// </summary>
    /// <param name="edgeName"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{name}/containers/count", Name = "GetNumEdgeContainersByName")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumEdgeContainersByName(string edgeName)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(edgeName);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}