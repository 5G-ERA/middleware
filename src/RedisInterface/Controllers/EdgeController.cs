using System.Net;
using Microsoft.AspNetCore.Mvc;
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
public class EdgeController : ControllerBase
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));

            var response = models.ToEdgesResponse();
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
            var model = await _locationRepository.FindSingleAsync(e =>
                e.Id == id.ToString() && e.Type == LocationType.Edge.ToString());
            if (model == null) return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));

            var response = model.ToEdgeResponse();
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
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));

        try
        {
            var model = request.ToLocation();
            var edge = await _locationRepository.AddAsync(model);
            if (edge is null)
            {
                return StatusCode((int)HttpStatusCode.InternalServerError,
                    new ApiResponse((int)HttpStatusCode.NotFound,
                        "Problem while adding the Edge to the data store"));
            }

            var response = edge.ToEdgeResponse();
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
    ///     Partially update an existing Edge entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified Edge entity </returns>
    [HttpPut]
    [Route("{id}", Name = "EdgePatch")]
    [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchEdgeAsync([FromMultiSource] UpdateEdgeRequest request)
    {
        if (request is null)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Parameters were not specified or wrongly specified."));
        }

        try
        {
            var edge = request.ToLocation();
            var exists = await _locationRepository.GetByIdAsync(edge.Id);
            if (exists is null)
            {
                return NotFound(
                    new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
            }

            await _locationRepository.UpdateAsync(edge);
            var response = edge.ToEdgeResponse();
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "The id cannot be empty"));
            }

            var exists = await _locationRepository.GetByIdAsync(id);
            if (exists is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "The specified Edge has not been found."));
            }

            await _locationRepository.DeleteByIdAsync(id);
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
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "EdgeAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
    {
        if (model == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));

        try
        {
            var isValid = await _locationRepository.AddRelationAsync(model);
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
    [Route("relation/{name}", Name = "EdgeGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation name not specified"));
        if (id == Guid.Empty)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation ID not specified"));
        RelationDirection directionEnum;
        if (Enum.TryParse(direction, out directionEnum) == false)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Wrong Relation direction specified"));
        var inputDirection = directionEnum;
        try
        {
            var relations = await _locationRepository.GetRelation(id, name, inputDirection);
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

    [HttpGet]
    [Route("free", Name = "GetFreeEdgesIds")] //edges
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetFreeEdgesIdsAsync(List<EdgeModel> edgesToCheck)
    {
        try
        {
            if (!edgesToCheck.Any())
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));

            var edgeFree = await _locationRepository.FilterFreeLocationsAsync(edgesToCheck.ToLocations());
            if (!edgeFree.Any())
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest,
                    "There are no edges connected to the Robot"));
            }

            var response = edgeFree.ToEdgesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpGet]
    [Route("lessBusy", Name = "GetLessBusyEdges")]
    [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLessBusyEdgesAsync(List<EdgeModel> edgesToCheck)
    {
        try
        {
            if (!edgesToCheck.Any())
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));

            var lessBusyEdge = await _locationRepository.OrderLocationsByUtilizationAsync(edgesToCheck.ToLocations());
            if (!lessBusyEdge.Any())
                return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no busy edges"));

            var response = lessBusyEdge.ToEdgesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));

            var response = edgeResource.ToEdgeResponse();
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
    ///     Check if a edge is busy by Id.
    /// </summary>
    /// <param name="edgeId"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{id}/busy", Name = "IsBusyEdgeById")]
    [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<bool>> IsBusyEdgeById(Guid edgeId)
    {
        try
        {
            var busy = await _locationRepository.IsBusyAsync(edgeId);
            return Ok(busy);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    /// <summary>
    ///     Returns the number of containers that are deployed in a cloud entity base on cloud Id.
    /// </summary>
    /// <param name="edgeId"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{id}/containers/count", Name = "GetNumEdgeContainersById")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumEdgeContainersById(Guid edgeId)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(edgeId);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
    public async Task<ActionResult<int>> GetNumEdgeContainersByName(string edgeName)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(edgeName);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}