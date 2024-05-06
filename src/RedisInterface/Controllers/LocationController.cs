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
public class LocationController : MiddlewareController
{
    private readonly ILocationRepository _locationRepository;
    private readonly ILogger _logger;

    public LocationController(ILocationRepository locationRepository, ILogger<LocationController> logger)
    {
        _locationRepository = locationRepository;
        _logger = logger;
    }

    /// <summary>
    ///     Get all the Location entities
    /// </summary>
    /// <returns> the list of Location entities </returns>
    [HttpGet(Name = "LocationGetAll")]
    [ProducesResponseType(typeof(GetLocationsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var models = await _locationRepository.GetAllAsync();
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "locations", $"Locations were not found.");

            var response = models.ToLocationsResponse();
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
    ///     Get an LocationModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the LocationModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "LocationGetById")]
    [ProducesResponseType(typeof(LocationResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            var model = await _locationRepository.GetByIdAsync(id);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Location with id {id} was not found.");

            var response = model.ToLocationResponse();
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
    ///     Add a new Location entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created Location entity </returns>
    [HttpPost(Name = "LocationAdd")]
    [ProducesResponseType(typeof(LocationResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] LocationRequest request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var model = request.ToLocation();
            var existingLoc = await _locationRepository.GetByNameAsync(model.Name);
            if (existingLoc is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name),
                    $"Location with specified name already exists.");
            }

            var location = await _locationRepository.AddAsync(model);
            if (location is null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"Problem while adding the Location to the data store");
            }

            var response = location.ToLocationResponse();
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
    ///     Update an existing Location entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified Location entity </returns>
    [HttpPut]
    [Route("{id}", Name = "LocationUpdate")]
    [ProducesResponseType(typeof(LocationResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchLocationAsync([FromMultiSource] UpdateLocationRequest request)
    {
        if (request is null)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");
        }

        try
        {
            var location = request.ToLocation();
            var exists = await _locationRepository.GetByIdAsync(location.Id);
            if (exists is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(location.Id),
                    $"Location with id {location.Id} was not found.");
            }

            await _locationRepository.UpdateAsync(location);
            var response = location.ToLocationResponse();
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
    ///     Delete an LocationModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "LocationDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            if (id == Guid.Empty)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"The id cannot be empty.");
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
    [Route("AddRelation", Name = "LocationAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified.");

        try
        {
            var isValid = await _locationRepository.AddRelationAsync(request);
            if (!isValid)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request),
                    $"The relation was not created.");
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
    [Route("relation/{name}", Name = "LocationGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), $"Relation name was not specified.");
        }

        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Location ID not specified.");
        }

        if (Enum.TryParse<RelationDirection>(direction, out var directionEnum) == false)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction),
                $"Wrong Relation direction specified");
        }

        var inputDirection = directionEnum;
        try
        {
            var relations = await _locationRepository.GetRelation(id, name, inputDirection);
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
    [Route("relations/{firstName}/{secondName}", Name = "LocationGetRelationsByName")]
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
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relations", $"Request body was not specified.");

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
    [Route("name/{name}", Name = "LocationGetDataByName")]
    [ProducesResponseType(typeof(LocationResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLocationByNameAsync(string name)
    {
        try
        {
            var location = await _locationRepository.GetByNameAsync(name);
            if (location is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(name),
                    $"Location with name {name} was not found.");

            var response = location.ToLocationResponse();
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
    ///     Check if a Location is busy by Id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{id}/busy", Name = "IsBusyLocationById")]
    [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyLocationById(Guid id)
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
    ///     Check if a Location is busy by Name.
    /// </summary>
    /// <param name="name"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{name}/busy", Name = "IsBusyLocationByName")]
    [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyLocationByName(string name)
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
    ///     Returns the number of containers that are deployed in a location base on location Id.
    /// </summary>
    /// <param name="id"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{id}/containers/count", Name = "GetNumLocationContainersById")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumLocationContainersById(Guid id)
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
    /// <param name="name"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{name}/containers/count", Name = "GetNumLocationContainersByName")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumLocationContainersByName(string name)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(name);
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