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
public class CloudController : MiddlewareController
{
    private readonly ILocationRepository _locationRepository;
    private readonly ILogger _logger;

    public CloudController(ILocationRepository locationRepository, ILogger<CloudController> logger)
    {
        _locationRepository = locationRepository ?? throw new ArgumentNullException(nameof(locationRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }

    /// <summary>
    ///     Get all the CloudModel entities
    /// </summary>
    /// <returns> the list of CloudModel entities </returns>
    [HttpGet(Name = "CloudGetAll")]
    [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllAsync()
    {
        try
        {
            var cloudType = LocationType.Cloud.ToString();
            var models = await _locationRepository.FindAsync(c => c.Type == cloudType);
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "cloud", $"No Clouds were found.");
            
            return Ok(models.ToCloudsResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get a CloudModel entity by id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> the CloudModel entity for the specified id </returns>
    [HttpGet]
    [Route("{id}", Name = "CloudGetById")]
    [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetByIdAsync(Guid id)
    {
        try
        {
            var cloudType = LocationType.Cloud.ToString();
            var model = await _locationRepository.FindSingleAsync(l =>
                l.Id == id.ToString() && l.Type == cloudType);
            if (model is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Cloud with specified id: '{id}' was not found.");
            }

            return Ok(model.ToCloudResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }


    /// <summary>
    ///     Add a new CloudModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created CloudModel entity </returns>
    [HttpPost(Name = "CloudAdd")]
    [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddAsync([FromBody] CloudRequest request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"The request body was not specified");
        
        try
        {
            var existingLoc = await _locationRepository.GetByNameAsync(request.Name);
            if (existingLoc is not null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Name), "Location with specified name already exists");
            }
            var location = await _locationRepository.AddAsync(request.ToLocation());
            if (location is null)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Could not add the cloud to the data store");
            }

            return Ok(location.ToCloudResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Partially update an existing CloudModel entity
    /// </summary>
    /// <param name="patch"></param>
    /// <returns> the modified CloudModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "CloudPatch")]
    [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> PatchCloudAsync([FromMultiSource] UpdateCloudRequest patch)
    {
        try
        {
            var location = patch.ToLocation();
            await _locationRepository.UpdateAsync(location);
            var response = location.ToCloudResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Delete an CloudModel entity for the given id
    /// </summary>
    /// <param name="id"></param>
    /// <returns> no return </returns>
    [HttpDelete]
    [Route("{id}", Name = "CloudDelete")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteByIdAsync(Guid id)
    {
        try
        {
            var deleted = await _locationRepository.DeleteByIdAsync(id);
            if (deleted == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Cloud with specified id: '{id}' was not found.");
            }

            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("AddRelation", Name = "CloudAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> AddRelationAsync([FromBody] RelationModel request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"Request body was not specified");
        try
        {
            var isValid = await _locationRepository.AddRelationAsync(request);
            if (!isValid)
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"The relation was not created");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
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
    [Route("relation/{name}", Name = "CloudGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = "Outgoing")
    {
        if (string.IsNullOrWhiteSpace(name))
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(name), $"Relation name not specified");
        if (id == Guid.Empty)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), $"Relation id not specified");
        if (Enum.TryParse(direction, out RelationDirection directionEnum) == false)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(direction), $"Wrong Relation direction specified");
        
        var inputDirection = directionEnum;
        try
        {
            var relations = await _locationRepository.GetRelation(id, name, inputDirection);
            if (!relations.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(name), $"No relation was found.");
                
            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
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
    [Route("relations/{firstName}/{secondName}", Name = "CloudGetRelationsByName")]
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
                return ErrorMessageResponse(HttpStatusCode.NotFound, "relation", $"No relations were found.");
                
            return Ok(relations);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("name/{name}", Name = "CloudGetDataByName")]
    [ProducesResponseType(typeof(CloudResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetCloudResourceDetailsByNameAsync(string name)
    {
        try
        {
            var cloudResource = await _locationRepository.GetByNameAsync(name);
            if (cloudResource is null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(name), $"No cloud was found.");
                
            return Ok(cloudResource.ToCloudResponse());
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get the free clouds that have connectivity to the robot
    /// </summary>
    /// <param name="request"></param>
    /// <returns>list of cloudModel</returns>
    [HttpGet]
    [Route("free", Name = "GetFreeCloudIds")]
    [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetFreeCloudsIdsAsync(List<CloudModel> request)
    {
        try
        {
            if (!request.Any())
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"No cloud were specified.");

            var freeClouds = await _locationRepository.FilterFreeLocationsAsync(request.ToLocations());
            if (!freeClouds.Any())
                return ErrorMessageResponse(HttpStatusCode.NotFound, "clouds", $"There are no free clouds");

            var response = freeClouds.ToCloudsResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("lessBusy", Name = "GetLessBusyClouds")]
    [ProducesResponseType(typeof(GetCloudsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLessBusyCloudsAsync(List<CloudModel> request)
    {
        try
        {
            if (!request.Any())
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), $"No Cloud ids were provided");
                

            var lessBusyCloud = await _locationRepository.OrderLocationsByUtilizationAsync(request.ToLocations());
            if (!lessBusyCloud.Any())
                return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no busy edges"));

            var response = lessBusyCloud.ToCloudsResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Returns the number of containers that are deployed in a cloud entity base on cloud Name.
    /// </summary>
    /// <param name="name"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{name}/containers/count", Name = "GetNumCloudContainersByName")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumCloudContainersByName(string name)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(name);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Returns the number of containers that are deployed in a cloud entity base on cloud Id.
    /// </summary>
    /// <param name="cloudId"></param>
    /// <returns>int</returns>
    [HttpGet]
    [Route("{id}/containers/count", Name = "GetNumCloudContainersById")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNumCloudContainersById(Guid cloudId)
    {
        try
        {
            var countContainers = await _locationRepository.GetDeployedInstancesCountAsync(cloudId);
            return Ok(countContainers);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Returns bool for status of BusyCloud by Name
    /// </summary>
    /// <param name="name"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{name}/busy", Name = "IsBusyCloudByName")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyCloudByName(string name)
    {
        try
        {
            if (string.IsNullOrWhiteSpace(name))
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Name was not provided"));

            var busy = await _locationRepository.IsBusyAsync(name);
            return Ok(busy);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Returns bool for status of BusyCloud by Id
    /// </summary>
    /// <param name="cloudId"></param>
    /// <returns>bool</returns>
    [HttpGet]
    [Route("{id}/busy", Name = "isBusyCloudById")]
    [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> IsBusyCloudById(Guid cloudId)
    {
        try
        {
            var busy = await _locationRepository.IsBusyAsync(cloudId);
            return Ok(busy);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system", $"An error has occurred: {ex.Message}");
        }
    }
}