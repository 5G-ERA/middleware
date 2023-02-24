using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.Common.Services;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;


namespace Middleware.Orchestrator.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class StatusController : Controller
{
    private readonly INetAppStatusRepository _netAppStatusRepository;
    private readonly IRobotStatusRepository _robotStatusRepository;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClientService _redisInterfaceClient;

    public StatusController(INetAppStatusRepository netAppStatusRepository,
        IRobotStatusRepository robotStatusRepository,
        ILogger<StatusController> logger,
        IRedisInterfaceClientService redisInterfaceClient
        )
    {
        _netAppStatusRepository = netAppStatusRepository;
        _robotStatusRepository = robotStatusRepository;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
    }
    /// <summary>
    /// Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("robot", Name = "RobotStatusGetById")]
    [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RobotStatusModel>> GetRobotStatusByIdAsync(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int) HttpStatusCode.BadRequest,
                "Identifier of the robot must be specified"));
        }
        try
        {
            //TODO: put behind service
            var status = await _robotStatusRepository.GetByIdAsync(id);
            if (status is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Status for the specified robot was not found."));
            }
            return Ok(status);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
    /// <summary>
    /// Add a new RobotStatusModel entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("robot", Name = "RobotStatusAdd")]
    [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RobotStatusModel>> AddRobotStatusAsync([FromBody] RobotStatusModel model)
    {
        if (model.IsValid() == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        }
        try
        {
            //TODO: put behind service
            await _robotStatusRepository.AddAsync(model, () => model.Id);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }

    /// <summary>
    /// Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("netapp", Name = "NetAppStatusGetById")]
    [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<NetAppStatusModel>> GetNetAppStatusByIdAsync(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Identifier of the NetApp must be specified"));
        }
        try
        {
            //TODO: put behind service
            var status = await _netAppStatusRepository.GetByIdAsync(id);
            if (status is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Status for the specified robot was not found."));
            }
            return Ok(status);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
    /// <summary>
    /// Add a new RobotStatusModel entity
    /// </summary>
    /// <param name="model"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("netapp", Name = "NetAppStatusAdd")]
    [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<NetAppStatusModel>> AddNetAppStatusAsync([FromBody] NetAppStatusModel model)
    {
        if (model.IsValid() == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        }
        try
        {
            //TODO: put behind service
            await _netAppStatusRepository.AddAsync(model, () => model.Id);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
        return Ok(model);
    }
    /// <summary>
    /// Gets the status of the NetApps by the id of an instance definition
    /// </summary>
    /// <param name="id">Identifier of the instance definition</param>
    /// <returns></returns>
    [HttpGet]
    [Route("netapp/instance", Name = "NetAppStatusGetByInstanceId")]
    [ProducesResponseType(typeof(List<NetAppStatusModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<List<NetAppStatusModel>>> GetNetAppStatusByInstanceAsync(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Identifier of the instance definition must be specified"));
        }

        try
        {
            //TODO: put behind service
            
            var allActionPlans = await _redisInterfaceClient.ActionPlanGetAllAsync();
            var instanceIds = allActionPlans 
                .SelectMany(a => a.ActionSequence.SelectMany(x => x.Services))
                .Where(i => i.Id == id)
                .Select(i => i.ServiceInstanceId)
                .ToHashSet();

            if (instanceIds.Any() == false)
            {
                return NotFound(new ApiResponse((int) HttpStatusCode.NotFound,
                    $"No deployed instances of {id} were found"));
            }

            var statuses = new List<NetAppStatusModel>();
            foreach (var instanceId in instanceIds)
            {
                var status = await _netAppStatusRepository.GetByIdAsync(instanceId);
                if (status is not null)
                {
                    statuses.Add(status);
                }
            }
            return Ok(statuses);
        }
        catch (Exception ex)
        {
            int statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}