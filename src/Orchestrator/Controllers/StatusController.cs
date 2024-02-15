using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Contracts.Mappings;
using Middleware.Orchestrator.Contracts.Requests;
using Middleware.Orchestrator.Contracts.Responses;
using Middleware.Orchestrator.ExtensionMethods;
using Middleware.Orchestrator.Heartbeat;
using Middleware.Orchestrator.Models;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class StatusController : Controller
{
    private readonly IHeartbeatService _heartbeatService;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly IInfluxNetAppStatusRepository _influxNetAppStatusRepository;

    public StatusController(ILogger<StatusController> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IHeartbeatService heartbeatService,
        IInfluxNetAppStatusRepository influxNetAppStatusRepository
    )
    {
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _heartbeatService = heartbeatService;
        _influxNetAppStatusRepository = influxNetAppStatusRepository;
    }


    /// <summary>
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("robot", Name = "RobotStatusGetAll")]
    [ProducesResponseType(typeof(GetRobotsHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RobotStatusModel>> GetRobotStatusesAsync(bool generateFakeData = false)
    {
        try
        {
            var status = await _heartbeatService.GetAllRobotStatusesAsync(generateFakeData);
            if (status is null || !status.Any())
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "No robot statuses were found."));
            }

            var resp = status.ToRobotsHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("robot/{id}", Name = "RobotStatusGetById")]
    [ProducesResponseType(typeof(GetRobotHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<GetRobotHeartbeatResponse>> GetRobotStatusByIdAsync(Guid id,
        bool generateFakeData = false)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Identifier of the robot must be specified"));
        }

        try
        {
            var status = await _heartbeatService.GetRobotStatusByIdAsync(id, generateFakeData);
            if (status is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "Status for the specified robot was not found."));
            }

            var resp = status.ToRobotHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Add a new RobotStatusModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("robot", Name = "RobotStatusAdd")]
    [ProducesResponseType(typeof(GetRobotHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<GetRobotHeartbeatResponse>> AddRobotStatusAsync(
        [FromBody] CreateRobotHeartbeatRequest request)
    {
        try
        {
            var result = await _heartbeatService.AddRobotStatus(request.ToRobotStatus());
            if (result.IsSuccess == false && result.NotFound)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, result.ErrMessage));
            }

            if (result.IsSuccess == false && result.NotFound == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, result.ErrMessage));
            }

            var resp = result.Value.ToRobotHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("netapp", Name = "NetAppStatuses")]
    [ProducesResponseType(typeof(GetNetAppsHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<GetNetAppsHeartbeatResponse>> GetNetAppStatusesAsync(bool generateFakeData = false)
    {
        try
        {
            var status = await _heartbeatService.GetAllAppStatusesAsync(generateFakeData);
            if (status is null || !status.Any())
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "No NetApp statuses were found."));
            }

            var resp = status.ToNetAppsHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("netapp/{id}", Name = "NetAppStatusGetById")]
    [ProducesResponseType(typeof(GetNetAppHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<GetNetAppHeartbeatResponse>> GetNetAppStatusByIdAsync(Guid id, bool generateFakeData = false)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Identifier of the NetApp must be specified"));
        }

        try
        {
            var status = await _heartbeatService.GetNetAppStatusByIdAsync(id, generateFakeData);
            if (status is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "Status for the specified NetApp was not found."));
            }

            var resp = status.ToNetAppHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Add a new RobotStatusModel entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the newly created RobotStatusModel entity </returns>
    [HttpPost]
    [Route("netapp", Name = "NetAppStatusAdd")]
    [ProducesResponseType(typeof(GetNetAppHeartbeatResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<GetNetAppHeartbeatResponse>> AddNetAppStatusAsync(
        [FromBody] CreateNetAppHeartbeatRequest request)
    {
        try
        {
            var model = request.ToNetAppStatus();
            //TODO: put behind service
            //await _netAppStatusRepository.AddAsync(model, () => model.Id);
            await _influxNetAppStatusRepository.AddAsync(model);

            var resp = model.ToNetAppHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Gets the status of the NetApps by the id of an instance definition
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
            if (allActionPlans is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "No action plans were found"));
            }

            var instanceIds = allActionPlans
                .SelectMany(a => a.ActionSequence!.SelectMany(x => x.Services))
                .Where(i => i.Id == id)
                .Select(i => i.ServiceInstanceId)
                .ToHashSet();

            if (instanceIds.Any() == false)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    $"No deployed instances of {id} were found"));
            }

            var statuses = new List<NetAppStatusModel>();
            foreach (var instanceId in instanceIds)
            {
                //var status = await _netAppStatusRepository.GetByIdAsync(instanceId);
                //  NetAppStatusModel // 7
                var status = await _influxNetAppStatusRepository.GetStatusByIdAsync(instanceId);
                if (status is not null) statuses.Add(status);
            }

            return Ok(statuses);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}