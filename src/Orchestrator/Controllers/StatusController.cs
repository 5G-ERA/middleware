using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Contracts.Mappings;
using Middleware.Orchestrator.Contracts.Requests;
using Middleware.Orchestrator.Contracts.Responses;
using Middleware.Orchestrator.Heartbeat;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class StatusController : MiddlewareController
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
    public async Task<IActionResult> GetRobotStatusesAsync(bool generateFakeData = false)
    {
        try
        {
            var status = await _heartbeatService.GetAllRobotStatusesAsync(generateFakeData);
            if (status is null || !status.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "plan", $"No robot statuses were found.");
            }

            var resp = status.ToRobotsHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> GetRobotStatusByIdAsync(Guid id,
        bool generateFakeData = false)
    {
        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id),
                "Identifier of the robot must be specified");
        }

        try
        {
            var status = await _heartbeatService.GetRobotStatusByIdAsync(id, generateFakeData);
            if (status is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    "Status for the specified robot was not found.");
            }

            var resp = status.ToRobotHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> AddRobotStatusAsync(
        [FromBody] CreateRobotHeartbeatRequest request)
    {
        try
        {
            var result = await _heartbeatService.AddRobotStatus(request.ToRobotStatus());
            if (result.IsSuccess == false && result.NotFound)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id), result.ErrMessage);
            }

            if (result.IsSuccess == false && result.NotFound == false)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Id), result.ErrMessage);
            }

            var resp = result.Value.ToRobotHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.NotFound, "system", $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> GetNetAppStatusesAsync(bool generateFakeData = false)
    {
        try
        {
            var status = await _heartbeatService.GetAllAppStatusesAsync(generateFakeData);
            if (status is null || !status.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, "netApp", "No NetApp statuses were found.");
            }

            var resp = status.ToNetAppsHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> GetNetAppStatusByIdAsync(Guid id,
        bool generateFakeData = false)
    {
        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id),
                "Identifier of the NetApp must be specified");
        }

        try
        {
            var status = await _heartbeatService.GetNetAppStatusByIdAsync(id, generateFakeData);
            if (status is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    "Status for the specified NetApp was not found.");
            }

            var resp = status.ToNetAppHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> AddNetAppStatusAsync([FromBody] CreateNetAppHeartbeatRequest request)
    {
        try
        {
            var model = request.ToNetAppStatus();
            var result = await _heartbeatService.AddNetAppStatus(model);
            if (result.IsSuccess == false && result.NotFound)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id), result.ErrMessage);
            }

            if (result.IsSuccess == false && result.NotFound == false)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request.Id), result.ErrMessage);
            }

            var resp = result.Value.ToNetAppHeartbeatResponse();
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
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
    public async Task<IActionResult> GetNetAppStatusByInstanceAsync(Guid id)
    {
        if (id == Guid.Empty)
        {
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id),
                "Identifier of the instance definition must be specified");
        }

        try
        {
            //TODO: put behind service
            var allActionPlans = await _redisInterfaceClient.ActionPlanGetAllAsync();
            if (allActionPlans is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), "No action plans were found");
            }

            var instanceIds = allActionPlans
                .SelectMany(a => a.ActionSequence!.SelectMany(x => x.Services))
                .Where(i => i.Id == id)
                .Select(i => i.ServiceInstanceId)
                .ToHashSet();

            if (instanceIds.Any() == false)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"No deployed instances of {id} were found");
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
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}