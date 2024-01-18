using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Heartbeat;
using Middleware.Orchestrator.Models.Responses;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class StatusController : Controller
{
    private readonly IHeartbeatService _heartbeatService;
    private readonly ILogger _logger;
    private readonly INetAppStatusRepository _netAppStatusRepository;
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly IRobotStatusRepository _robotStatusRepository;
    private readonly IInfluxNetAppStatusRepository _influxNetAppStatusRepository;
    private readonly IInfluxRobotStatusRepository _influxRobotStatusRepository;

    public StatusController(INetAppStatusRepository netAppStatusRepository,
        IRobotStatusRepository robotStatusRepository,
        ILogger<StatusController> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IHeartbeatService heartbeatService,
        IInfluxNetAppStatusRepository influxNetAppStatusRepository,
        IInfluxRobotStatusRepository influxRobotStatusRepository
    )
    {
        _netAppStatusRepository = netAppStatusRepository;
        _robotStatusRepository = robotStatusRepository;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _heartbeatService = heartbeatService;
        _influxNetAppStatusRepository = influxNetAppStatusRepository;
        _influxRobotStatusRepository = influxRobotStatusRepository;
    }


    /// <summary>
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("robot", Name = "RobotStatusGetAll")]
    [ProducesResponseType(typeof(GetRobotStatusesResponse), (int)HttpStatusCode.OK)]
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

            var resp = new GetRobotStatusesResponse
            {
                Robots = status
            };
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
    [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<RobotStatusModel>> GetRobotStatusByIdAsync(Guid id, bool generateFakeData = false)
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

            return Ok(status);
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
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            //TODO: put behind service
            //await _robotStatusRepository.AddAsync(model, () => model.Id);
            await _influxRobotStatusRepository.AddOneAsync(model);
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
    ///     Get the robot status
    /// </summary>
    /// <returns> the list of RobotStatusModel entities </returns>
    [HttpGet]
    [Route("netapp", Name = "NetAppStatuses")]
    [ProducesResponseType(typeof(GetNetAppStatusesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<NetAppStatusModel>> GetNetAppStatusesAsync(bool generateFakeData = false)
    {
        try
        {
            var status = await _heartbeatService.GetAllAppStatusesAsync(generateFakeData);
            if (status is null || !status.Any())
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                    "No NetApp statuses were found."));
            }

            var resp = new GetNetAppStatusesResponse
            {
                NetApps = status
            };
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
    [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<NetAppStatusModel>> GetNetAppStatusByIdAsync(Guid id, bool generateFakeData = false)
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

            return Ok(status);
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
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            //TODO: put behind service
            //await _netAppStatusRepository.AddAsync(model, () => model.Id);
            await _influxNetAppStatusRepository.AddOneAsync(model);
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
                var status = await _netAppStatusRepository.GetByIdAsync(instanceId);
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