using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Helpers;
using Middleware.Common.Responses;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Services;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class DashboardController : MiddlewareController
{
    private readonly IDashboardService _dashboardService;
    private readonly ILogger<DashboardController> _logger;
    private readonly IUriService _uriService;

    public DashboardController(IDashboardService dashboardService, ILogger<DashboardController> logger,
        IUriService uriService)
    {
        _uriService = uriService;
        _logger = logger;
        _dashboardService = dashboardService;
    }

    /// <summary>
    ///     Basic control grid for robot-tasks
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    [HttpGet("tasks")]
    [ProducesResponseType(typeof(PagedResponse<List<TaskRobotResponse>>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetTaskRobotResponseAsync([FromQuery] PaginationFilter filter)
    {
        try
        {
            var route = Request.Path.Value;
            var (data, count) = await _dashboardService.GetRobotStatusListAsync(filter);

            var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route!);
            return Ok(pagedResponse);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Basic control grid for edge and cloud
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    [HttpGet("locations")]
    [ProducesResponseType(typeof(PagedResponse<List<LocationStatusResponse>>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetLocationStatusesAsync([FromQuery] PaginationFilter filter)
    {
        try
        {
            var route = Request.Path.Value;
            var (data, count) = await _dashboardService.GetLocationsStatusListAsync(filter);

            var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route!);
            return Ok(pagedResponse);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return to the cascading grid the action sequence names for all tasks
    /// </summary>
    /// <returns></returns>
    [HttpGet("tasks/actions")]
    [ProducesResponseType(typeof(ActionSequenceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetActionSequenceAsync()
    {
        try
        {
            var actionsAndTask = await _dashboardService.GetActionSequenceAsync();
            return Ok(actionsAndTask);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return the onboarding types names => Drop down menu.
    /// </summary>
    /// <returns></returns>
    [HttpGet("types")]
    [ProducesResponseType(typeof(ActionSequenceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public IActionResult GetOnboardingItemTypesAsync()
    {
        try
        {
            var onboardingTypes = _dashboardService.GetOnboardingItemNames();
            return Ok(onboardingTypes);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Basic control grid for netApps
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    [HttpGet("netApps")]
    [ProducesResponseType(typeof(PagedResponse<List<NetAppsDetailsResponse>>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetNetAppsDataAsync([FromQuery] PaginationFilter filter)
    {
        try
        {
            var route = Request.Path.Value;
            var (data, count) = await _dashboardService.GetNetAppsDataListAsync(filter);

            var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route!);
            return Ok(pagedResponse);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Basic control grid for robots along
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    [HttpGet("robots")]
    [ProducesResponseType(typeof(PagedResponse<List<DashboardRobotResponse>>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRobotsAsync([FromQuery] PaginationFilter filter)
    {
        try
        {
            var route = Request.Path.Value;
            var (data, count) = await _dashboardService.GetRobotsDataAsync(filter);

            var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route!);
            return Ok(pagedResponse);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return the Graph
    /// </summary>
    /// <returns></returns>
    [HttpGet("graph")]
    [ProducesResponseType(typeof(GraphResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetGraphAsync()
    {
        try
        {
            var relations = await _dashboardService.GetAllRelationModelsAsync();

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
    ///     Return the organization structure
    /// </summary>
    /// <returns></returns>
    [HttpGet("org/{orgName}")]
    [ProducesResponseType(typeof(OrgStructureResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetOrganizationStructureAsync(string orgName = "5G-ERA")
    {
        try
        {
            var structure = await _dashboardService.GetOrganizationStructureAsync(orgName);
            if (structure is null || !structure.Any())
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(orgName),
                    $"The structure for the following organization {orgName} was not found.");
            }

            var resp = new OrgStructureResponse
            {
                Locations = structure
            };
            return Ok(resp);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}