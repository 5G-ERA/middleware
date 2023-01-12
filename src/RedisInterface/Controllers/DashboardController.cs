﻿using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Helpers;
using Middleware.Common.Responses;
using Middleware.RedisInterface.Responses;
using Middleware.RedisInterface.Services;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class DashboardController : ControllerBase
    {
        private readonly IDashboardService _dashboardService;
        private readonly ILogger<DashboardController> _logger;
        private readonly IUriService _uriService;
        public DashboardController(IDashboardService dashboardService, ILogger<DashboardController> logger, IUriService uriService)
        {
            _uriService = uriService;
            _logger = logger;
            _dashboardService = dashboardService;
        }

        /// <summary>
        /// Basic control grid for robot-tasks
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
                (var data, int count) = await _dashboardService.GetRobotStatusListAsync(filter);

                var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route);
                return Ok(pagedResponse);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        /// Basic control grid for edge and cloud
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
                (var data, int count) = await _dashboardService.GetLocationsStatusListAsync(filter);

                var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route);
                return Ok(pagedResponse);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Return to the cascading grid the action sequence names for all tasks
        /// </summary>
        /// <returns></returns>
        [HttpGet("tasks/actions")]
        [ProducesResponseType(typeof(ActionSequenceResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetActionSequenceAsync()
        {
            try
            {
                List<ActionSequenceResponse> actionsAndTask = await _dashboardService.GetActionSequenceAsync();
                return Ok(actionsAndTask);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Return the onboarding types names => Drop down menu.
        /// </summary>
        /// <returns></returns>
        [HttpGet("types")]
        [ProducesResponseType(typeof(ActionSequenceResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetOnboardingItemTypesAsync()
        {
            try
            {
                List<string> onboardingTypes = _dashboardService.GetOnboardingItemNames();
                return Ok(onboardingTypes);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Basic control grid for netApps
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
                (var data, int count) = await _dashboardService.GetNetAppsDataListAsync(filter);

                var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route);
                return Ok(pagedResponse);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Basic control grid for robots along
        /// </summary>
        /// <param name="filter"></param>
        /// <returns></returns>
        [HttpGet("robots")]
        [ProducesResponseType(typeof(PagedResponse<List<RobotResponse>>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRobotsAsync([FromQuery] PaginationFilter filter)
        {
            try
            {
                var route = Request.Path.Value;
                (var data, int count) = await _dashboardService.GetRobotsDataAsync(filter);

                var pagedResponse = data.ToPagedResponse(filter, count, _uriService, route);
                return Ok(pagedResponse);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Return the Graph
        /// </summary>
        /// <returns></returns>
        [HttpGet("dashboard/graph")]
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
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
    }
}
