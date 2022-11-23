using System.Net;
using Microsoft.AspNetCore.Http;
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

        [HttpGet("locations")]
        [ProducesResponseType(typeof(PagedResponse<List<LocationStatusResponse>>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetCloudRobotStatusesAsync([FromQuery] PaginationFilter filter)
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
    }
}
