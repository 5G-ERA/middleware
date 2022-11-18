using System.Net;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
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
        public DashboardController(IDashboardService dashboardService, ILogger<DashboardController> logger)
        {
            _logger = logger;
            _dashboardService = dashboardService;
        }

        [HttpGet]
        [ProducesResponseType(typeof(ApiResponse<List<TaskRobotResponse>>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<ApiResponse>> GetTaskRobotResponseAsync()
        {
            try
            {
                var data = await _dashboardService.GetRobotStatusListAsync();

                return Ok(new ApiResponse<List<TaskRobotResponse>>(data));
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
