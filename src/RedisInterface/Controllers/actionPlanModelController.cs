using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class actionPlanModelController : ControllerBase
    {
        private readonly ILogger _logger;
        private readonly IActionPlanRepository _actionPlanRepository;

        public actionPlanModelController(IActionPlanRepository actionPlanRepository, ILogger<actionPlanModelController> logger)
        {
            _actionPlanRepository = actionPlanRepository ?? throw new ArgumentNullException(nameof(actionPlanRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get action plan given robot Id.
        /// </summary>
        /// <returns>List<ActionPlanModel></returns>
        [HttpPost]
        [Route("{id}", Name = "ActionPlanModelById")]
        [ProducesResponseType(typeof(List<ActionPlanModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<ActionPlanModel>>> GetActionPlanModelsAsync([FromRoute] Guid id)
        {
            try
            {
                // Get list of actionPlans from specific robotId.
                List<ActionPlanModel> actionPlans = await _actionPlanRepository.GetActionPlanModelsAsync(id); 
                if (actionPlans == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }
                //List<ActionPlanModel> activePoliciesRecords = actionPlans.Select(p => new ActionPlanModel(p.Id, p.Name, p.Description)).ToList();
                return Ok(actionPlans);
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
