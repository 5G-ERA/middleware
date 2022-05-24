using System.Net;
using Microsoft.AspNetCore.Mvc;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ActionController : ControllerBase
    {
        private readonly IActionRepository _actionRepository;
        private readonly IActionPlanRepository _actionPlanRepository;

        private readonly ILogger _logger;
        private readonly IOptions<ElasticConfig> _elasticOptions;

        public ActionController(IActionRepository actionRepository, IActionPlanRepository actionPlanRepository, ILogger<ActionController> logger, IOptions<ElasticConfig> elasticOptions)
        {
            _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
            _actionPlanRepository = actionPlanRepository ?? throw new ArgumentNullException(nameof(actionPlanRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
            _elasticOptions = elasticOptions;
        }

        /// <summary>
        /// Get all the ActionModel entities
        /// </summary>
        /// <returns> the list of ActionModel entities </returns>
        [HttpGet(Name = "ActionGetAll")]
        [ProducesResponseType(typeof(List<ActionModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<ActionModel>>> GetAllAsync()
        {
            try
            {

                List<ActionModel> models = await _actionRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No actions were found."));
                }
                return Ok(models);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Get an ActionModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the ActionModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "ActionGetById")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                ActionModel model = await _actionRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, $"Action with id: '{id}' was not found."));
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Add a new ActionModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created ActionModel entity </returns>
        [HttpPost(Name = "ActionAdd")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<ActionModel>> AddAsync([FromBody] ActionModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                await _actionRepository.AddAsync(model);
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
        /// Partially update an existing ActionModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified ActionModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "ActionPatch")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchActionAsync([FromBody] ActionModel patch, [FromRoute] Guid id)
        {
            try
            {
                ActionModel model = await _actionRepository.PatchActionAsync(id, patch);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        /// Delete an ActionModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "ActionDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _actionRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok();

        }


        [HttpPost]
        [Route("AddRelation", Name = "ActionAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest("Parameters were not specified.");
            }
            try
            {
                bool isValid = await _actionRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError, new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                }
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok(model);
        }


        [HttpGet]
        [Route("relation/{name}", Name = "ActionGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            if (string.IsNullOrWhiteSpace(name))
            {
                return BadRequest(new ApiResponse((int) HttpStatusCode.BadRequest, "Relation name not specified"));
            }

            if (id == Guid.Empty)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Relation name not specified"));
            }
            try
            {
                var relations = await _actionRepository.GetRelation(id, name);
                if (!relations.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "ActionGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _actionRepository.GetRelations(id, relationNames);
                if (!relations.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found"));
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        #region ActionPlan

        [HttpGet]
        [Route("plan", Name = "ActionPlanGetAll")]
        [ProducesResponseType(typeof(List<ActionPlanModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetAllActionPlansAsync()
        {
            try
            {
                var plans = await _actionPlanRepository.GetAllAsync();
                if (plans == null || plans.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No plans have been found."));
                }
                return Ok(plans);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                int statusCode = (int)HttpStatusCode.InternalServerError;
                return StatusCode(statusCode, new ApiResponse(statusCode, ex.Message));
            }
        }

        [HttpGet]
        [Route("plan/{id:guid}", Name = "ActionPlanGetById")]
        [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetActionPlanByIdAsync(Guid id)
        {
            try
            {
                var plan = await _actionPlanRepository.GetByIdAsync(id);
                if (plan == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Specified plan was not found."));
                }
                return Ok(plan);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                int statusCode = (int)HttpStatusCode.InternalServerError;
                return StatusCode(statusCode, new ApiResponse(statusCode, ex.Message));
            }
        }
        [HttpPost]
        [Route("plan", Name = "ActionPlanAdd")]
        [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> AddActionPlanAsync(ActionPlanModel actionPlan)
        {
            try
            {
                if (actionPlan == null)
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Plan cannot be null"));
                }
                var plan = await _actionPlanRepository.AddAsync(actionPlan, () => actionPlan.Id);
                if (plan == null)
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "The specified plan has not been added."));
                }
                return Ok(plan);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
        [HttpDelete]
        [Route("plan/{id}", Name = "ActionPlanDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> DeleteActionPlanAsync(Guid id)
        {
            try
            {
                var deleted = await _actionPlanRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified plan has not been found."));
                }
                return Ok();
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        [HttpPut]
        [Route("plan/{id:guid}", Name = "ActionPlanPatch")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchActionPlanAsync(Guid id, [FromBody] ActionPlanModel actionPlan)
        {
            try
            {
                if (actionPlan == null || id == Guid.Empty)
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Id or updated object has not been specified"));
                }
                var deleted = await _actionPlanRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified plan has not been found."));
                }

                var updatedPlan = await _actionPlanRepository.AddAsync(actionPlan, () => id);
                return Ok(updatedPlan);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        #endregion

    }
}
