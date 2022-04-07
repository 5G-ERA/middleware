using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.JsonPatch;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ActionController : ControllerBase
    {
        private readonly IActionRepository _actionRepository;
        private readonly ILogger _logger;

        public ActionController(IActionRepository actionRepository, ILogger<ActionController> logger)
        {
            _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the ActionModel entities
        /// </summary>
        /// <returns> the list of ActionModel entities </returns>
        [HttpGet(Name = "ActionGetAll")]
        [ProducesResponseType(typeof(List<ActionModel>), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<ActionModel>>> GetAllAsync()
        {
            try
            {
                List<ActionModel> models = await _actionRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound("Objects were not found.");
                }
                return Ok(models);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
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
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                ActionModel model = await _actionRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound("Object was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Add a new ActionModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created ActionModel entity </returns>
        [HttpPost(Name = "ActionAdd")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ActionModel>> AddAsync([FromBody] ActionModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _actionRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
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
        [Route("{id}", Name ="ActionPatch")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchActionAsync([FromBody] ActionModel patch, [FromRoute] Guid id)
        {
            try
            {
                ActionModel model = await _actionRepository.PatchActionAsync(id, patch);
                if (model == null)
                {
                    return NotFound("Object to be updated was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        
        /// <summary>
        /// Delete an ActionModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name ="ActionDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _actionRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();

        }


        [HttpGet]
        [Route("relation/{name}", Name = "ActionGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _actionRepository.GetRelation(id, name);
                if (relations.Any())
                {
                    return NotFound("Relations were not found.");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "ActionGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _actionRepository.GetRelations(id, relationNames);
                if (relations.Any())
                {
                    return NotFound("Relations were not found");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }
    }
}
