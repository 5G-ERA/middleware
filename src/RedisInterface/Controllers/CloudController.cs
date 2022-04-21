using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class CloudController : ControllerBase
    {
        private readonly ICloudRepository _cloudRepository;
        private readonly ILogger _logger;

        public CloudController(ICloudRepository cloudRepository, ILogger<CloudController> logger)
        {
            _cloudRepository = cloudRepository ?? throw new ArgumentNullException(nameof(cloudRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the CloudModel entities
        /// </summary>
        /// <returns> the list of CloudModel entities </returns>
        [HttpGet(Name = "CloudGetAll")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<CloudModel>>> GetAllAsync()
        {
            try
            {
                List<CloudModel> models = await _cloudRepository.GetAllAsync();
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
        /// Get a CloudModel entity by id 
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the CloudModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "CloudGetById")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                CloudModel model = await _cloudRepository.GetByIdAsync(id);
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
        /// Add a new CloudModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created CloudModel entity </returns>
        [HttpPost(Name = "CloudAdd")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<CloudModel>> AddAsync([FromBody] CloudModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _cloudRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }

        /// <summary>
        /// Partially update an existing CloudModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified CloudModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "CloudPatch")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchCloudAsync([FromBody] CloudModel patch, [FromRoute] Guid id)
        {
            try
            {
                CloudModel model = await _cloudRepository.PatchCloudAsync(id, patch);
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
        /// Delete an CloudModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "CloudDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _cloudRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();
        }


        [HttpGet]
        [Route("relation/{name}", Name = "CloudGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _cloudRepository.GetRelation(id, name);
                if (!relations.Any())
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
        [Route("relations/{firstName}/{secondName}", Name = "CloudGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _cloudRepository.GetRelations(id, relationNames);
                if (!relations.Any())
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
