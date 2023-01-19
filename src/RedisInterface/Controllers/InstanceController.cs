using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class InstanceController : ControllerBase
    {
        private readonly IInstanceRepository _instanceRepository;
        private readonly ILogger _logger;

        public InstanceController(IInstanceRepository instanceRepository, ILogger<InstanceController> logger)
        {
            _instanceRepository = instanceRepository ?? throw new ArgumentNullException(nameof(instanceRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the InstanceModel entities
        /// </summary>
        /// <returns> the list of InstanceModel entities </returns>
        [HttpGet(Name = "InstanceGetAll")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<InstanceModel>>> GetAllAsync()
        {
            try
            {
                List<InstanceModel> models = await _instanceRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));
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
        /// Get an InstanceModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the InstanceModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "InstanceGetById")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                InstanceModel model = await _instanceRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
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
        /// Add a new InstanceModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created InstanceModel entity </returns>
        [HttpPost(Name = "InstanceAdd")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<InstanceModel>> AddAsync([FromBody] InstanceModel model)
        {
            if (model == null)
            {
                return BadRequest("Parameters were not specified.");
            }
            if (model.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }
            try 
            { 
                InstanceModel instance = await _instanceRepository.AddAsync(model);
                if (instance is null)
                {
                    return StatusCode((int) HttpStatusCode.InternalServerError,
                        new ApiResponse((int) HttpStatusCode.InternalServerError,
                            "Could not add the instance to the data store"));
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

        /// <summary>
        /// Partially update an existing InstanceModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "InstancePatch")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchInstanceAsync([FromBody] InstanceModel patch, [FromRoute] Guid id)
        {
            if (patch.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }
            try
            {
                InstanceModel model = await _instanceRepository.PatchInstanceAsync(id, patch);
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
        /// Delete an InstanceModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "InstanceDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                var deleted = await _instanceRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified Instance has not been found."));
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

        /// <summary>
        /// Creates a new relation between two models
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        [HttpPost]
        [Route("AddRelation", Name = "InstanceAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                bool isValid = await _instanceRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError, new ApiResponse((int)HttpStatusCode.NotFound, "The relation was not created"));
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

        
        /// <summary>
        /// Deletes a new relation between two models
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        [HttpDelete]
        [Route("DeleteRelation", Name = "InstanceDeleteRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                bool isValid = await _instanceRepository.DeleteRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError, new ApiResponse((int)HttpStatusCode.NotFound, "The relation was not deleted"));
                }
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok();
        }

        /// <summary>
        /// Retrieves a single relation by name
        /// </summary>
        /// <param name="id"></param>
        /// <param name="name"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relation/{name}", Name = "InstanceGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _instanceRepository.GetRelation(id, name);
                if (!relations.Any()) 
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
                }
                return Ok(relations);
            }
            catch(Exception ex) 
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Retrieves two relations by their names
        /// </summary>
        /// <param name="id"></param>
        /// <param name="firstName"></param>
        /// <param name="secondName"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "InstanceGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _instanceRepository.GetRelations(id, relationNames);
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

        /// <summary>
        /// Return alternative instance of the same instance family of the provided Instance Id.
        /// </summary>
        /// <param name="instanceId"></param>
        /// <returns>InstanceModel</returns>
        [HttpGet]
        [Route("alternative/{id}", Name = "InstanceGetAlternative")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> FindAlternativeInstance(Guid id)
        {
            try
            {
                var alternativeInstance = await _instanceRepository.FindAlternativeInstance(id);
                if (alternativeInstance == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
                }
                return Ok(alternativeInstance);
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
