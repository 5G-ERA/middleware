using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class EdgeController : ControllerBase
    {
        private readonly IEdgeRepository _edgeRepository;
        private readonly ILogger _logger;

        public EdgeController(IEdgeRepository edgeRepository, ILogger<EdgeController> logger)
        {
            _edgeRepository = edgeRepository ?? throw new ArgumentNullException(nameof(edgeRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the EdgeModel entities
        /// </summary>
        /// <returns> the list of EdgeModel entities </returns>
        [HttpGet(Name = "EdgeGetAll")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<EdgeModel>>> GetAllAsync()
        {
            try
            {
                List<EdgeModel> models = await _edgeRepository.GetAllAsync();
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
        /// Get an EdgeModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the EdgeModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "EdgeGetById")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                EdgeModel model = await _edgeRepository.GetByIdAsync(id);
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
        /// Add a new EdgeModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created EdgeModel entity </returns>
        [HttpPost(Name = "EdgeAdd")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<EdgeModel>> AddAsync([FromBody] EdgeModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                EdgeModel edge = await _edgeRepository.AddAsync(model);
                if (edge is null)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.NotFound,
                            "Problem while adding the Edge to the data store"));
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
        [Route("{id}", Name = "EdgePatch")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchEdgeAsync([FromBody] EdgeModel patch, [FromRoute] Guid id)
        {

            try
            {
                EdgeModel model = await _edgeRepository.PatchEdgeAsync(id, patch);
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
        /// Delete an EdgeModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "EdgeDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                var deleted = await _edgeRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified Edge has not been found."));
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
        [Route("AddRelation", Name = "EdgeAddRelation")]
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
                bool isValid = await _edgeRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
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
        /// Retrieves a single relation by name
        /// </summary>
        /// <param name="id"></param>
        /// <param name="name"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relation/{name}", Name = "EdgeGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _edgeRepository.GetRelation(id, name);
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

        /// <summary>
        /// Retrieves two relations by their names
        /// </summary>
        /// <param name="id"></param>
        /// <param name="firstName"></param>
        /// <param name="secondName"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "EdgeGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _edgeRepository.GetRelations(id, relationNames);
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

        [HttpPost]
        [Route("free", Name = "GetFreeEdgesIdsAsync")]//edges
        [ProducesResponseType(typeof(List<Guid>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<Guid>>> GetFreeEdgesIdsAsync(List<Guid> edgesToCheck)
        {
            try
            {
                if (!edgesToCheck.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));
                }

                List<Guid> edgeFree = await _edgeRepository.GetFreeEdgesIdsAsync(edgesToCheck);
                if (!edgeFree.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no edges connected to the Robot"));
                }
                return Ok(edgeFree);
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
