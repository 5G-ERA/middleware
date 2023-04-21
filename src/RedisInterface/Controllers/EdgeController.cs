using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Attributes;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;

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
        [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetAllAsync()
        {
            try
            {
                List<EdgeModel> models = await _edgeRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));
                }

                var response = models.ToEdgesResponse();
                return Ok(response);
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
        [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
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

                var response = model.ToEdgeResponse();
                return Ok(response);
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
        /// <param name="request"></param>
        /// <returns> the newly created EdgeModel entity </returns>
        [HttpPost(Name = "EdgeAdd")]
        [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> AddAsync([FromBody] EdgeRequest request)
        {
            if (request == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }

            try
            {
                var model = request.ToEdge();
                EdgeModel edge = await _edgeRepository.AddAsync(model);
                if (edge is null)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.NotFound,
                            "Problem while adding the Edge to the data store"));
                }

                var response = edge.ToEdgeResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Partially update an existing InstanceModel entity
        /// </summary>
        /// <param name="request"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPut]
        [Route("{id}", Name = "EdgePatch")]
        [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchEdgeAsync([FromMultiSource] UpdateEdgeRequest request)
        {
            if (request is null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                    "Parameters were not specified or wrongly specified."));
            }

            try
            {
                var edge = request.ToEdge();
                var exists = await _edgeRepository.GetByIdAsync(edge.Id);
                if (exists is null)
                {
                    return NotFound(
                        new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
                }
                await _edgeRepository.UpdateAsync(edge);
                var response = edge.ToEdgeResponse();
                return Ok(response);
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
        public async Task<IActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                if (id == Guid.Empty)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                        "The id cannot be empty"));
                }

                var exists = await _edgeRepository.GetByIdAsync(id);
                if (exists is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound,
                        "The specified Edge has not been found."));
                }
                await _edgeRepository.DeleteByIdAsync(id);
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

        [HttpGet]
        [Route("free", Name = "GetFreeEdgesIds")] //edges
        [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetFreeEdgesIdsAsync(List<EdgeModel> edgesToCheck)
        {
            try
            {
                if (!edgesToCheck.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));
                }

                List<EdgeModel> edgeFree = await _edgeRepository.GetFreeEdgesIdsAsync(edgesToCheck);
                if (!edgeFree.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest,
                        "There are no edges connected to the Robot"));
                }

                var response = edgeFree.ToEdgesResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        [HttpGet]
        [Route("lessBusy", Name = "GetLessBusyEdges")]
        [ProducesResponseType(typeof(GetEdgesResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetLessBusyEdgesAsync(List<EdgeModel> edgesToCheck)
        {
            try
            {
                if (!edgesToCheck.Any())
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "No Edge ids were provided"));
                }

                List<EdgeModel> lessBusyEdge = await _edgeRepository.GetLessBusyEdgesAsync(edgesToCheck);
                if (!lessBusyEdge.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.BadRequest, "There are no busy edges"));
                }

                var response = lessBusyEdge.ToEdgesResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        [HttpGet]
        [Route("name/{name}", Name = "EdgeGetDataByName")]
        [ProducesResponseType(typeof(EdgeResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetEdgeResourceDetailsByNameAsync(string name)
        {
            try
            {
                EdgeModel edgeResource = await _edgeRepository.GetEdgeResourceDetailsByNameAsync(name);
                if (edgeResource is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }

                var response = edgeResource.ToEdgeResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Check if a edge is busy by Id.
        /// </summary>
        /// <param name="edgeId"></param>
        /// <returns>bool</returns>
        [HttpGet]
        [Route("{id}/busy", Name = "IsBusyEdgeById")]
        [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<bool>> IsBusyEdgeById(Guid edgeId)
        {
            try
            {
                bool busy = await _edgeRepository.IsBusyEdgeByIdAsync(edgeId);
                return Ok(busy);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Check if a edge is busy by Name.
        /// </summary>
        /// <param name="name"></param>
        /// <returns>bool</returns>
        [HttpGet]
        [Route("{name}/busy", Name = "IsBusyEdgeByName")]
        [ProducesResponseType(typeof(bool), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> IsBusyEdgeByName(string name)
        {
            try
            {
                bool busy = await _edgeRepository.IsBusyEdgeByNameAsync(name);
                return Ok(busy);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        ///  Returns the number of containers that are deployed in a cloud entity base on cloud Id. 
        /// </summary>
        /// <param name="edgeId"></param>
        /// <returns>int</returns>
        [HttpGet]
        [Route("{id}/containers/count", Name = "GetNumEdgeContainersById")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetNumEdgeContainersById(Guid edgeId)
        {
            try
            {
                int countContainers = await _edgeRepository.GetNumContainersByIdAsync(edgeId);
                return Ok(countContainers);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        ///  Returns the number of containers that are deployed in a cloud entity base on cloud Name. 
        /// </summary>
        /// <param name="edgeName"></param>
        /// <returns>int</returns>
        [HttpGet]
        [Route("{name}/containers/count", Name = "GetNumEdgeContainersByName")]
        [ProducesResponseType(typeof(int), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<int>> GetNumEdgeContainersByName(string edgeName)
        {
            try
            {
                int countContainers = await _edgeRepository.GetNumContainersByNameAsync(edgeName);
                return Ok(countContainers);
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