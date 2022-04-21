﻿using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using System.Net;

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
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<EdgeModel>>> GetAllAsync()
        {
            try
            {
                List<EdgeModel> models = await _edgeRepository.GetAllAsync();
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
        /// Get an EdgeModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the EdgeModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "EdgeGetById")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                EdgeModel model = await _edgeRepository.GetByIdAsync(id);
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
        /// Add a new EdgeModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created EdgeModel entity </returns>
        [HttpPost(Name = "EdgeAdd")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<EdgeModel>> AddAsync([FromBody] EdgeModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _edgeRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
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
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchEdgeAsync([FromBody] EdgeModel patch, [FromRoute] Guid id)
        {

            try
            {
                EdgeModel model = await _edgeRepository.PatchEdgeAsync(id, patch);
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
        /// Delete an EdgeModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "EdgeDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _edgeRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();
        }

        [HttpGet]
        [Route("relation/{name}", Name = "EdgeGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _edgeRepository.GetRelation(id, name);
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
        [Route("relations/{firstName}/{secondName}", Name = "EdgeGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _edgeRepository.GetRelations(id, relationNames);
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
