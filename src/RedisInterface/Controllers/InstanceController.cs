using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.JsonPatch;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using System.Net;

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
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<InstanceModel>>> GetAllAsync()
        {
            try
            {
                List<InstanceModel> models = await _instanceRepository.GetAllAsync();
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
        /// Get an InstanceModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the InstanceModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "InstanceGetById")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                InstanceModel model = await _instanceRepository.GetByIdAsync(id);
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
        /// Add a new InstanceModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created InstanceModel entity </returns>
        [HttpPost(Name = "InstanceAdd")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<InstanceModel>> AddAsync([FromBody] InstanceModel model)
        {
            if (model == null)
            {
                return BadRequest("Parameters were not specified.");
            }
            try 
            { 
                await _instanceRepository.AddAsync(model); 
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
        [Route("{id}", Name = "InstancePatch")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchInstanceAsync([FromBody] InstanceModel patch, [FromRoute] Guid id)
        {
            try
            {
                InstanceModel model = await _instanceRepository.PatchInstanceAsync(id, patch);
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
        /// Delete an InstanceModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "InstanceDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _instanceRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex) 
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();
        }


        [HttpPost]
        [Route("AddRelation", Name = "InstanceAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest("Parameters were not specified.");
            }
            try
            {
                bool isValid = await _instanceRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return Problem("The relation was not created");
                }
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }


        [HttpGet]
        [Route("relation/{name}", Name = "InstanceGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _instanceRepository.GetRelation(id, name);
                if (!relations.Any()) 
                {
                    return NotFound("Relations were not found.");
                }
                return Ok(relations);
            }
            catch(Exception ex) 
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "InstanceGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _instanceRepository.GetRelations(id, relationNames);
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
