using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ContainerImageController : ControllerBase
    {
        private readonly IContainerImageRepository _containerImageRepository;
        private readonly ILogger _logger;

        public ContainerImageController(IContainerImageRepository containerImageRepository, ILogger<ActionController> logger)
        {
            _containerImageRepository = containerImageRepository ?? throw new ArgumentNullException(nameof(containerImageRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the ContainerImageModel entities
        /// </summary>
        /// <returns> the list of ContainerImageModel entities </returns>
        [HttpGet(Name = "ContainerImageGetAll")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<ContainerImageModel>>> GetAllAsync()
        {
            try
            {
                List<ContainerImageModel> models = await _containerImageRepository.GetAllAsync();
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
        /// Get an ContainerImageModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the ContainerImageModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "ContainerImageGetbyId")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                ContainerImageModel model = await _containerImageRepository.GetByIdAsync(id);
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
        /// Add a new ContainerImageModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created ContainerImageModel entity </returns>
        [HttpPost(Name = "ContainerImageAdd")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<ContainerImageModel>> AddAsync([FromBody] ContainerImageModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _containerImageRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }

        /// <summary>
        /// Partially update an existing ContainerImageModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified ContainerImageModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "ContainerImagePatch")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchContainerImageAsync([FromBody] ContainerImageModel patch, [FromRoute] Guid id)
        {
            try
            {
                ContainerImageModel model = await _containerImageRepository.PatchContainerImageAsync(id, patch);
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
        /// Delete a ContainerImageModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "ContainerImageDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _containerImageRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
            return Ok();
        }

        [HttpPost]
        [Route("AddRelation", Name = "ContainerImageAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _containerImageRepository.AddRelationAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }


        [HttpGet]
        [Route("relation/{name}", Name = "ContainerImageGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _containerImageRepository.GetRelation(id, name);
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
        [Route("relations/{firstName}/{secondName}", Name = "ContainerImageGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _containerImageRepository.GetRelations(id, relationNames);
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

        /// <summary>
        /// Gets the images associated with the specified instance
        /// </summary>
        /// <param name="id">Identifier of the instance</param>
        /// <returns></returns>
        [HttpGet]
        [Route("instance/{id}", Name = "ContainerImageGetForInstance")]
        [ProducesResponseType(typeof(List<ContainerImageModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType((int)HttpStatusCode.NotFound)]
        public async Task<IActionResult> GetImagesForInstanceAsync(Guid id)
        {
            try
            {
                var images = await _containerImageRepository.GetImagesForInstanceAsync(id);
                if (images.Any() == false)
                {
                    return NotFound();
                }
                return Ok(images);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An unexpected error has happened: ");
                return Problem($"An unexpected error has occurred: {ex.Message}");
            }
        }
    }
}
