using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class EdgeController : ControllerBase
    {
        private readonly IEdgeRepository _edgeRepository;

        public EdgeController(IEdgeRepository edgeRepository)
        {
            _edgeRepository = edgeRepository ?? throw new ArgumentNullException(nameof(edgeRepository));
        }

        /// <summary>
        /// Get all the EdgeModel entities
        /// </summary>
        /// <returns> the list of EdgeModel entities </returns>
        [HttpGet(Name = "EdgeGetAll")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<EdgeModel>>> GetAllAsync()
        {
            List<EdgeModel> models = await _edgeRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get an EdgeModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the EdgeModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "EdgeGetById")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            EdgeModel model = await _edgeRepository.GetByIdAsync(id);

            return Ok(model);
        }

        /// <summary>
        /// Add a new EdgeModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created EdgeModel entity </returns>
        [HttpPost(Name = "EdgeAdd")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<EdgeModel>> AddAsync([FromBody] EdgeModel model)
        {
            await _edgeRepository.AddAsync(model);
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
        public async Task<IActionResult> PatchEdgeAsync([FromBody] EdgeModel patch, [FromRoute] Guid id)
        {

            EdgeModel model = await _edgeRepository.PatchEdgeAsync(id, patch);
            return Ok(model);
        }


        /// <summary>
        /// Delete an EdgeModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "EdgeDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _edgeRepository.DeleteByIdAsync(id);
            return Ok();
        }

        [HttpGet]
        [Route("relation/{name}", Name = "GetEdgeRelationByName")]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations = await _edgeRepository.GetRelation(id, name);
            return Ok(relations);
        }
    }
}
