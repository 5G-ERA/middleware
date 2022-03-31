using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.JsonPatch;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class InstanceController : ControllerBase
    {
        private readonly IInstanceRepository _instanceRepository;

        public InstanceController(IInstanceRepository instanceRepository)
        {
           _instanceRepository = instanceRepository;
        }

        /// <summary>
        /// Get all the InstanceModel entities
        /// </summary>
        /// <returns> the list of InstanceModel entities </returns>
        [HttpGet(Name = "InstanceGetAll")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<InstanceModel>>> GetAllAsync()
        {
            List<InstanceModel> models = await _instanceRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get an InstanceModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the InstanceModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "InstanceGetById")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        { 
            InstanceModel model = await _instanceRepository.GetByIdAsync(id);

            return Ok(model);  
        }

        /// <summary>
        /// Add a new InstanceModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created InstanceModel entity </returns>
        [HttpPost(Name = "InstanceAdd")] 
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<InstanceModel>> AddAsync([FromBody] InstanceModel model)
        {
            await _instanceRepository.AddAsync(model);
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
        public async Task<IActionResult> PatchInstanceAsync([FromBody] InstanceModel patch, [FromRoute] Guid id) 
        {

            InstanceModel model = await _instanceRepository.PatchInstanceAsync(id, patch) ;
            return Ok(model);
        }

        /// <summary>
        /// Delete an InstanceModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "InstanceDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _instanceRepository.DeleteByIdAsync(id);
            return Ok();
        }

        [HttpGet]
        [Route("relation/{name}", Name = "InstanceGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations = await _instanceRepository.GetRelation(id, name);
            return Ok(relations);
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "InstanceGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            List<string> relationNames = new List<string>() { firstName, secondName };
            var relations = await _instanceRepository.GetRelations(id, relationNames);
            return Ok(relations);
        }
    }
}
