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

        public ActionController(IActionRepository actionRepository)
        {
            _actionRepository = actionRepository ?? throw new ArgumentNullException(nameof(actionRepository));
        }

        /// <summary>
        /// Get all the ActionModel entities
        /// </summary>
        /// <returns> the list of ActionModel entities </returns>
        [HttpGet(Name = "ActionGetAll")]
        [ProducesResponseType(typeof(List<ActionModel>), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<ActionModel>>> GetAllAsync()
        {
            List<ActionModel> models = await _actionRepository.GetAllAsync();

            return Ok(models);
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
            ActionModel model = await _actionRepository.GetByIdAsync(id);

            return Ok(model);
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
            await _actionRepository.AddAsync(model);
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
            ActionModel model = await _actionRepository.PatchActionAsync(id, patch);
            return Ok(model);
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
            await _actionRepository.DeleteByIdAsync(id);
            return Ok();
        }


        [HttpGet]
        [Route("relation/{name}", Name = "GetActionRelationByName")]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations = await _actionRepository.GetRelation(id, name);
            return Ok(relations);
        }
    }
}
