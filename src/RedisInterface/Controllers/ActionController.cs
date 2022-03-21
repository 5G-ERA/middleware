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


        [HttpGet(Name = "ActionGetAll")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<ActionModel>>> GetAllAsync()
        {
            List<ActionModel> models = await _actionRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "ActionGetById")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            ActionModel model = await _actionRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "ActionAdd")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ActionModel>> AddAsync([FromBody] ActionModel model)
        {
            await _actionRepository.AddAsync(model);
            return Ok(model);
        }


        [HttpPatch]
        [Route("{id}", Name ="ActionPatch")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchActionAsync([FromBody] ActionModel patch, [FromRoute] Guid id)
        {
            ActionModel model = await _actionRepository.PatchActionAsync(id, patch);
            return Ok(model);
        }

        

        [HttpDelete]
        [Route("{id}", Name ="ActionDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _actionRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
