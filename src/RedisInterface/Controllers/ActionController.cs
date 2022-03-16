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

        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            ActionModel actionModel = await _actionRepository.GetByIdAsync(id);

            return Ok(actionModel);
        }


        [HttpPost]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ActionModel>> AddAsync([FromBody] ActionModel actionModel)
        {
            await _actionRepository.AddAsync(actionModel);
            return Ok(actionModel);
        }

        [HttpPatch]
        [Route("{id}")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchInstanceAsync([FromBody] JsonPatchDocument actionModel, [FromRoute] Guid id)
        {
            ActionModel action = new ActionModel();
            return Ok(action);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _actionRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
