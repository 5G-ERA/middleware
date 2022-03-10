using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ActionController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionModel> GetActionByIdAsync(Guid id)
        {
            ActionModel action = new ActionModel();
            return action;
        }

        [HttpPost]
        [ProducesResponseType(typeof(ActionModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ActionModel>> PostActionAsync([FromBody] ActionModel actionModel)
        {
            ActionModel action = new ActionModel();
            return Ok(action);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteActionAsync(Guid id)
        {
            //Delete Action by id
            return Ok();
        }
    }
}
