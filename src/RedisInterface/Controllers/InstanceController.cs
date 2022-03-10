using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class InstanceController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<InstanceModel> GetInstanceByIdAsync(Guid id)
        {
            InstanceModel instance = new InstanceModel();
            return instance;
        }

        [HttpPost] 
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<InstanceModel>> PostInstanceAsync([FromBody] InstanceModel instanceModel)
        {
            InstanceModel instance = new InstanceModel();
            return Ok(instance);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteInstance(Guid id)
        {
            //Delete Instance by id
            return Ok();
        }
    }
}
