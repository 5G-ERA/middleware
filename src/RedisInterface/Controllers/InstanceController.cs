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

        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        { 
            InstanceModel instanceModel = await _instanceRepository.GetByIdAsync(id);

            return Ok(instanceModel);  
        }


        [HttpPost] 
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<InstanceModel>> AddAsync([FromBody] InstanceModel instanceModel)
        {
            await _instanceRepository.AddAsync(instanceModel);
            return Ok(instanceModel);
        }

        [HttpPatch]
        [Route("{id}")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchInstanceAsync([FromBody] JsonPatchDocument instanceModel, [FromRoute] Guid id) 
        {
            InstanceModel instance = new InstanceModel();
            return Ok(instance);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _instanceRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
