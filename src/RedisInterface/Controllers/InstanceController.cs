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

        [HttpGet(Name = "InstanceGetAll")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<InstanceModel>>> GetAllAsync()
        {
            List<InstanceModel> models = await _instanceRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "InstanceGetById")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        { 
            InstanceModel model = await _instanceRepository.GetByIdAsync(id);

            return Ok(model);  
        }


        [HttpPost(Name = "InstanceAdd")] 
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<InstanceModel>> AddAsync([FromBody] InstanceModel model)
        {
            await _instanceRepository.AddAsync(model);
            return Ok(model);
        }

        [HttpPatch]
        [Route("{id}", Name = "InstancePatch")]
        [ProducesResponseType(typeof(InstanceModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchInstanceAsync([FromBody] InstanceModel patch, [FromRoute] Guid id) 
        {

            InstanceModel model = await _instanceRepository.PatchInstanceAsync(id, patch) ;
            return Ok(model);
        }

        [HttpDelete]
        [Route("{id}", Name = "InstanceDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _instanceRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
