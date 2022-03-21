using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class CloudController : ControllerBase
    {
        private readonly ICloudRepository _cloudRepository;

        public CloudController(ICloudRepository cloudRepository)
        {
            _cloudRepository = cloudRepository ?? throw new ArgumentNullException(nameof(cloudRepository));
        }

        [HttpGet(Name = "CloudGetAll")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<CloudModel>>> GetAllAsync()
        {
            List<CloudModel> models = await _cloudRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "CloudGetById")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            CloudModel model = await _cloudRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "CloudAdd")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<CloudModel>> AddAsync([FromBody] CloudModel model)
        {
            await _cloudRepository.AddAsync(model);
            return Ok(model);
        }


        [HttpDelete]
        [Route("{id}", Name = "CloudDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _cloudRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
