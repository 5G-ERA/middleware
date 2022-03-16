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

        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            CloudModel model = await _cloudRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpGet]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<CloudModel>>> GetAllAsync()
        {
            List<CloudModel> models = await _cloudRepository.GetAllAsync();

            return Ok(models);
        }
    }
}
