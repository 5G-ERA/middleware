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

        /// <summary>
        /// Get all the CloudModel entities
        /// </summary>
        /// <returns> the list of CloudModel entities </returns>
        [HttpGet(Name = "CloudGetAll")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<CloudModel>>> GetAllAsync()
        {
            List<CloudModel> models = await _cloudRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get a CloudModel entity by id 
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the CloudModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "CloudGetById")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            CloudModel model = await _cloudRepository.GetByIdAsync(id);

            return Ok(model);
        }


        /// <summary>
        /// Add a new CloudModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created CloudModel entity </returns>
        [HttpPost(Name = "CloudAdd")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<CloudModel>> AddAsync([FromBody] CloudModel model)
        {
            await _cloudRepository.AddAsync(model);
            return Ok(model);
        }

        /// <summary>
        /// Partially update an existing CloudModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified CloudModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "CloudPatch")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchCloudAsync([FromBody] CloudModel patch, [FromRoute] Guid id)
        {

            CloudModel model = await _cloudRepository.PatchCloudAsync(id, patch);
            return Ok(model);
        }

        /// <summary>
        /// Delete an CloudModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
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
