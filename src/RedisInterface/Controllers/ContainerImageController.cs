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
    public class ContainerImageController : ControllerBase
    {
        private readonly IContainerImageRepository _containerImageRepository;

        public ContainerImageController(IContainerImageRepository containerImageRepository)
        {
            _containerImageRepository = containerImageRepository ?? throw new ArgumentNullException(nameof(containerImageRepository));
        }


        [HttpGet(Name = "ContainerImageGetAll")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<ContainerImageModel>>> GetAllAsync()
        {
            List<ContainerImageModel> models = await _containerImageRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "ContainerImageGetbyId")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            ContainerImageModel model = await _containerImageRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "ContainerImageAdd")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ContainerImageModel>> AddAsync([FromBody] ContainerImageModel model)
        {
            await _containerImageRepository.AddAsync(model);
            return Ok(model);
        }

        [HttpPatch]
        [Route("{id}", Name = "ContainerImagePatch")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchContainerImageAsync([FromBody] ContainerImageModel patch, [FromRoute] Guid id)
        {
            ContainerImageModel model = await _containerImageRepository.PatchContainerImageAsync(id, patch);
            return Ok(model);
        }



        [HttpDelete]
        [Route("{id}", Name = "ContainerImageDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _containerImageRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
