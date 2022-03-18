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


        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            ContainerImageModel containerImageModel = await _containerImageRepository.GetByIdAsync(id);

            return Ok(containerImageModel);
        }


        [HttpPost(Name = "ContainerImageAdd")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ContainerImageModel>> AddAsync([FromBody] ContainerImageModel containerImageModel)
        {
            await _containerImageRepository.AddAsync(containerImageModel);
            return Ok(containerImageModel);
        }

        [HttpPatch]
        [Route("{id}")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> PatchContainerImageAsync([FromBody] JsonPatchDocument containerImageModel, [FromRoute] Guid id)
        {
            ContainerImageModel containerImage = new ContainerImageModel();
            return Ok(containerImage);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _containerImageRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
