using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class ContainerImageController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ContainerImageModel> GetContainerImageByIdAsync(Guid id)
        {
            ContainerImageModel containerImage = new ContainerImageModel();
            return containerImage;
        }

        [HttpPost]
        [ProducesResponseType(typeof(ContainerImageModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ContainerImageModel>> PostContainerImageAsync([FromBody] ContainerImageModel containerImageModel)
        {
            ContainerImageModel containerImage = new ContainerImageModel();
            return Ok(containerImage);
        }

        [HttpDelete]
        [Route("{id}")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteContainerImageAsync(Guid id)
        {
            //Delete ContainerImage by id
            return Ok();
        }
    }
}
