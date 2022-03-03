using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        [HttpGet]
        [Route("[action]", Name = "GetResource")]
        [ProducesResponseType(typeof(ResourceModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ResourceModel>> GetResource()
        {
            var resource = new ResourceModel();
            return Ok(resource);
        }
    }
}
