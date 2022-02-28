using Microsoft.AspNetCore.Mvc;
using System.Net;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourcesInstanceController : ControllerBase
    {
        [HttpGet]
        [Route("[action]", Name = "GetResourcesInstance")]
        [ProducesResponseType(typeof(ResourcesInstance), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<ResourcesInstance>> GetResourcesInstance()
        {
            var resourcesInstance = new ResourcesInstance();
            return Ok(resourcesInstance);
        }
    }
}
