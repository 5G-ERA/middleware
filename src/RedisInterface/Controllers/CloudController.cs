using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class CloudController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<CloudModel> GetCloudByIdAsync(Guid id)
        {
            CloudModel cloud = new CloudModel();
            return cloud;
        }


        [HttpGet]
        [ProducesResponseType(typeof(CloudModel), (int)HttpStatusCode.OK)]
        public async Task<List<CloudModel>> GetAllCloudAsync()
        {
            List<CloudModel> clouds = new List<CloudModel>();
            return clouds;
        }
    }
}
