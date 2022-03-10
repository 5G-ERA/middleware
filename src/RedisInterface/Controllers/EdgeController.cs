using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class EdgeController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<EdgeModel> GetEdgeByIdAsync(Guid id)
        {
            EdgeModel edge = new EdgeModel();
            return edge;
        }


        [HttpGet]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<List<EdgeModel>> GetAllEdgeAsync()
        {
            List<EdgeModel> edges = new List<EdgeModel>();
            return edges;
        }
    }
}
