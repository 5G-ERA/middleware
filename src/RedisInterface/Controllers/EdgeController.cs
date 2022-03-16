using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class EdgeController : ControllerBase
    {
        private readonly IEdgeRepository _edgeRepository;

        public EdgeController(IEdgeRepository edgeRepository)
        {
            _edgeRepository = edgeRepository ?? throw new ArgumentNullException(nameof(edgeRepository));
        }


        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            EdgeModel model = await _edgeRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpGet]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<EdgeModel>>> GetAllAsync()
        {
            List<EdgeModel> models = await _edgeRepository.GetAllAsync();
            return Ok(models);
        }
    }
}
