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


        [HttpGet(Name = "EdgeGetAll")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<EdgeModel>>> GetAllAsync()
        {
            List<EdgeModel> models = await _edgeRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "EdgeGetById")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            EdgeModel model = await _edgeRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "EdgeAdd")]
        [ProducesResponseType(typeof(EdgeModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<EdgeModel>> AddAsync([FromBody] EdgeModel model)
        {
            await _edgeRepository.AddAsync(model);
            return Ok(model);
        }


        [HttpDelete]
        [Route("{id}", Name = "EdgeDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _edgeRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
