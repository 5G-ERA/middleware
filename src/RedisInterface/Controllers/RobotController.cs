using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class RobotController : ControllerBase
    {
        private readonly IRobotRepository _robotRepository;

        public RobotController(IRobotRepository robotRepository)
        {
            _robotRepository = robotRepository ?? throw new ArgumentNullException(nameof(robotRepository));
        }

        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            RobotModel robotModel = await _robotRepository.GetByIdAsync(id);

            return Ok(robotModel);
        }

        [HttpGet]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<RobotModel>>> GetAllAsync() 
        {
            List<RobotModel> models = await _robotRepository.GetAllAsync();
            return Ok(models);
        }
    }
}
