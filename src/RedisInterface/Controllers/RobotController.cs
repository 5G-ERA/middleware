using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class RobotController : ControllerBase
    {
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<RobotModel> GetRobotByIdAsync(Guid id)
        {
            RobotModel robot = new RobotModel();
            return robot;
        }


        [HttpGet]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<List<RobotModel>> GetAllRobotAsync()
        {
            List<RobotModel> robots = new List<RobotModel>();
            return robots;
        }
    }
}
