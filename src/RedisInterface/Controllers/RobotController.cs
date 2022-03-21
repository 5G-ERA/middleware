﻿using Microsoft.AspNetCore.Http;
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

        [HttpGet(Name = "RobotGetAll")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<RobotModel>>> GetAllAsync()
        {
            List<RobotModel> models = await _robotRepository.GetAllAsync();

            return Ok(models);
        }


        [HttpGet]
        [Route("{id}", Name = "RobotGetById")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            RobotModel model = await _robotRepository.GetByIdAsync(id);

            return Ok(model);
        }


        [HttpPost(Name = "RobotAdd")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<RobotModel>> AddAsync([FromBody] RobotModel model)
        {
            await _robotRepository.AddAsync(model);
            return Ok(model);
        }


        [HttpDelete]
        [Route("{id}", Name = "RobotDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _robotRepository.DeleteByIdAsync(id);
            return Ok();
        }
    }
}
