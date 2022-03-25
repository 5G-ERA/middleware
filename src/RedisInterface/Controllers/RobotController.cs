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

        /// <summary>
        /// Get all the RobotModel entities
        /// </summary>
        /// <returns> the list of RobotModel entities </returns>
        [HttpGet(Name = "RobotGetAll")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<IEnumerable<RobotModel>>> GetAllAsync()
        {
            List<RobotModel> models = await _robotRepository.GetAllAsync();

            return Ok(models);
        }

        /// <summary>
        /// Get a RobotModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the RobotModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "RobotGetById")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            RobotModel model = await _robotRepository.GetByIdAsync(id);

            return Ok(model);
        }

        /// <summary>
        /// Add a new RobotModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created RobotModel entity </returns>
        [HttpPost(Name = "RobotAdd")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<RobotModel>> AddAsync([FromBody] RobotModel model)
        {
            await _robotRepository.AddAsync(model);
            return Ok(model);
        }

        /// <summary>
        /// Delete an RobotModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "RobotDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            await _robotRepository.DeleteByIdAsync(id);
            return Ok();
        }

        //[HttpGet]
        //[ProducesResponseType(typeof(List <TaskModel>), (int)HttpStatusCode.OK)]
        //public async Task<ActionResult<IEnumerable<TaskModel>>> GetTaskForRobotByIdAsync()
        //{
        //  //  List<TaskModel> models = await _robotRepository.GetAllAsync();
        //    return Ok();
        //}

        [HttpGet]
        [Route("relation/{name}", Name ="GetRobotRelationByName")]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            var relations =await _robotRepository.GetRelation(id, name);
            return Ok(relations);
        }
    }
}
