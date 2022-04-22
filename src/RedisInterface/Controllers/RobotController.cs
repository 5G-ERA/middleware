using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class RobotController : ControllerBase
    {
        private readonly IRobotRepository _robotRepository;
        private readonly ILogger _logger;

        public RobotController(IRobotRepository robotRepository, ILogger<RobotController> logger)
        {
            _robotRepository = robotRepository ?? throw new ArgumentNullException(nameof(robotRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }

        /// <summary>
        /// Get all the RobotModel entities
        /// </summary>
        /// <returns> the list of RobotModel entities </returns>
        [HttpGet(Name = "RobotGetAll")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<RobotModel>>> GetAllAsync()
        {
            try
            {
                List<RobotModel> models = await _robotRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound("Objects were not found.");
                }
                return Ok(models);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Get a RobotModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the RobotModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "RobotGetById")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                RobotModel model = await _robotRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound("Object was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Add a new RobotModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created RobotModel entity </returns>
        [HttpPost(Name = "RobotAdd")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RobotModel>> AddAsync([FromBody] RobotModel model)
        {
            if (model == null)
            {
                BadRequest("Parameters were not specified.");
            }
            try
            {
                await _robotRepository.AddAsync(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex.Message);
                return Problem("Something went wrong while calling the API");
            }
            return Ok(model);
        }

        /// <summary>
        /// Partially update an existing InstanceModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "RobotPatch")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchRobotAsync([FromBody] RobotModel patch, [FromRoute] Guid id)
        {
            try
            {
                RobotModel model = await _robotRepository.PatchRobotAsync(id, patch);
                if (model == null)
                {
                    return NotFound("Object to be updated was not found.");
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }

        /// <summary>
        /// Delete an RobotModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "RobotDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                await _robotRepository.DeleteByIdAsync(id);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
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
        [Route("relation/{name}", Name ="RobotGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _robotRepository.GetRelation(id, name);
                if (!relations.Any())
                {
                    return NotFound("Relations were not found.");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }


        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "RobotGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _robotRepository.GetRelations(id, relationNames);
                if (!relations.Any())
                {
                    return NotFound("Relations were not found");
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "An error occurred:");
                return Problem(ex.Message);
            }
        }
    }
}
