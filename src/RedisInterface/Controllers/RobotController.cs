using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using System.Net;
using Middleware.Models.Domain;

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
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<IEnumerable<RobotModel>>> GetAllAsync()
        {
            try
            {
                List<RobotModel> models = await _robotRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));
                }
                return Ok(models);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetByIdAsync(Guid id)
        {
            try
            {
                RobotModel model = await _robotRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Add a new RobotModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created RobotModel entity </returns>
        [HttpPost(Name = "RobotAdd")]
        [ProducesResponseType(typeof(RobotModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RobotModel>> AddAsync([FromBody] RobotModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            if (model.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }
            try
            {
                RobotModel robot = await _robotRepository.AddAsync(model);
                if (robot is null)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError,
                            "Could not add the robot to the data store"));
                }
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
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
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchRobotAsync([FromBody] RobotModel patch, [FromRoute] Guid id)
        {
            if (patch.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }
            try
            {
                RobotModel model = await _robotRepository.PatchRobotAsync(id, patch);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
                }
                return Ok(model);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Delete a RobotModel entity for the given id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> no return </returns>
        [HttpDelete]
        [Route("{id}", Name = "RobotDelete")]
        [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> DeleteByIdAsync(Guid id)
        {
            try
            {
                var deleted = await _robotRepository.DeleteByIdAsync(id);
                if (deleted == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "The specified Robot has not been found."));
                }
                return Ok();

            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Creates a new relation between two models
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        [HttpPost]
        [Route("AddRelation", Name = "RobotAddRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> AddRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                bool isValid = await _robotRepository.AddRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                }

            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok(model);
        }

        /// <summary>
        /// Deletes a new relation between two models
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        [HttpDelete]
        [Route("DeleteRelation", Name = "RobotDeleteRelation")]
        [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RelationModel>> DeleteRelationAsync([FromBody] RelationModel model)
        {
            if (model == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                bool isValid = await _robotRepository.DeleteRelationAsync(model);
                if (!isValid)
                {
                    return StatusCode((int)HttpStatusCode.InternalServerError,
                        new ApiResponse((int)HttpStatusCode.InternalServerError, "The relation was not created"));
                }

            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok(model);
        }


        /// <summary>
        /// Retrieves a single relation by name
        /// </summary>
        /// <param name="id"></param>
        /// <param name="name"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relation/{name}", Name = "RobotGetRelationByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationAsync(Guid id, string name)
        {
            try
            {
                var relations = await _robotRepository.GetRelation(id, name);
                if (!relations.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }

        /// <summary>
        /// Retrieves two relations by their names
        /// </summary>
        /// <param name="id"></param>
        /// <param name="firstName"></param>
        /// <param name="secondName"></param>
        /// <returns></returns>
        [HttpGet]
        [Route("relations/{firstName}/{secondName}", Name = "RobotGetRelationsByName")]
        [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetRelationsAsync(Guid id, string firstName, string secondName)
        {
            try
            {
                List<string> relationNames = new List<string>() { firstName, secondName };
                var relations = await _robotRepository.GetRelations(id, relationNames);
                if (!relations.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found"));
                }
                return Ok(relations);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        [HttpGet]
        [Route("{robotId}/edges/connected", Name = "RobotGetConnectedEdgesIds")]
        [ProducesResponseType(typeof(List<EdgeModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<EdgeModel>>> GetConnectedEdgesIdsAsync(Guid robotId)
        {
            try
            {
                if (robotId == Guid.Empty)
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "The Robot Id was not provided"));
                }
                List<EdgeModel> edgeIds = await _robotRepository.GetConnectedEdgesIdsAsync(robotId);
                if (!edgeIds.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No connected Edges have been found."));
                }
                return Ok(edgeIds);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }

        /// <summary>
        ///  Return the connected clouds to the robot
        /// </summary>
        /// <param name="robotId"></param>
        /// <returns>returns a list of cloudModels that have conectivity to the robot</returns>
        [HttpGet]
        [Route("{robotId}/clouds/connected", Name = "RobotGetConnectedCloudsIds")]
        [ProducesResponseType(typeof(List<CloudModel>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<CloudModel>>> GetConnectedCloudsIdsAsync(Guid robotId)
        {
            try
            {
                if (robotId == Guid.Empty)
                {
                    return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "The Robot Id was not provided"));
                }
                List<CloudModel> cloudsIds = await _robotRepository.GetConnectedCloudsIdsAsync(robotId);
                if (!cloudsIds.Any())
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "No connected clouds have been found."));
                }
                return Ok(cloudsIds);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }

        }
        [HttpGet]
        [Route("{robotId}/topic", Name = "RobotChangeTopicStatus")]
        [ProducesResponseType(typeof(RosTopicModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult> EnableTopicAsync(Guid robotId,string topicName, bool topicEnabled)
        {
            try
            {
                RobotModel robot = await _robotRepository.GetByIdAsync(robotId);
                RosTopicModel topicModel = robot.GetTopicModelFromRobot(topicName);
                if (topicEnabled == true)
                {
                    topicModel.Enable();
                }
                else
                {
                    topicModel.Disable();
                }
                await _robotRepository.AddAsync(robot, () => robot.Id);
                return Ok(topicModel);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
    }
}

