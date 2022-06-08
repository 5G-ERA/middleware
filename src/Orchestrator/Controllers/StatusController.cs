using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories;

namespace Middleware.Orchestrator.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class StatusController : Controller
    {
        private readonly INetAppStatusRepository _netAppStatusRepository;
        private readonly IRobotStatusRepository _robotStatusRepository;
        private readonly ILogger _logger;

        public StatusController(INetAppStatusRepository netAppStatusRepository, IRobotStatusRepository robotStatusRepository, ILogger<StatusController> logger)
        {
            _netAppStatusRepository = netAppStatusRepository;
            _robotStatusRepository = robotStatusRepository;
            _logger = logger;
        }
        /// <summary>
        /// Get the robot status
        /// </summary>
        /// <returns> the list of RobotStatusModel entities </returns>
        [HttpGet]
        [Route("robot")]
        [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RobotStatusModel>> GetRobotStatusByIdAsync(Guid id)
        {
            if (id == Guid.Empty)
            {
                return BadRequest(new ApiResponse((int) HttpStatusCode.BadRequest,
                    "Identifier of the robot must be specified"));
            }
            try
            {

                var status = await _robotStatusRepository.GetByIdAsync(id);
                if (status is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Status for the specified robot was not found."));
                }
                return Ok(status);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
        /// <summary>
        /// Add a new RobotStatusModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created RobotStatusModel entity </returns>
        [HttpPost]
        [Route("robot")]
        [ProducesResponseType(typeof(RobotStatusModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<RobotStatusModel>> AddRobotStatusAsync([FromBody] RobotStatusModel model)
        {
            if (model.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                await _robotStatusRepository.AddAsync(model, () => model.Id);
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
        /// Get the robot status
        /// </summary>
        /// <returns> the list of RobotStatusModel entities </returns>
        [HttpGet]
        [Route("netapp")]
        [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<NetAppStatusModel>> GetNetAppStatusByIdAsync(Guid id)
        {
            if (id == Guid.Empty)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                    "Identifier of the robot must be specified"));
            }
            try
            {
                var status = await _netAppStatusRepository.GetByIdAsync(id);
                if (status is null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Status for the specified robot was not found."));
                }
                return Ok(status);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }
        /// <summary>
        /// Add a new RobotStatusModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created RobotStatusModel entity </returns>
        [HttpPost]
        [Route("netapp")]
        [ProducesResponseType(typeof(NetAppStatusModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<NetAppStatusModel>> AddNetAppStatusAsync([FromBody] NetAppStatusModel model)
        {
            if (model.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                await _netAppStatusRepository.AddAsync(model, () => model.Id);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
            return Ok(model);
        }
    }
}
