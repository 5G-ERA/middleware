using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class PolicyController : ControllerBase
    {
        private readonly IPolicyRepository _policyRepository;
        private readonly ILogger _logger;

        public PolicyController(IPolicyRepository policyRepository, ILogger<PolicyController> logger)
        {
            _policyRepository = policyRepository ?? throw new ArgumentNullException(nameof(policyRepository));
            _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        }


        /// <summary>
        /// Add a new PolicyModel entity
        /// </summary>
        /// <param name="model"></param>
        /// <returns> the newly created PolicyModel entity </returns>
        [HttpPost(Name = "PolicyAdd")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<PolicyModel>> PolicyAdd ([FromBody] PolicyModel model)
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
                await _policyRepository.AddAsync(model);
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
        /// Get a PolicyModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the PolicyModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "PolicyGetById")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<PolicyModel>> GetPolicyByIdAsync(Guid id)
        {
            try
            {
                PolicyModel model = await _policyRepository.GetByIdAsync(id);
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
        /// Get all the PolicyModel entities
        /// </summary>
        /// <returns> the list of PolicyModel entities </returns>
        [HttpGet(Name = "PolicyGetAll")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<PolicyModel>>> GetAllPoliciesAsync()
        {
            try
            {
                List<PolicyModel> models = await _policyRepository.GetAllPoliciesAsync();
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
        /// Represents the single currently active policy
        /// </summary>
        /// <param name="Id"></param>
        /// <param name="PolicyName"></param>
        /// <param name="PolicyDescription"></param>
        public record ActivePolicy(Guid Id, string PolicyName, string PolicyDescription);

        [HttpGet]
        [Route("current", Name = "PolicyGetActive")]
        [ProducesResponseType(typeof(List<ActivePolicy>), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<List<ActivePolicy>>> GetActivePolicies()
        {
            try
            {
                List<PolicyModel> activePolicies = await _policyRepository.GetActivePoliciesAsync();
                if (activePolicies == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }
                List<ActivePolicy> activePoliciesRecords = activePolicies.Select(p => new ActivePolicy(p.Id, p.Name, p.Description)).ToList();
                return Ok(activePoliciesRecords);     
            }
            catch (Exception ex) 
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        /// Partially update an existing InstanceModel entity
        /// </summary>
        /// <param name="patch"></param>
        /// <param name="id"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPatch]
        [Route("{id}", Name = "PolicyPatch")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchPolicyAsync([FromBody] PolicyModel patch, [FromRoute] Guid id)
        {
            if (patch.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }
            try
            {
                PolicyModel model = await _policyRepository.PatchPolicyAsync(id, patch);
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
    }
}
