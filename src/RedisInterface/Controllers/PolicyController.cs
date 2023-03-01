using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Attributes;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;

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
        /// <param name="request"></param>
        /// <returns> the newly created PolicyModel entity </returns>
        [HttpPost(Name = "PolicyAdd")]
        [ProducesResponseType(typeof(PolicyResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PolicyAdd ([FromBody] PolicyRequest request)
        {
            if (request == null)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
            }
            try
            {
                var policy = request.ToPolicy();
                policy = await _policyRepository.AddAsync(policy);
                var response = policy.ToPolicyResponse();
                return Ok(response);
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                _logger.LogError(ex, "An error occurred:");
                return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
            }
        }


        /// <summary>
        /// Get a PolicyModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the PolicyModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}", Name = "PolicyGetById")]
        [ProducesResponseType(typeof(PolicyResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetPolicyByIdAsync(Guid id)
        {
            try
            {
                PolicyModel model = await _policyRepository.GetByIdAsync(id);
                if (model == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object was not found."));
                }

                var response = model.ToPolicyResponse();
                return Ok(response);
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
        [ProducesResponseType(typeof(GetPoliciesResponse), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> GetAllPoliciesAsync()
        {
            try
            {
                List<PolicyModel> models = await _policyRepository.GetAllAsync();
                if (models.Any() == false)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Objects were not found."));
                }

                var response = models.ToPoliciesResponse();
                return Ok(response);
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
                if (activePolicies.Any() == false)
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
        /// Partially update an existing policy entity
        /// </summary>
        /// <param name="request"></param>
        /// <returns> the modified InstanceModel entity </returns>
        [HttpPut]
        [Route("{id}", Name = "PolicyPatch")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<IActionResult> PatchPolicyAsync([FromMultiSource] UpdatePolicyRequest request)
        {
            try
            {
                var model = request.ToPolicy();
                var exists = await _policyRepository.GetByIdAsync(model.Id);
                if (exists == null)
                {
                    return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Object to be updated was not found."));
                }
                await _policyRepository.UpdateAsync(model);
                var response = model.ToPolicyResponse();
                return Ok(response);
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
