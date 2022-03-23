using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.RedisInterface.Repositories;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class PolicyController : ControllerBase
    {
        private readonly IPolicyRepository _policyRepository;

        public PolicyController(IPolicyRepository policyRepository)
        {
            _policyRepository = policyRepository;
        }

        /// <summary>
        /// Get a PolicyModel entity by id
        /// </summary>
        /// <param name="id"></param>
        /// <returns> the PolicyModel entity for the specified id </returns>
        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<PolicyModel>> GetPolicyByIdAsync(Guid id)
        {
            PolicyModel policy = new PolicyModel();
            return Ok(policy);
        }

        /// <summary>
        /// Get all the PolicyModel entities
        /// </summary>
        /// <returns> the list of PolicyModel entities </returns>
        [HttpGet]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<List<PolicyModel>>> GetAllPoliciesAsync()
        {
            List<PolicyModel> policies = await _policyRepository.GetAllPoliciesAsync();
            return Ok(policies);
        }

        /// <summary>
        /// Represents the single currently active policy
        /// </summary>
        /// <param name="Id"></param>
        /// <param name="PolicyName"></param>
        /// <param name="PolicyDescription"></param>
        public record ActivePolicy(Guid Id, string PolicyName, string PolicyDescription);

        [HttpGet]
        [Route("current")]
        [ProducesResponseType(typeof(ActivePolicy), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<List<ActivePolicy>>> GetActivePolicies()
        {
            List<PolicyModel> activePolicies = await _policyRepository.GetActivePoliciesAsync();

            List<ActivePolicy> activePoliciesRecords = activePolicies.Select(p => new ActivePolicy(p.Id, p.PolicyName, p.Description)).ToList();

            return Ok(activePoliciesRecords);
        }
    }
}
