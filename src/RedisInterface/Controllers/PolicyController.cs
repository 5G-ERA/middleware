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

        [HttpGet]
        [Route("{id}")]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        public async Task<PolicyModel> GetPolicyByIdAsync(Guid id)
        {
            PolicyModel policy = new PolicyModel();
            return policy;
        }

        [HttpGet]
        [ProducesResponseType(typeof(PolicyModel), (int)HttpStatusCode.OK)]
        public async Task<List<PolicyModel>> GetAllPolicyAsync()
        {
            List<PolicyModel> policies = await _policyRepository.GetAllPoliciesAsync();
            return policies;
        }
    }
}
