using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using System.Net;

namespace Middleware.RedisInterface.Controllers
{
    [Route("api/v1/[controller]")]
    [ApiController]
    public class PolicyController : ControllerBase
    {
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
            List<PolicyModel> policies = new List<PolicyModel>();
            return policies;
        }
    }
}
