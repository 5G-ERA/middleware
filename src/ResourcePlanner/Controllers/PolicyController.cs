using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class PolicyController : Controller
    {
        public record PolicyTrimmedRecord(Guid Id, string PolicyName, string PolicyDescription);

        [HttpGet] // GET /POLICY/Current
        [Route("current")]
        [ProducesResponseType(typeof(PolicyTrimmedRecord), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<PolicyTrimmedRecord>> GetCurrentPolicy()
        {
            var policy = new PolicyModel();
            var policyRecord = new PolicyTrimmedRecord(policy.Id, policy.Name, policy.Description);
            return Ok(policyRecord);
        }
        [HttpGet]
        [Route("all")]
        [ProducesResponseType(typeof(List<PolicyModel>), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<List<PolicyModel>>> GetAllPolicies()
        {
            var list = new List<PolicyModel>() { new PolicyModel()};
            return Ok(list);
        }
        
    }
}
