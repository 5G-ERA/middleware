using System.Net;
using JetBrains.Annotations;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Attributes;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Mappings;
using Middleware.RedisInterface.Requests;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class PolicyController : MiddlewareController
{
    private readonly ILogger _logger;
    private readonly IPolicyRepository _policyRepository;

    public PolicyController(IPolicyRepository policyRepository, ILogger<PolicyController> logger)
    {
        _policyRepository = policyRepository ?? throw new ArgumentNullException(nameof(policyRepository));
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
    }

    /// <summary>
    ///     Get a PolicyModel entity by id
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
            var model = await _policyRepository.GetByIdAsync(id);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id), $"Policy with id {id} was not found.");

            var response = model.ToPolicyResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get Policy by name
    /// </summary>
    /// <param name="name">Name of the Policy</param>
    /// <returns> the PolicyModel entity for the specified id </returns>
    [HttpGet]
    [Route("name/{name}", Name = "PolicyGetByName")]
    [ProducesResponseType(typeof(PolicyResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetPolicyByNameAsync(string name)
    {
        try
        {
            var model = await _policyRepository.GetPolicyByName(name);
            if (model == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(name), $"Policy with name {name} was not found.");

            var response = model.ToPolicyResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Get all the PolicyModel entities
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
            var models = await _policyRepository.GetAllAsync();
            if (models.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "policy", $"Policies were not found.");

            var response = models.ToPoliciesResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    [HttpGet]
    [Route("current", Name = "PolicyGetActive")]
    [ProducesResponseType(typeof(List<ActivePolicy>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetActivePolicies()
    {
        try
        {
            var activePolicies = await _policyRepository.GetActivePoliciesAsync();
            if (activePolicies.Any() == false)
                return ErrorMessageResponse(HttpStatusCode.NotFound, "policies", $"No active policies found.");
                
            var activePoliciesRecords =
                activePolicies.Select(p => new ActivePolicy(p.Id, p.Name, p.Description)).ToList();
            
            return Ok(activePoliciesRecords);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }


    /// <summary>
    ///     Partially update an existing policy entity
    /// </summary>
    /// <param name="request"></param>
    /// <returns> the modified InstanceModel entity </returns>
    [HttpPut]
    [Route("{id}", Name = "PolicyPatch")]
    [ProducesResponseType(typeof(PolicyResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    // ReSharper disable once RouteTemplates.MethodMissingRouteParameters
    public async Task<IActionResult> PatchPolicyAsync([FromMultiSource] UpdatePolicyRequest request)
    {
        try
        {
            var model = request.ToLimitedPolicy();
            var exists = await _policyRepository.GetByIdAsync(model.Id);
            if (exists == null)
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(request.Id), $"Policy with id {request.Id} was not found.");

            exists.IsActive = model.IsActive;
            exists.Priority = model.Priority;
            exists.Timestamp = DateTime.UtcNow;

            await _policyRepository.UpdateAsync(exists);
            var response = exists.ToPolicyResponse();
            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Represents the single currently active policy
    /// </summary>
    /// <param name="Id"></param>
    /// <param name="PolicyName"></param>
    /// <param name="PolicyDescription"></param>
    private record ActivePolicy([UsedImplicitly]Guid Id, string PolicyName, string PolicyDescription);
}