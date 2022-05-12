using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class OrchestrateController : Controller
{
    private readonly IDeploymentService _deploymentService;
    private readonly IMapper _mapper;
    private readonly ILogger _logger;
    private readonly RedisInterface.RedisApiClient _redisClient;

    public OrchestrateController(IDeploymentService deploymentService, IApiClientBuilder clientBuilder, IMapper mapper, ILogger<OrchestrateController> logger)
    {
        _deploymentService = deploymentService;
        _mapper = mapper;
        _logger = logger;
        _redisClient = clientBuilder.CreateRedisApiClient();
    }

    /// <summary>
    /// Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan", Name = "InstantiateNewPlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> InstantiateNewPlan([FromBody] TaskModel task)
    {
        int statusCode = (int)HttpStatusCode.BadRequest;
        if (task is null)
        {
            return BadRequest("Plan is not specified");
        }
        try
        {
            statusCode = (int)HttpStatusCode.OK;
            var result = await _deploymentService.DeployAsync(task);
            if (result == false)
            {
                statusCode = (int)HttpStatusCode.InternalServerError;
                //TODO: provide more detailed information on what went wrong with the deployment of the task
                return StatusCode(statusCode, new ApiResponse(statusCode, "There was a problem while deploying the task instance"));
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was a problem while deploying the task instance: {task}", task);
            statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode, new ApiResponse(statusCode, $"There was a problem while deploying the task instance: {ex.Message}"));
        }

        return Ok(task);
    }

    /// <summary>
    /// Get the action plan by the ActionPlanId identifier 
    /// </summary>
    /// <param name="id">Identifier of the created ActionPlan</param>
    /// <returns></returns>
    [HttpGet]
    [Route("plan/{id}", Name = "GetActionPlanByPlanId")]
    [ProducesResponseType(typeof(ActionPlanModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<ActionPlanModel>> GetActionsByPlanId(Guid id)
    {
        int statusCode = (int)HttpStatusCode.OK;
        try
        {
            RedisInterface.ActionPlanModel riActionPlan = await _redisClient.ActionPlanGetByIdAsync(id);
            if (riActionPlan is null)
            {
                statusCode = (int)HttpStatusCode.NotFound;
                return NotFound(new ApiResponse(statusCode, "Specified action plan with specified id not found"));
            }

            ActionPlanModel actionPlan = _mapper.Map<ActionPlanModel>(riActionPlan);

            //TODO: retrieve the latest info about the plan?
            return Ok(actionPlan);
        }
        catch (RedisInterface.ApiException<RedisInterface.ApiResponse> apiEx)
        {
            _logger.LogError(apiEx, "Error while querying the redis-api");
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was an error while retrieving the action plan information");
            statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode, new ApiResponse(statusCode, $"There was an error while retrieving the action plan information: {ex.Message}"));
        }
    }

    /// <summary>
    /// Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPatch]
    [Route("plan", Name = "UpdatePlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    public async Task<IActionResult> UpdatePlan([FromBody] TaskModel task)
    {
        //TODO: redeploy services for new plan
        return Ok(task);
    }

    /// <summary>
    /// Deletes the instances instantiated with the specified action 
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("action/{id}", Name = "DeleteActionById")]
    [ProducesResponseType((int)HttpStatusCode.OK)]
    [ProducesResponseType((int)HttpStatusCode.NotFound)]
    public async Task<IActionResult> DeleteActionById(Guid id)
    {
        // TODO: Delete action with specified Id
        return Ok();
    }
    /// <summary>
    /// Delete plan by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}", Name = "DeletePlanById")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]

    public async Task<ActionResult> DeletePlanById(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Id of the plan  has to be specified"));
        }
        try
        {
            RedisInterface.ActionPlanModel riActionPlan = await _redisClient.ActionPlanGetByIdAsync(id);
            if (riActionPlan is null)
            {
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, $"Could not find the Action Plan with specified id: {id}"));
            }

            ActionPlanModel actionPlan = _mapper.Map<ActionPlanModel>(riActionPlan);

            var isSuccess = await _deploymentService.DeletePlanAsync(actionPlan);

            if (isSuccess == false)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
                return StatusCode(statusCode,
                    new ApiResponse(statusCode, $"Unable to delete the services for the action plan with id {id}"));
            }

            await _redisClient.ActionPlanDeleteAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error while deleting the specified action plan with id {id}", id);
            int statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"Could not delete the action plan with id: {id}"));
        }
    }


    /// <summary>
    /// Instantiate the resources for specified actions
    /// </summary>
    /// <param name="actions">List of actions to be instantiated</param>
    /// <returns>Http Status code and List of instantiated services</returns>
    [HttpPost]
    [Route("execute")]
    [ProducesResponseType(typeof(List<InstanceModel>), (int)HttpStatusCode.OK)]
    public async Task<IActionResult> InstantiateResources([FromBody] List<ActionModel> actions)
    {
        //TODO: instantiate services for action
        return Ok(new List<InstanceModel>());
    }

}