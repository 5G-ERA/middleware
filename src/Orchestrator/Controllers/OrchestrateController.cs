using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class OrchestrateController : MiddlewareController
{
    private readonly IDeploymentService _deploymentService;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public OrchestrateController(IDeploymentService deploymentService,
        ILogger<OrchestrateController> logger,
        IRedisInterfaceClient redisInterfaceClient)
    {
        _deploymentService = deploymentService;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
    }

    /// <summary>
    ///     Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan", Name = "InstantiateNewPlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> InstantiateNewPlan([FromBody] OrchestratorResourceInput request)
    {
        if (request is null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Plan is not specified");
        try
        {
            var result = await _deploymentService.DeployActionPlanAsync(request.Task, request.Robot.Id);
            if (result.IsSuccess == false)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, "plan", result.Error);
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was a problem while deploying the task instance: {task}", request);
            return ErrorMessageResponse(HttpStatusCode.BadRequest, "plan",
                $"There was a problem while deploying the task instance: {ex.Message}");
        }

        return Ok(request);
    }

    /// <summary>
    ///     Get the action plan by the ActionPlanId identifier
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
        int statusCode;
        try
        {
            var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(id);
            if (actionPlan is null)
            {
                statusCode = (int)HttpStatusCode.NotFound;
                return NotFound(new ApiResponse(statusCode, "Specified action plan with specified id not found"));
            }

            //TODO: retrieve the latest info about the plan?
            return Ok(actionPlan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was an error while retrieving the action plan information");
            statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode,
                    $"There was an error while retrieving the action plan information: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Request orchestration of the resources defied in the plan
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPatch]
    [Route("plan", Name = "UpdatePlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    public Task<IActionResult> UpdatePlan([FromBody] TaskModel task)
    {
        //TODO: redeploy services for new plan
        return Task.FromResult<IActionResult>(Ok(task));
    }

    /// <summary>
    ///     Deletes the instances instantiated with the specified action
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("action/{id}", Name = "DeleteActionById")]
    [ProducesResponseType((int)HttpStatusCode.OK)]
    [ProducesResponseType((int)HttpStatusCode.NotFound)]
    public Task<IActionResult> DeleteActionById(Guid id)
    {
        // TODO: Delete action with specified Id
        return Task.FromResult<IActionResult>(Ok());
    }

    /// <summary>
    ///     Delete plan by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}", Name = "DeletePlanById")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeletePlanById(Guid id)
    {
        if (id == Guid.Empty)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(id), "Id of the plan must be specified");
        try
        {
            var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(id);
            if (actionPlan is null)
            {
                return ErrorMessageResponse(HttpStatusCode.NotFound, nameof(id),
                    $"Could not find the Action Plan with specified id: {id}");
            }

            var result = await _deploymentService.DeletePlanAsync(actionPlan);
            if (result.IsSuccess == false)
            {
                return ErrorMessageResponse(HttpStatusCode.BadRequest, "plan", $"Unable to delete the services for the action plan with id: {id} because: {result.Error}");
            }

            await _redisInterfaceClient.ActionPlanDeleteAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error while deleting the specified action plan with id {id}", id);
            return ErrorMessageResponse(HttpStatusCode.BadRequest, "plan", $"Could not delete the action plan with id: {id}");
        }
    }


    /// <summary>
    ///     Instantiate the resources for specified actions
    /// </summary>
    /// <param name="actions">List of actions to be instantiated</param>
    /// <returns>Http Status code and List of instantiated services</returns>
    [HttpPost]
    [Route("execute")]
    [ProducesResponseType(typeof(List<InstanceModel>), (int)HttpStatusCode.OK)]
    public Task<IActionResult> InstantiateResources([FromBody] List<ActionModel> actions)
    {
        //TODO: instantiate services for action
        return Task.FromResult<IActionResult>(Ok(new List<InstanceModel>()));
    }

    public record OrchestratorResourceInput(TaskModel Task, RobotModel Robot);
}