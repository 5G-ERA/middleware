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
    public async Task<IActionResult> InstantiateNewPlan([FromBody] TaskModel task)
    {

        if (task is null)
        {
            return BadRequest("Plan is not specified");
        }

        try
        {
            var result = await _deploymentService.DeployAsync(task);
            if (result == false)
            {
                //TODO: provide more detailed information on what went wrong with the deployment of the task
                return Problem("There was a problem while deploying the task instance: {ex");
            }
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was a problem while deploying the task instance: {task}", task);
            return Problem("There was a problem while deploying the task instance: {ex}", ex.Message);
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
    [ProducesResponseType(typeof(string), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(string), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<ActionPlanModel>> GetActionsByPlanId(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest("Guid has not been specified");
        }
        try
        {
            RedisInterface.ActionPlanModel riActionPlan = await _redisClient.ActionPlanGetByIdAsync(id);
            if (riActionPlan is null)
            {
                return NotFound("Specified action plan with specified id not found");
            }
            ActionPlanModel actionPlan = _mapper.Map<ActionPlanModel>(riActionPlan);

            //TODO: retrieve the latest info about the plan?
            return Ok(actionPlan);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "There was an error while retrieving the action plan information");
            return Problem($"There was an error while retrieving the action plan information: {ex.Message}");
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
    [ProducesResponseType(typeof(string),(int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(string),(int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(string), (int)HttpStatusCode.InternalServerError)]

    public async Task<ActionResult> DeletePlanById(Guid id)
    {
        if (id == Guid.Empty)
        {
            return BadRequest("Id of the plan  has to be specified");
        }

        try
        {
            RedisInterface.ActionPlanModel riActionPlan = await _redisClient.ActionPlanGetByIdAsync(id);
            if (riActionPlan is null)
            {
                return NotFound($"Could not find the Action Plan with specified id: {id}");
            }

            ActionPlanModel actionPlan = _mapper.Map<ActionPlanModel>(riActionPlan);

            var isSuccess = await _deploymentService.DeletePlanAsync(actionPlan);

            if (isSuccess == false)
            {
                return Problem($"Unable to delete the services for the action plan with id {id}");
            }

            await _redisClient.ActionPlanDeleteAsync(id);
            return Ok();
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Error while deleting the specified action plan with id {id}", id);
            return Problem($"Could not delete the action plan with id {id}");
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