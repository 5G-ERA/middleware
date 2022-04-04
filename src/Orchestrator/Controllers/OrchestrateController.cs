using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Orchestrator.ApiReference;
using Middleware.Orchestrator.Osm;
using Middleware.Orchestrator.RedisInterface;
using ActionModel = Middleware.Common.Models.ActionModel;
using InstanceModel = Middleware.Common.Models.InstanceModel;
using TaskModel = Middleware.Common.Models.TaskModel;

namespace Middleware.Orchestrator.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class OrchestrateController : Controller
{
    private readonly IMapper _mapper;
    private readonly RedisApiClient _client;

    public OrchestrateController(IApiClientBuilder apiClientBuilder, IMapper mapper)
    {
        _mapper = mapper;
        _client = apiClientBuilder.CreateRedisApiClient();
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
        
        //TODO: instantiate new plan
        return Ok(task);
    }

    /// <summary>
    /// Get actions deployed with the plan by the plan Id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("plan/{id}", Name = "GetActionsByPlanId")]
    [ProducesResponseType(typeof(List<ActionModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType((int)HttpStatusCode.NotFound)]
    public async Task<ActionResult<List<ActionModel>>> GetActionsByPlanId(Guid id)
    {
        return Ok(new List<ActionModel>());
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
        // TODO: Delete plan with specified Id
        return Ok();
    }
    /// <summary>
    /// Delete plan by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}", Name = "DeletePlanById")]
    public async Task<ActionResult> DeletePlanById(Guid id)
    {
        // TODO: Delete plan with specified Id
        return Ok();
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