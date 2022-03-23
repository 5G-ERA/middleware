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

    [HttpGet]
    public async Task<IActionResult> Get()
    {
        var actions = await _client.ActionGetAllAsync();
        var first = actions.FirstOrDefault();
        var action = _mapper.Map<ActionModel>(first);
        return Ok(action);
    }

    /// <summary>
    /// Get plan information by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("{id}")]
    public async Task<ActionResult<TaskModel>> GetPlanById(Guid id)
    {
        return Ok(new List<ActionModel>());
    }

    /// <summary>
    /// Execute the plan from the body of the request
    /// </summary>
    /// <param name="task"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("plan")]
    public async Task<IActionResult> ExecutePlan([FromBody] TaskModel task)
    {
        //TODO: do something with a task
        return Ok(task);
    }

    /// <summary>
    /// Delete plan by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("plan/{id}")]
    public async Task<ActionResult> DeletePlanById(Guid id)
    {
        // TODO: Delete plan with specified Id
        return Ok();
    }
    /// <summary>
    /// Instantiate the resources specified in the list
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

    /// <summary>
    /// Deleted the specified action and its resources by its id
    /// </summary>
    /// <param name="id">Id of an action</param>
    /// <returns>HttpStatus Code if action has succeeded</returns>
    [HttpDelete]
    [Route("{id}")]
    public async Task<ActionResult> DeleteActionById(Guid id)
    {
        // TODO: Delete plan with specified Id
        return Ok();
    }

}