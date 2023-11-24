using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Contracts.Requests;
using Middleware.TaskPlanner.Exceptions;
using Middleware.TaskPlanner.ResourcePlanner;
using Middleware.TaskPlanner.Services;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using TaskModel = Middleware.Models.Domain.TaskModel;

namespace Middleware.TaskPlanner.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class PlanController : ControllerBase
{
    private readonly IActionPlanner _actionPlanner;
    private readonly ILogger<PlanController> _logger;
    private readonly IMapper _mapper;
    private readonly IPublishService _publishService;
    private readonly ResourcePlannerApiClient _resourcePlannerClient;


    public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper,
        IPublishService publishService, ILogger<PlanController> logger)
    {
        _actionPlanner = actionPlanner;
        _mapper = mapper;
        _publishService = publishService;
        _logger = logger;
        _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
    }

    [HttpPost]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<TaskModel>> GetPlan([FromBody] CreatePlanRequest inputModel,
        bool dryRun = false)
    {
        if (inputModel == null)
            return BadRequest("Parameters were not specified.");

        if (inputModel.IsValid() == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Parameters were not specified or wrongly specified."));
        }

        var id = inputModel.Id;
        var lockResource = inputModel.LockResourceReUse;
        var robotId = inputModel.RobotId;

        try
        {
            _actionPlanner.Initialize(new(), DateTime.Now);

            var (plan, robot2) =
                await _actionPlanner.Plan(id, robotId);
            plan.ResourceLock = lockResource;

            var resourcePlan = await _publishService.RequestResourcePlan(plan, robot2);

            if (!resourcePlan.IsSuccess)
            {
                var statusCode = (int)HttpStatusCode.BadRequest;
                return StatusCode(statusCode,
                    new ApiResponse(statusCode,
                        $"Error while preparing ResourcePlan: {resourcePlan.Error}"));
            }

            if (dryRun)
                return Ok(resourcePlan);

            await _publishService.PublishPlanAsync(resourcePlan.Task, robot2);

            return Ok(resourcePlan);
        }
        catch (Orchestrator.ResourcePlannerApiClient.ApiException<ResourcePlanner.ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Orchestrator.ResourcePlannerApiClient.ApiException<ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"There was an error while preparing the task plan: {ex.Message}"));
        }
    }

    [HttpPost("semantic")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<TaskModel>> GetSemanticPlan([FromBody] CreatePlanRequest inputModel,
        bool dryRun = false)
    {
        if (inputModel == null)
            return BadRequest("Parameters were not specified.");

        if (inputModel.ContextKnown == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Semmantic planning is not yet available."));
        }

        if (inputModel.IsValid() == false)
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                "Parameters were not specified or wrongly specified."));
        }

        var id = inputModel.Id; //task id
        var lockResource = inputModel.LockResourceReUse;
        var robotId = inputModel.RobotId; //robot id
        var contextKnown = inputModel.ContextKnown;
        var dialogueTemp = inputModel.Questions;

        try
        {
            _actionPlanner.Initialize(new(), DateTime.Now);
            // INFER ACTION SEQUENCE PROCESS
            var (plan, robot2) =
                await _actionPlanner.InferActionSequence(id, contextKnown, lockResource, dialogueTemp, robotId);

            // call resource planner for resources
            var tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
            var tmpRobotSend = _mapper.Map<RobotModel>(robot2);
            var resourceInput = new ResourceInput
            {
                Robot = tmpRobotSend,
                Task = tmpTaskSend
            };
            var tmpFinalTask =
                await _resourcePlannerClient.GetResourcePlanAsync(resourceInput);
            var resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);

            if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
                return Ok(resourcePlan);

            await _publishService.PublishPlanAsync(resourcePlan, robot2);
            return Ok(resourcePlan);
        }
        catch (Orchestrator.ResourcePlannerApiClient.ApiException<ResourcePlanner.ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"There was an error while preparing the task plan: {ex.Message}"));
        }
    }

    [HttpPost("switchover")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> Switchover([FromBody] PerformSwitchoverRequest request)
    {
        try
        {
            await _publishService.PublishSwitchoverDeployInstance(request.ActionPlanId, request.ActionId,
                request.Destination, request.DestinationType);
            await _publishService.PublishSwitchoverDeleteInstance(request.ActionPlanId, request.ActionId);

            return Ok();
        }
        catch (IncorrectLocationException ex)
        {
            _logger.LogInformation(
                "Attempted to obtain non existing Middleware within organization. Name: {0}, Type: {1}",
                ex.LocationName, ex.LocationType);
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, ex.Message));
        }
    }
}