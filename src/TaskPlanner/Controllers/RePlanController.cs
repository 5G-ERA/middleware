using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Contracts.Requests;
using Middleware.TaskPlanner.ResourcePlanner;
using Middleware.TaskPlanner.Services;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using TaskModel = Middleware.Models.Domain.TaskModel;

namespace Middleware.TaskPlanner.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class RePlanController : ControllerBase
{
    private readonly IActionPlanner _actionPlanner;
    private readonly IMapper _mapper;
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly ResourcePlannerApiClient _resourcePlannerClient;

    public RePlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper,
        IRedisInterfaceClient redisInterfaceClient)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _actionPlanner = actionPlanner;
        _mapper = mapper;
        _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
    }

    [HttpPost] //http get replan 
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    public async Task<ActionResult<TaskModel>> GetReplan([FromBody] CreateRePlanRequest inputModel, bool dryRun = false)
    {
        try
        {
            _actionPlanner.Initialize(new(), DateTime.Now);

            var oldTask = inputModel.TaskID; //Task the robot wanted to execute
            var robotId = inputModel.RobotId;
            var contextKnown = inputModel.ContextKnown;
            var completeReplan = inputModel.CompleteReplan; // The robot wants a partial or complete replan.
            var tempDialog = inputModel.Questions;

            var tempOldTaskModel = (await _redisInterfaceClient.TaskGetByIdAsync(oldTask)).ToTask();

            //Adding the old plan to the old -tempOldTaskModel-
            var oldPlanModel = await _redisInterfaceClient.GetLatestActionPlanByRobotIdAsync(robotId);
            if (oldPlanModel == null) throw new NullReferenceException();
            tempOldTaskModel.ActionPlanId = oldPlanModel.Id;

            var (plan, oldPlan, robot) = await _actionPlanner.ReInferActionSequence(tempOldTaskModel, robotId,
                contextKnown, completeReplan, tempDialog);

            var tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
            var tmpOldPlanSend = _mapper.Map<ResourcePlanner.TaskModel>(oldPlan);
            var tmpRobotSend = _mapper.Map<RobotModel>(robot);

            var tempReplanInput = new ResourceReplanInputModel
            {
                Task = tmpTaskSend,
                OldTask = tmpOldPlanSend,
                Robot = tmpRobotSend,
                FullReplan = completeReplan
            };

            var tmpFinalTask =
                await _resourcePlannerClient.GetResourceRePlanAsync(tempReplanInput); //API call to resource planner

            //if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
            return Ok(tmpFinalTask);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode,
                    $"There was an error while preparing the task plan: {ex.Message}"));
        }
    }
}