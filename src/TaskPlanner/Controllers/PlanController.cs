using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.Common.Services;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Contracts.Requests;
using Middleware.TaskPlanner.Exceptions;
using Middleware.TaskPlanner.Services;

namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class PlanController : ControllerBase
    {
        private readonly IActionPlanner _actionPlanner;
        private readonly IMapper _mapper;
        private readonly IPublishService _publishService;
        private readonly ResourcePlanner.ResourcePlannerApiClient _resourcePlannerClient;
        private readonly IRedisInterfaceClientService _redisInterfaceClient;
        private readonly ILogger<PlanController> _logger;


        public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder,
            IRedisInterfaceClientService redisInterfaceClient, IMapper mapper,
            IPublishService publishService, ILogger<PlanController> logger)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _publishService = publishService;
            _logger = logger;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _redisInterfaceClient = redisInterfaceClient;
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
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest,
                    "Parameters were not specified or wrongly specified."));

            Guid id = inputModel.Id;
            bool lockResource = inputModel.LockResourceReUse;
            Guid robotId = inputModel.RobotId;

            try
            {
                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

                var (plan, robot2) =
                    await _actionPlanner.Plan(id, robotId);
                plan.ResourceLock = lockResource;

                // call resource planner for resources
                ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
                ResourcePlanner.RobotModel tmpRobotSend = _mapper.Map<ResourcePlanner.RobotModel>(robot2);
                ResourcePlanner.ResourceInput resourceInput = new ResourcePlanner.ResourceInput
                {
                    Robot = tmpRobotSend,
                    Task = tmpTaskSend
                };
                ResourcePlanner.TaskModel tmpFinalTask =
                    await _resourcePlannerClient.ResourceAsync(resourceInput);
                TaskModel resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);

                if (dryRun)
                    return Ok(resourcePlan);

                await _redisInterfaceClient.AddRelationAsync(robot2, resourcePlan, "OWNS");

                await _publishService.PublishPlanAsync(resourcePlan, robot2);

                return Ok(resourcePlan);
            }
            catch (Orchestrator.ApiException<ResourcePlanner.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
            }
            catch (Orchestrator.ApiException<Orchestrator.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
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

            Guid id = inputModel.Id; //task id
            bool lockResource = inputModel.LockResourceReUse;
            Guid robotId = inputModel.RobotId; //robot id
            bool contextKnown = inputModel.ContextKnown;
            List<DialogueModel> dialogueTemp = inputModel.Questions;

            try
            {
                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);
                // INFER ACTION SEQUENCE PROCESS
                var (plan, robot2) =
                    await _actionPlanner.InferActionSequence(id, contextKnown, lockResource, dialogueTemp, robotId);

                // call resource planner for resources
                ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
                ResourcePlanner.RobotModel tmpRobotSend = _mapper.Map<ResourcePlanner.RobotModel>(robot2);
                ResourcePlanner.ResourceInput resourceInput = new ResourcePlanner.ResourceInput
                {
                    Robot = tmpRobotSend,
                    Task = tmpTaskSend
                };
                ResourcePlanner.TaskModel tmpFinalTask =
                    await _resourcePlannerClient.GetResourcePlanAsync(resourceInput);
                TaskModel resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);

                if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
                    return Ok(resourcePlan);

                await _redisInterfaceClient.AddRelationAsync(robot2, resourcePlan, "OWNS");

                await _publishService.PublishPlanAsync(resourcePlan, robot2);

                //TODO: orchestrator has to create the relations between the instance and the location the services are deployed in
                /*Orchestrator.OrchestratorResourceInput tmpTaskOrchestratorSend =
                    _mapper.Map<Orchestrator.OrchestratorResourceInput>(resourcePlan);
                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);
         
                */
                return Ok(resourcePlan);
            }
            catch (Orchestrator.ApiException<ResourcePlanner.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
            }
            catch (Orchestrator.ApiException<Orchestrator.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
            }
            catch (Exception ex)
            {
                int statusCode = (int)HttpStatusCode.InternalServerError;
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
                await _publishService.PublishSwitchoverDeployInstance(request.ActionPlanId, request.InstanceId,
                    request.Destination, request.DestinationType);
                await _publishService.PublishSwitchoverDeleteInstance(request.ActionPlanId, request.InstanceId);

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
}