using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Responses;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.Services;

namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class PlanController : ControllerBase
    {
        private readonly IActionPlanner _actionPlanner;
        private readonly IMapper _mapper;
        private readonly ResourcePlanner.ResourcePlannerApiClient _resourcePlannerClient;
        private readonly Orchestrator.OrchestratorApiClient _orchestratorClient;        
        private readonly IRedisInterfaceClientService _redisInterfaceClient;


        public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder,
            IRedisInterfaceClientService redisInterfaceClient, IMapper mapper)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _orchestratorClient = builder.CreateOrchestratorApiClient();
            _redisInterfaceClient = redisInterfaceClient;
        }

        [HttpPost]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<TaskModel>> GetPlan([FromBody] TaskPlannerInputModel inputModel,
            bool dryRun = false)
        {
            if (inputModel == null)
            {
                return BadRequest("Parameters were not specified.");
            }

            if (inputModel.IsValid() == false)
            {
                return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified or wrongly specified."));
            }

            Guid id = inputModel.Id; //task id
            bool lockResource = inputModel.LockResourceReUse;
            Guid robotId = inputModel.RobotId; //robot id
            bool contextKnown = inputModel.ContextKnown;
            List<Common.Models.DialogueModel> dialogueTemp = inputModel.Questions;

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
                    await _resourcePlannerClient.GetResourcePlanAsync(resourceInput); //API call to resource planner
                TaskModel resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);

                if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
                    return Ok(resourcePlan);
                
                // var robot = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
                // var task = await _redisInterfaceClient.TaskGetByIdAsync(id);
                await _redisInterfaceClient.AddRelationAsync(robot2, resourcePlan, "OWNS");
                
                // call orchestrator for deployment of the resources
                // Orchestrator.TaskModel tmpTaskOrchestratorSend = _mapper.Map<Orchestrator.TaskModel>(resourcePlan);
                Orchestrator.OrchestratorResourceInput tmpTaskOrchestratorSend =
                    _mapper.Map<Orchestrator.OrchestratorResourceInput>(resourcePlan);

                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);
                
                //Create LOCATED_AT relationship in redis from instance to edge/cloud resource.
                foreach (ActionModel action in resourcePlan.ActionSequence)
                {
                    BaseModel location;
                    //TODO: recognize by type PlacementType property
                    location = action.Placement.ToLower().Contains("cloud")
                        ? await _redisInterfaceClient.GetCloudByNameAsync(action.Placement)
                        : await _redisInterfaceClient.GetEdgeByNameAsync(action.Placement);

                    foreach (InstanceModel instance in action.Services)
                    {
                        var result = await _redisInterfaceClient.AddRelationAsync(instance, location, "LOCATED_AT");
                    }
                }

                return Ok(finalPlan);
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
    }
}