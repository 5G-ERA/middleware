using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.TaskPlanner.ApiReference;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.TaskPlanner.Contracts.Requests;
using Middleware.TaskPlanner.Services;


namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class RePlanController : ControllerBase
    {
        private readonly IActionPlanner _actionPlanner;
        private readonly IMapper _mapper;
        private readonly ResourcePlanner.ResourcePlannerApiClient _resourcePlannerClient;
        private readonly Orchestrator.OrchestratorApiClient _orchestratorClient;
        private readonly IRedisInterfaceClient _redisInterfaceClient;

        public RePlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper, IRedisInterfaceClient redisInterfaceClient)
        {
            _redisInterfaceClient = redisInterfaceClient;
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _orchestratorClient = builder.CreateOrchestratorApiClient();

        }

        [HttpPost] //http get replan 
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetReplan([FromBody] CreateRePlanRequest inputModel, bool dryRun = false)
        {
            try
            {
                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

                Guid oldTask = inputModel.TaskID; //Task the robot wanted to execute
                Guid robotId = inputModel.RobotId;
                bool contextKnown = inputModel.ContextKnown;
                bool CompleteReplan = inputModel.CompleteReplan; // The robot wants a partial or complete replan.
                List<DialogueModel> tempDialog = inputModel.Questions;

                TaskModel tempOldTaskModel = (await _redisInterfaceClient.TaskGetByIdAsync(oldTask)).ToTask();

                //Adding the old plan to the old -tempOldTaskModel-
                ActionPlanModel oldPlanModel = await _redisInterfaceClient.GetLatestActionPlanByRobotIdAsync(robotId);
                if (oldPlanModel == null)
                {
                    throw new NullReferenceException();
                }
                tempOldTaskModel.ActionPlanId = oldPlanModel.Id;

                var (plan, oldPlan, robot) = await _actionPlanner.ReInferActionSequence(tempOldTaskModel, robotId, contextKnown, CompleteReplan, tempDialog);

                ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
                ResourcePlanner.TaskModel tmpOldPlanSend = _mapper.Map<ResourcePlanner.TaskModel>(oldPlan);
                ResourcePlanner.RobotModel tmpRobotSend = _mapper.Map<ResourcePlanner.RobotModel>(robot);

                ResourcePlanner.ResourceReplanInputModel tempReplanInput = new ResourcePlanner.ResourceReplanInputModel
                {
                    Task = tmpTaskSend,
                    OldTask = tmpOldPlanSend,
                    Robot = tmpRobotSend,
                    FullReplan = CompleteReplan
                };

                ResourcePlanner.TaskModel tmpFinalTask = await _resourcePlannerClient.GetResourceRePlanAsync(tempReplanInput);//API call to resource planner

                if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
                    return Ok(tmpFinalTask);

                //Delete previous plan in orchestrator and graph. --> TODO: AL 06/11/22 The replan is not shown in the graph, only in the ActionPlanModel index db. 
                await _orchestratorClient.DeletePlanByIdAsync(oldPlan.ActionPlanId);

                await _redisInterfaceClient.AddRelationAsync(robot, plan, "OWNS");

                // Deploy replan
                Orchestrator.OrchestratorResourceInput tmpTaskOrchestratorSend = new Orchestrator.OrchestratorResourceInput()
                {
                    Task = _mapper.Map<Orchestrator.TaskModel>(plan),
                    Robot = _mapper.Map<Orchestrator.RobotModel>(robot)
                };

                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel orchestratedPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);

                //Create LOCATED_AT relationship in redis from instance to edge/cloud resource.                
                foreach (ActionModel action in orchestratedPlan.ActionSequence)
                {
                    BaseModel location;
                    //TODO: recognize by type PlacementType property
                    location = action.Placement.ToLower().Contains("cloud")
                        ? (await _redisInterfaceClient.GetCloudByNameAsync(action.Placement)).ToCloud()
                        : (await _redisInterfaceClient.GetEdgeByNameAsync(action.Placement)).ToEdge();

                    foreach (InstanceModel instance in action.Services)
                    {
                        var result = await _redisInterfaceClient.AddRelationAsync(instance, location, "LOCATED_AT");
                    }

                }

                return Ok(orchestratedPlan);
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
/*

{
    "TaskId": "guid"
  "ActionPlanId": "guid",
  "ActionSequence": [
    {
        "ActionId": 2,
      "Status": "Done/In progress/Failed/Unable to execute",
      "Timestamp": "dd/MM/yyyy HH24:ss.mmm"
    }
  ],
  
}
*/