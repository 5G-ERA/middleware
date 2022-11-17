using System.Net;
using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using Middleware.TaskPlanner.Services;
using AutoMapper;
using Middleware.TaskPlanner.ApiReference;
using System;
using YamlDotNet.Core;
using Middleware.Common.Responses;

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
        private readonly IApiClientBuilder _apiClientBuilder;


        public RePlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _orchestratorClient = builder.CreateOrchestratorApiClient();
            _apiClientBuilder = builder;

        }


        [HttpPost] //http get replan 
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetReplan([FromBody] TaskReplanInputModel inputModel, bool dryRun = false)
        {
            try
            {
                var _redisApiClient = _apiClientBuilder.CreateRedisApiClient();
                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

                Guid oldTask = inputModel.TaskID; //Task the robot wanted to execute
                Guid robotId = inputModel.RobotId;
                bool contextKnown = inputModel.ContextKnown;
                bool CompleteReplan = inputModel.CompleteReplan; // The robot wants a partial or complete replan.
                List<DialogueModel> tempDialog = inputModel.Questions;

                RedisInterface.TaskModel tempOldTaskObject = await _redisApiClient.TaskGetByIdAsync(oldTask);
                TaskModel tempOldTaskModel = _mapper.Map<TaskModel>(tempOldTaskObject);

                //Adding the old plan to the old -tempOldTaskModel-
                RedisInterface.ActionPlanModel RedOldPlan = await _redisApiClient.GetLatestActionPlanByRobotIdAsync(robotId);
                ActionModel oldPlanModel = _mapper.Map<ActionModel>(RedOldPlan);
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

                // Create new relationship of types owns between robot and task.
                RedisInterface.TaskModel tempTaskObject = await _redisApiClient.TaskGetByIdAsync(oldTask);
                TaskModel tempTaskModel = _mapper.Map<TaskModel>(tempTaskObject);

                RedisInterface.GraphEntityModel tempRobotGraph = new RedisInterface.GraphEntityModel();
                tempRobotGraph.Id = robotId;
                tempRobotGraph.Name = robot.Name;

                RedisInterface.GraphEntityModel tempTaskGraph = new RedisInterface.GraphEntityModel();
                tempTaskGraph.Id = oldTask;
                tempTaskGraph.Name = tempTaskModel.Name;

                RedisInterface.RelationModel robotOwnsTaskRelation = new RedisInterface.RelationModel();
                robotOwnsTaskRelation.RelationName = "OWNS";
                robotOwnsTaskRelation.InitiatesFrom = tempRobotGraph;
                robotOwnsTaskRelation.PointsTo = tempTaskGraph;

                RedisInterface.RelationModel newRobotOwnsTaskRelation = await _redisApiClient.RobotAddRelationAsync(robotOwnsTaskRelation);


                //Deploy replan
                Orchestrator.OrchestratorResourceInput tmpTaskOrchestratorSend = _mapper.Map<Orchestrator.OrchestratorResourceInput>(tmpFinalTask);

                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);

                //Create LOCATED_AT relationship in redis from instance to edge/cloud resource.
                TaskModel tempNewPlan = _mapper.Map<TaskModel>(tmpFinalTask);

                List<ActionModel> actionSequence = tempNewPlan.ActionSequence;
                foreach (ActionModel action in actionSequence)
                {
                    RedisInterface.RelationModel tempRelationModel = new RedisInterface.RelationModel();
                    tempRelationModel.RelationName = "LOCATED_AT";

                    if (action.Placement.Contains("CLOUD"))
                    {
                        RedisInterface.CloudModel tempRedisCloud = await _redisApiClient.CloudGetDataByNameAsync(action.Placement);
                        CloudModel tempCloud = _mapper.Map<CloudModel>(tempRedisCloud);
                        RedisInterface.GraphEntityModel tempGraph = new RedisInterface.GraphEntityModel();
                        tempGraph.Name = tempCloud.Name;
                        tempGraph.Type = "CLOUD";
                        tempGraph.Id = tempCloud.Id;
                        tempRelationModel.PointsTo = tempGraph;
                    }
                    else
                    {
                        RedisInterface.EdgeModel tempRedisEdge = await _redisApiClient.EdgeGetDataByNameAsync(action.Placement);
                        EdgeModel tempEdge = _mapper.Map<EdgeModel>(tempRedisEdge);
                        RedisInterface.GraphEntityModel tempGraph = new RedisInterface.GraphEntityModel();
                        tempGraph.Name = tempGraph.Name;
                        tempGraph.Type = "EDGE";
                        tempGraph.Id = tempGraph.Id;
                        tempRelationModel.PointsTo = tempGraph;
                    }

                    foreach (InstanceModel instance in action.Services)
                    {
                        GraphEntityModel tempInstanceGraph = new GraphEntityModel(); //The instances that the action need.
                        tempInstanceGraph.Name = instance.Name;
                        tempInstanceGraph.Id = instance.Id;
                        tempInstanceGraph.Type = "INSTANCE";

                        //Add all the located_at relationships between all instances of 1 action and the resources been edge/cloud
                        RedisInterface.RelationModel newRelation = await _redisApiClient.InstanceAddRelationAsync(tempRelationModel);
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