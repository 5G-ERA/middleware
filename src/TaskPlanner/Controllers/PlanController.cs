using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
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
        private readonly IApiClientBuilder _apiClientBuilder;
       

        public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _orchestratorClient = builder.CreateOrchestratorApiClient();
            _apiClientBuilder = builder;

        }

        [HttpPost] 
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<TaskModel>> GetPlan([FromBody] TaskPlannerInputModel inputModel, bool dryRun = false)
        {
            Guid id = inputModel.Id;//task id
            bool lockResource = inputModel.LockResourceReUse;
            Guid robotId = inputModel.RobotId; //robot id
            bool contextKnown = inputModel.ContextKnown;
            List<Common.Models.DialogueModel> DialogueTemp = inputModel.Questions;
            List<RosTopicModel> InputTopics = inputModel.InputTopics;
            List<RosTopicModel> OutputTopics = inputModel.OutputTopics;

            try
            {

                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

                var _redisApiClient = _apiClientBuilder.CreateRedisApiClient();
                // Create a relationship of types OWNS between a robot and a task objects.

                RedisInterface.RobotModel tempRobotObject = await _redisApiClient.RobotGetByIdAsync(robotId);
                CloudModel tempRobotModel = _mapper.Map<CloudModel>(tempRobotObject);

                RedisInterface.TaskModel tempTaskObject = await _redisApiClient.TaskGetByIdAsync(id);
                TaskModel tempTaskModel = _mapper.Map<TaskModel>(tempTaskObject);

                RedisInterface.GraphEntityModel tempRobotGraph = new RedisInterface.GraphEntityModel();
                tempRobotGraph.Id = robotId;
                tempRobotGraph.Name = tempRobotModel.Name;

                RedisInterface.GraphEntityModel tempTaskGraph = new RedisInterface.GraphEntityModel();
                tempTaskGraph.Id = id;
                tempTaskGraph.Name = tempTaskModel.Name;

                RedisInterface.RelationModel robotOwnsTaskRelation = new RedisInterface.RelationModel();
                robotOwnsTaskRelation.RelationName = "OWNS";
                robotOwnsTaskRelation.InitiatesFrom = tempRobotGraph;
                robotOwnsTaskRelation.PointsTo = tempTaskGraph;

                RedisInterface.RelationModel newRobotOwnsTaskRelation = await _redisApiClient.RobotAddRelationAsync(robotOwnsTaskRelation);

                // INFER ACTION SEQUENCE PROCESS
                var (plan, robot) = await _actionPlanner.InferActionSequence(InputTopics, OutputTopics, id, contextKnown,lockResource, DialogueTemp, robotId);

                // call resource planner for resources
                ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
                ResourcePlanner.RobotModel tmpRobotSend = _mapper.Map<ResourcePlanner.RobotModel>(robot);
                ResourcePlanner.ResourceInput resourceInput = new ResourcePlanner.ResourceInput
                {
                    Robot = tmpRobotSend,
                    Task = tmpTaskSend
                };
                ResourcePlanner.TaskModel tmpFinalTask = await _resourcePlannerClient.GetResourcePlanAsync(resourceInput);//API call to resource planner

                TaskModel resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);


                if (dryRun) // Will skip the orchestrator if true (will not deploy the actual plan.)
                    return Ok(resourcePlan);
                // call orchestrator for deployment of the resources
                // Orchestrator.TaskModel tmpTaskOrchestratorSend = _mapper.Map<Orchestrator.TaskModel>(resourcePlan);
                Orchestrator.OrchestratorResourceInput tmpTaskOrchestratorSend = _mapper.Map<Orchestrator.OrchestratorResourceInput>(resourcePlan);
            
                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);

                //Create LOCATED_AT relationship in redis from instance to edge/cloud resource.
                //var _redisApiClient = _apiClientBuilder.CreateRedisApiClient();

                List<ActionModel> actionSequence = resourcePlan.ActionSequence;
                foreach (ActionModel action in actionSequence)
                {
                    RedisInterface.RelationModel tempRelationModel = new RedisInterface.RelationModel();
                    tempRelationModel.RelationName = "LOCATED_AT";

                    if ( action.Placement.Contains("CLOUD")) { 
                        RedisInterface.CloudModel tempRedisCloud = await _redisApiClient.CloudGetDataByNameAsync(action.Placement);
                        CloudModel tempCloud = _mapper.Map<CloudModel>(tempRedisCloud);
                        RedisInterface.GraphEntityModel tempGraph = new RedisInterface.GraphEntityModel();
                        tempGraph.Name = tempCloud.Name;
                        tempGraph.Type = "CLOUD";
                        tempGraph.Id = tempCloud.Id;
                        tempRelationModel.PointsTo = tempGraph;
                    }
                    else {
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
                int statusCode = (int) HttpStatusCode.InternalServerError;
                return StatusCode(statusCode,
                    new ApiResponse(statusCode, $"There was an error while preparing the task plan: {ex.Message}"));
            }
        }


    }
}
