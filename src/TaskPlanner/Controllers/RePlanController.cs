using System.Net;
using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using Middleware.TaskPlanner.Services;
using AutoMapper;
using Middleware.TaskPlanner.ApiReference;
using System;
using YamlDotNet.Core;

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
        public async Task<ActionResult<TaskModel>> GetReplan([FromBody] TaskReplanInputModel inputModel)
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

            return Ok(tmpFinalTask);
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