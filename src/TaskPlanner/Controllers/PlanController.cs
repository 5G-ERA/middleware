using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.TaskPlanner.ApiReference;

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

        public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
            _orchestratorClient = builder.CreateOrchestratorApiClient();
        }

        [HttpPost] 
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        public async Task<ActionResult<TaskModel>> GetPlan([FromBody] TaskPlannerInputModel inputModel, bool dryRun = false)
        {
            Guid id = inputModel.Id;
            bool lockResource = inputModel.LockResourceReUse;
            List<Common.Models.DialogueModel> DialogueTemp = inputModel.Questions;

            try
            {

                _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

                TaskModel plan = await _actionPlanner.InferActionSequence(id, lockResource, DialogueTemp);

                // call resource planner for resources
                ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
                ResourcePlanner.TaskModel tmpFinalTask = await _resourcePlannerClient.GetResourcePlanAsync(tmpTaskSend);

                TaskModel resourcePlan = _mapper.Map<TaskModel>(tmpFinalTask);

                if (dryRun)
                    return Ok(resourcePlan);
                // call orchestrator for deployment of the resources
                Orchestrator.TaskModel tmpTaskOrchestratorSend = _mapper.Map<Orchestrator.TaskModel>(resourcePlan);
                Orchestrator.TaskModel tmpFinalOrchestratorTask =
                    await _orchestratorClient.InstantiateNewPlanAsync(tmpTaskOrchestratorSend);
                TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalOrchestratorTask);

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
