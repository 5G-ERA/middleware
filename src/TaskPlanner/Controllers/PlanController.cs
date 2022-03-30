using System.Net;
using AutoMapper;
using Middleware.Common.Models;
using Microsoft.AspNetCore.Mvc;
using Middleware.TaskPlanner.ApiReference;
using Middleware.TaskPlanner.ResourcePlanner;
using ActionModel = Middleware.Common.Models.ActionModel;
using TaskModel = Middleware.Common.Models.TaskModel;


namespace Middleware.TaskPlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class PlanController : ControllerBase
    {
        private readonly IActionPlanner _actionPlanner;
        private readonly IMapper _mapper;
        private readonly ResourcePlannerApiClient _resourcePlannerClient;

        public PlanController(IActionPlanner actionPlanner, IApiClientBuilder builder, IMapper mapper)
        {
            _actionPlanner = actionPlanner;
            _mapper = mapper;
            _resourcePlannerClient = builder.CreateResourcePlannerApiClient();
        }

        [HttpPost] //http get request
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        public async Task<ActionResult<TaskModel>> GetPlan([FromBody]TaskPlannerInputModel inputModel)
        {
            Guid id = inputModel.Id;

            _actionPlanner.Initialize(new List<ActionModel>(), DateTime.Now);

            TaskModel plan = await _actionPlanner.InferActionSequence(id);

            // call resource planner for resources
            ResourcePlanner.TaskModel tmpTaskSend = _mapper.Map<ResourcePlanner.TaskModel>(plan);
            ResourcePlanner.TaskModel tmpFinalTask = await _resourcePlannerClient.GetResourcePlanAsync(tmpTaskSend);

            TaskModel finalPlan = _mapper.Map<TaskModel>(tmpFinalTask);
            // call orchestrator for deployment of the resources
            // TODO: for 13th April deadline
            return Ok(finalPlan);
        }

      
    }
}
