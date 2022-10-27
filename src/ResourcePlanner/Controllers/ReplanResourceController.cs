using System.Collections.Generic;
using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using Middleware.ResourcePlanner.ApiReference;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ReplanResourceController : ControllerBase
    {
        private readonly IResourcePlanner _resourcePlanner;
        private readonly IActionPlanRepository _actionPlanRepository;
        private readonly IMapper _mapper;
        private readonly ILogger _logger;
        private readonly IApiClientBuilder _apiClientBuilder;


        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="resourcePlanner"></param>
        /// <param name="mapper"></param>
        /// <param name="logger"></param>
        public ReplanResourceController(IApiClientBuilder apiClientBuilder, IResourcePlanner resourcePlanner, IMapper mapper, ILogger<ResourceController> logger)
        {
            _resourcePlanner = resourcePlanner;
            _mapper = mapper;
            _logger = logger;
            _apiClientBuilder = apiClientBuilder;

        }

        [HttpPost(Name = "GetResourceRePlan")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]

        public async Task<ActionResult<TaskModel>> GetResourceRePlan([FromBody] ResourceInput resource)
        {
            var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
            try
            {
                //Lua query to get plan by robot and lastest timestamped.
                List<RedisInterface.ActionPlanModel> reActionPlans = (await redisApiClient.GetActionPlanByRobotIdAsync(resource.Robot.Id)).ToList();
                List<ActionPlanModel> actionPlans = _mapper.Map<List<ActionPlanModel>>(reActionPlans);

                //Get the newest task of robot.
                Dictionary<ActionPlanModel, DateTime> tempDic = new Dictionary<ActionPlanModel, DateTime>();
                Dictionary<ActionPlanModel, DateTime> OrderedTempDic = new Dictionary<ActionPlanModel, DateTime>();

                // Complete tempDic
                foreach (ActionPlanModel plan in actionPlans)
                {
                    DateTime d;
                    DateTime.TryParseExact(plan.Status, "ggyyyy$dd-MMM (dddd)", System.Globalization.CultureInfo.InvariantCulture, System.Globalization.DateTimeStyles.None, out d);
                    tempDic.Add(plan, d);
                }

                // Order a new dictionary
                foreach (KeyValuePair<ActionPlanModel, DateTime> pair in tempDic.OrderByDescending(p => p.Value))
                {
                    OrderedTempDic.Add(pair.Key, pair.Value);
                }

                // Get last item which is the latest plan.
                ActionPlanModel last = OrderedTempDic.Keys.First();

                // Create a TaskModel entity with the values from the ActionPlanModel.
                TaskModel tempTask = new TaskModel();
                tempTask.Id = last.TaskId;
                tempTask.ActionPlanId = last.Id;
                tempTask.ActionSequence = last.ActionSequence;
                tempTask.FullReplan = resource.FullReplan;


                TaskModel updatedTask = await _resourcePlanner.RePlan(resource.Task, tempTask, resource.Robot, resource.FullReplan);

                return Ok(updatedTask);
            }
            catch (Orchestrator.ApiException<RedisInterface.ApiResponse> apiEx)
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
                    new ApiResponse(statusCode, $"There was an error while collecting the resources: {ex.Message}"));
            }
        }
    }

}
