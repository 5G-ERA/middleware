using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
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
        public async Task<ActionResult<TaskModel>> GetResourceRePlan([FromBody] ResourceReplanInputModel resource)
        {
            try
            {

                TaskModel updatedTask = await _resourcePlanner.RePlan(resource.Task, resource.oldTask, resource.Robot, resource.FullReplan);

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
