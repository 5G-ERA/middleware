using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;
using Middleware.Common.Responses;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        private readonly IResourcePlanner _resourcePlanner;
        private readonly IMapper _mapper;
        private readonly ILogger _logger;

        public ResourceController(IResourcePlanner resourcePlanner, IMapper mapper, ILogger<ResourceController> logger)
        {
            _resourcePlanner = resourcePlanner;
            _mapper = mapper;
            _logger = logger;
        }

       
        /// <summary>
        /// Return an updated taskModel with the resource specs.
        /// </summary>
        /// <param name="resource"></param>
        /// <returns></returns>
        [HttpPost(Name = "GetResourcePlan")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        
        public async Task<ActionResult<TaskModel>> GetResource([FromBody] ResourceInput resource)
        {
            try
            {
              
                TaskModel updatedTask = await _resourcePlanner.Plan(resource.Task, resource.Robot);
                
                return Ok(updatedTask);
            }
            catch (Orchestrator.ApiException<RedisInterface.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
            }
            catch (Orchestrator.ApiException<Orchestrator.ApiResponse> apiEx)
            {
                return StatusCode(apiEx.StatusCode,_mapper.Map<ApiResponse>(apiEx.Result));
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