using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Models;

namespace Middleware.ResourcePlanner.Controllers
{
    [ApiController]
    [Route("api/v1/[controller]")]
    public class ResourceController : ControllerBase
    {
        private readonly IResourcePlanner _resourcePlanner;
        private readonly IMapper _mapper;

        public ResourceController(IResourcePlanner resourcePlanner, IMapper mapper)
        {
            _resourcePlanner = resourcePlanner;
            _mapper = mapper;
        }

        [HttpPost(Name = "GetResourcePlan")]
        [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
        [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
        
        public async Task<ActionResult<TaskModel>> GetResource([FromBody] TaskModel inputModel)
        {
            try
            {
                TaskModel updatedTask = await _resourcePlanner.Plan(inputModel);
                return Ok(updatedTask);
            }
            catch (Orchestrator.ApiException<RedisInterface.ApiResponse> apiEx)
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
