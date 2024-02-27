using System.Net;
using AutoMapper;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common;
using Middleware.ResourcePlanner.Models;
using Middleware.ResourcePlanner.Orchestrator;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using TaskModel = Middleware.Models.Domain.TaskModel;

namespace Middleware.ResourcePlanner.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class ReplanResourceController : MiddlewareController
{
    private readonly IMapper _mapper;
    private readonly IResourcePlanner _resourcePlanner;


    /// <summary>
    ///     Constructor
    /// </summary>
    /// <param name="resourcePlanner"></param>
    /// <param name="mapper"></param>
    public ReplanResourceController(IResourcePlanner resourcePlanner,
        IMapper mapper)
    {
        _resourcePlanner = resourcePlanner;
        _mapper = mapper;
    }

    [HttpPost(Name = "GetResourceRePlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<TaskModel>> GetResourceRePlan([FromBody] ResourceReplanInputModel resource)
    {
        try
        {
            var updatedTask =
                await _resourcePlanner.RePlan(resource.Task, resource.oldTask, resource.Robot, resource.FullReplan);

            return Ok(updatedTask);
        }
        catch (ApiException<ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (ApiException<Orchestrator.ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            return StatusCode(statusCode,
                new ApiResponse(statusCode, $"There was an error while collecting the resources: {ex.Message}"));
        }
    }
}