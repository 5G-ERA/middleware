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
public class ResourceController : MiddlewareController
{
    private readonly ILogger _logger;
    private readonly IMapper _mapper;
    private readonly IResourcePlanner _resourcePlanner;

    public ResourceController(IResourcePlanner resourcePlanner, IMapper mapper, ILogger<ResourceController> logger)
    {
        _resourcePlanner = resourcePlanner;
        _mapper = mapper;
        _logger = logger;
    }


    /// <summary>
    ///     Return an updated taskModel with the resource specs.
    /// </summary>
    /// <param name="resource"></param>
    /// <returns></returns>
    [HttpPost]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetResource([FromBody] ResourceInput resource)
    {
        try
        {
            var updatedTask = await _resourcePlanner.Plan(resource.Task, resource.Robot);

            return Ok(updatedTask);
        }
        catch (ApiException<Orchestrator.ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"There was an error while collecting the resources: {ex.Message}");
        }
    }

    /// <summary>
    ///     Return an updated taskModel with the resource specs.
    /// </summary>
    /// <param name="resource"></param>
    /// <returns></returns>
    [HttpPost("semantic", Name = "GetResourcePlan")]
    [ProducesResponseType(typeof(TaskModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetSemanticResourcePlan([FromBody] ResourceInput resource)
    {
        try
        {
            var updatedTask = await _resourcePlanner.SemanticPlan(resource.Task, resource.Robot);

            return Ok(updatedTask);
        }
        catch (ApiException<Orchestrator.ApiResponse> apiEx)
        {
            return StatusCode(apiEx.StatusCode, _mapper.Map<ApiResponse>(apiEx.Result));
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"There was an error while collecting the resources: {ex.Message}");
        }
    }
}