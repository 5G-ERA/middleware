using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Services.Abstract;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.CentralApi.Contracts.Responses;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class RobotController : ControllerBase
{
    private readonly ILogger _logger;
    private readonly IRobotService _robotService;

    public RobotController(ILogger<RobotController> logger,
        IRobotService robotService)
    {
        _logger = logger ?? throw new ArgumentNullException(nameof(logger));
        _robotService = robotService ?? throw new ArgumentNullException(nameof(robotService));
    }

    /// <summary>
    ///     Creates a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("heartbeat", Name = "AddRobotToLocationAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<List<string>>> RobotToLocationAddRelation([FromBody] RelationToLocationRequest model)
    {
        if (model == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            List<string> errorss = await _robotService.CreateRelation(model);
            return errorss;
        } catch (ArgumentNullException ex1)
        {
            _logger.LogError(ex1, "Adding relation/relations did not succeed, robot was not found:");
            return StatusCode((int)HttpStatusCode.BadRequest,
                new ApiResponse((int)HttpStatusCode.BadRequest, "Adding relation/relations did not succeed, robot was not found."));
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Deletes a new relation between two models
    /// </summary>
    /// <param name="model"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("heartbeat", Name = "DeleteRobotToLocationRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult<List<string>>> DeleteRobotToLocationRelation([FromBody] RelationToLocationRequest model)
    {
        if (model == null)
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Parameters were not specified."));
        try
        {
            List<string> returnedErrorss = await _robotService.DeleteRelation(model);
            return returnedErrorss;
        }
        catch (ArgumentNullException ex)
        {
            _logger.LogError(ex, "Deleting relation/relations did not succeed, robot was not found:");
            return StatusCode((int)HttpStatusCode.BadRequest,
                new ApiResponse((int)HttpStatusCode.BadRequest, "Deleting relation/relations did not succeed, robot was not found."));
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}
