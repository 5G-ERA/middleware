using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Services.Abstract;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.Common;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class RobotController : MiddlewareController
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
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("heartbeat", Name = "AddRobotToLocationAddRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> RobotToLocationAddRelation([FromBody] RelationToLocationRequest? request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");
        try
        {
            List<string> errors = await _robotService.CreateRelation(request);
            return Ok(errors);
        } catch (ArgumentNullException ex1)
        {
            _logger.LogError(ex1, "Adding relation/relations did not succeed, robot was not found:");
            return ErrorMessageResponse(HttpStatusCode.NotFound, "robot", "Specified robot was not found");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }

    /// <summary>
    ///     Deletes a new relation between two models
    /// </summary>
    /// <param name="request"></param>
    /// <returns></returns>
    [HttpDelete]
    [Route("heartbeat", Name = "DeleteRobotToLocationRelation")]
    [ProducesResponseType(typeof(RelationModel), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteRobotToLocationRelation([FromBody] RelationToLocationRequest? request)
    {
        if (request == null)
            return ErrorMessageResponse(HttpStatusCode.BadRequest, nameof(request), "Request body was not specified");
            
        try
        {
            var returnedErrors = await _robotService.DeleteRelation(request);
            return Ok(returnedErrors);
        }
        catch (ArgumentNullException ex)
        {
            _logger.LogError(ex, "Deleting relation/relations did not succeed, robot was not found:");
            return ErrorMessageResponse(HttpStatusCode.NotFound, "robot", "Specified robot was not found");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return ErrorMessageResponse(HttpStatusCode.InternalServerError, "system",
                $"An error has occurred: {ex.Message}");
        }
    }
}
