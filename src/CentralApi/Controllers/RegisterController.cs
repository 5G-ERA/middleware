using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Mappings;
using Middleware.CentralApi.Services;
using Middleware.Common;
using Middleware.Common.Responses;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class RegisterController : MiddlewareController
{
    private readonly ILocationService _locationService;

    public RegisterController(ILocationService locationService)
    {
        _locationService = locationService;
    }

    // GET
    [HttpPost]
    [ProducesResponseType(typeof(LocationResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> Register([FromBody] RegisterRequest request)
    {
        var location = request.ToLocation();

        var result = await _locationService.RegisterLocation(location);

        return result.Match<IActionResult>(
            loc => Ok(loc.ToLocationResponse()),
            _ => ErrorMessageResponse(HttpStatusCode.NotFound, nameof(location),
                "The specified location does not exist"));
    }
}