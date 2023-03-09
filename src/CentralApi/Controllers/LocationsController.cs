using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Mappings;
using Middleware.CentralApi.Services;
using Middleware.Common.Responses;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class LocationsController : Controller
{
    private readonly ILocationService _locationService;

    public LocationsController(ILocationService locationService)
    {
        _locationService = locationService;
    }

    // GET
    [HttpGet]
    [ProducesResponseType(typeof(LocationsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> GetAvailableLocations(string organization)
    {
        if (string.IsNullOrEmpty(organization))
        {
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, $"Organization was not specified"));
        }

        var result = await _locationService.GetAvailableLocations(organization);

        return result.Match<IActionResult>(
            locations => { return Ok(locations.ToLocationsResponse()); },
            _ => { return NotFound(); });
    }
}