using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Mappings;
using Middleware.CentralApi.Services;
using Middleware.Common.Responses;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class LocationsController : Controller
{
    private readonly ILocationRepository _locationRepository;
    private readonly ILocationService _locationService;

    public LocationsController(ILocationService locationService, ILocationRepository locationRepository)
    {
        _locationService = locationService;
        _locationRepository = locationRepository;
    }

    // GET
    [HttpGet]
    [ProducesResponseType(typeof(LocationsResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> GetAvailableLocations(string organization)
    {
        if (string.IsNullOrEmpty(organization))
            return BadRequest(new ApiResponse((int)HttpStatusCode.BadRequest, "Organization was not specified"));

        var result = await _locationService.GetAvailableLocations(organization);

        return result.Match<IActionResult>(
            locations => { return Ok(locations.ToLocationsResponse()); },
            _ => { return NotFound(); });
    }

    // GET
    [HttpGet]
    [Route("status/{id}", Name = "GetOnlineStatusById")]
    [ProducesResponseType(typeof(CloudEdgeStatusResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> GetStatus(string? location, Guid id)
    {
        if (location == null)
            return BadRequest("location parameter not specified");
        try
        {
            var response = await _locationRepository.GetCloudOnlineStatusLastUpdatedTimeAsync(id);
            return Ok(response);
        }
        catch (ArgumentException argEx)
        {
            return NotFound(argEx.Message);
        }
        catch (Exception ex)
        {
            return BadRequest(ex.Message);
        }
    }

    // POST
    [HttpPost]
    [Route("status/{id}", Name = "SetOnlineStatusById")]
    [ProducesResponseType((int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public async Task<IActionResult> SetStatus([FromBody] CloudEdgeStatusRequest? request, Guid id)
    {
        if (request == null)
            return BadRequest();
        try
        {
            await _locationRepository.SetCloudOnlineStatusAsync(id, request.IsOnline);

            return Ok();
        }
        catch (ArgumentException argEx)
        {
            return NotFound(argEx.Message);
        }
        catch (Exception ex)
        {
            return BadRequest(ex.Message);
        }
    }
}