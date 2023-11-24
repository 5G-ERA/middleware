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
public class LocationsController : ControllerBase
{
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;
    private readonly ILocationService _locationService;

    public LocationsController(ILocationService locationService, ICloudRepository cloudRepository,
        IEdgeRepository edgeRepository)
    {
        _locationService = locationService;
        _cloudRepository = cloudRepository;
        _edgeRepository = edgeRepository;
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
    public async Task<IActionResult> GetStatus(string location, Guid id)
    {
        if (location == null)
            throw new ArgumentNullException(nameof(location));
        try
        {
            CloudEdgeStatusResponse response;

            if (location.ToLower() == "cloud")
                response = await _cloudRepository.GetCloudOnlineStatusLastUpdatedTimeAsync(id);
            else if (location.ToLower() == "edge")
                response = await _edgeRepository.GetEdgeOnlineStatusLastUpdatedTimeAsync(id);
            else
                return BadRequest();
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
    public async Task<IActionResult> SetStatus([FromBody] CloudEdgeStatusRequest request, Guid id)
    {
        if (request == null) return BadRequest();
        try
        {
            if (request.Type.ToLower() == "cloud")
                await _cloudRepository.SetCloudOnlineStatusAsync(id, request.IsOnline);
            else if (request.Type.ToLower() == "edge")
                await _edgeRepository.SetEdgeOnlineStatusAsync(id, request.IsOnline);
            else
                return BadRequest();
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