using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class SliceController : ControllerBase
{
    private readonly ILogger<SliceController> _logger;
    private readonly ISliceService _sliceService;

    public SliceController(ISliceService sliceService, ILogger<SliceController> logger)
    {
        _sliceService = sliceService;
        _logger = logger;
    }

    [HttpPost(Name = "SliceRegister")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.Created)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> RegisterSlice([FromBody] RegisterSlicesRequest request)
    {
        try
        {
            var location = request.Location?.ToLocation();
            var slices = request.ToSliceList();

            await _sliceService.ReRegisterSlices(slices, location);

            return StatusCode((int)HttpStatusCode.Created);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}