using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
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

            await _sliceService.ReRegisterSlicesAsync(slices, location);

            return StatusCode((int)HttpStatusCode.Created);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpGet]
    [ProducesResponseType(typeof(GetSlicesResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetAllSlices()
    {
        try
        {
            var slices = await _sliceService.GetAllSlicesAsync();

            var response = slices.ToSlicesResponse();

            return Ok(response);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpDelete("{id}")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> DeleteSliceById(Guid id)
    {
        try
        {
            await _sliceService.DeleteById(id);


            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpGet]
    [Route("{id}", Name = "SliceGetById")]
    [ProducesResponseType(typeof(SliceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> SliceGetById(Guid id)
    {
        var statusCode = HttpStatusCode.InternalServerError;
        try
        {
            var slice = await _sliceService.GetByIdAsync(id);
            if (slice is null)
            {
                statusCode = HttpStatusCode.NotFound;
                return StatusCode((int)statusCode, new ApiResponse((int)statusCode, "Specified Slice was not found."));
            }

            var response = slice.ToSliceResponse();

            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return StatusCode((int)statusCode,
                new ApiResponse((int)statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    [HttpGet("sliceId/{id}", Name = "GetBySliceIdAsync")]
    [ProducesResponseType(typeof(SliceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetBySliceIdAsync(string id)
    {
        var statusCode = HttpStatusCode.InternalServerError;
        try
        {
            var slice = await _sliceService.GetBySliceIdAsync(id);
            if (slice is null)
            {
                statusCode = HttpStatusCode.NotFound;
                return StatusCode((int)statusCode, new ApiResponse((int)statusCode, "Specified Slice was not found."));
            }

            var response = slice.ToSliceResponse();

            return Ok(response);
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "An error occurred:");
            return StatusCode((int)statusCode,
                new ApiResponse((int)statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Retrieves a single relation by name
    /// </summary>
    /// <param name="id"></param>
    /// <param name="name"></param>
    /// <param name="direction"></param>
    /// <returns></returns>
    [HttpGet]
    [Route("relation/{name}", Name = "SliceGetRelationByName")]
    [ProducesResponseType(typeof(List<RelationModel>), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetRelationAsync(Guid id, string name, string direction = null)
    {
        direction ??= RelationDirection.Outgoing.ToString();

        if (Enum.TryParse(direction, true, out RelationDirection directionEnum) == false)
        {
            var availableValues = Enum.GetValues(typeof(RelationDirection));
            return StatusCode(400,
                new ApiResponse(400,
                    $"Specified parameter value direction='{direction}' is incorrect available values are: {string.Join(", ", availableValues)}"));
        }


        try
        {
            var relations = await _sliceService.GetRelationAsync(id, name, directionEnum);
            if (!relations.Any())
                return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Relations were not found."));
            return Ok(relations);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Add new embb slice
    /// </summary>
    /// <param name="embbSlice"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("embb", Name = "SliceAddEmbb")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.Created)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> SliceAddEmbb([FromBody] SliceRequest embbSlice)
    {
        if (embbSlice == null) return BadRequest("Parameters were not specified correctly.");
        try
        {
            await _sliceService.SliceAddAsync(embbSlice.ToSlice());

            return StatusCode((int)HttpStatusCode.Created);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Add new urllc slice
    /// </summary>
    /// <param name="urllcSlice"></param>
    /// <returns></returns>
    [HttpPost]
    [Route("urllc", Name = "SliceAddUrllc")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.Created)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> SliceAddUrllc([FromBody] SliceRequest urllcSlice)
    {
        if (urllcSlice == null) return BadRequest("Parameters were not specified correctly.");
        try
        {
            await _sliceService.SliceAddAsync(urllcSlice.ToSlice());

            return StatusCode((int)HttpStatusCode.Created);
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpPut]
    [Route("embb", Name = "SliceUpdateEmbb")]
    [ProducesResponseType(typeof(SliceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> SliceUpdateEmbb([FromBody] SliceRequest embbSlice)
    {
        var existingSlice = await _sliceService.GetBySliceIdAsync(embbSlice.SliceId);
        if (existingSlice is null)
            return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Slice to be updated was not found."));
        try
        {
            await _sliceService.SliceUpdateAsync(embbSlice.ToSlice(existingSlice.Id));
            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    [HttpPut]
    [Route("urllc", Name = "SliceUpdateUrllc")]
    [ProducesResponseType(typeof(SliceResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> SliceUpdateUrllc([FromBody] SliceRequest urllcSlice)
    {
        var existingSlice = await _sliceService.GetBySliceIdAsync(urllcSlice.SliceId);
        if (existingSlice is null)
            return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "Slice to be updated was not found."));
        try
        {
            await _sliceService.SliceUpdateAsync(urllcSlice.ToSlice(existingSlice.Id));
            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    [HttpDelete]
    [Route("embb", Name = "SliceDeleteEmbb")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult> SliceDeleteEmbb(string id)
    {
        try
        {
            await _sliceService.DeleteBySliceId(id);
            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }


    [HttpDelete]
    [Route("urllc", Name = "SliceDeleteUrllc")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<ActionResult> SliceDeleteUrllc(string id)
    {
        try
        {
            await _sliceService.DeleteBySliceId(id);
            return Ok();
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}