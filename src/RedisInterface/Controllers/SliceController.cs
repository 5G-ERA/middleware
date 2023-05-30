﻿using System.Net;
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
internal class SliceController : ControllerBase
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

    [HttpPost]
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
}