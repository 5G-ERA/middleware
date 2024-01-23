﻿using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class ConfigController : ControllerBase
{
    private readonly ILogger<ConfigController> _logger;
    private readonly ISystemConfigService _systemConfigService;

    public ConfigController(ISystemConfigService systemConfigService, ILogger<ConfigController> logger)
    {
        _systemConfigService = systemConfigService;
        _logger = logger;
    }

    /// <summary>
    ///     Get config
    /// </summary>
    /// <returns> the list of ActionModel entities </returns>
    [HttpGet(Name = "ConfigGet")]
    [ProducesResponseType(typeof(SystemConfigResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> GetConfigAsync()
    {
        try
        {
            var cfg = await _systemConfigService.GetConfig();

            if (cfg is null) return NotFound(new ApiResponse((int)HttpStatusCode.NotFound, "System Config not found."));

            return Ok(cfg.ToSystemConfigResponse());
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }

    /// <summary>
    ///     Update config
    /// </summary>
    /// <returns> the list of ActionModel entities </returns>
    [HttpPut(Name = "ConfigUpdate")]
    [ProducesResponseType(typeof(SystemConfigResponse), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> UpdateConfigAsync([FromBody] SystemConfigRequest request)
    {
        try
        {
            var cfg = request.ToSystemConfig();
            var (resultCfg, err) = await _systemConfigService.UpdateConfig(cfg);

            if (string.IsNullOrEmpty(err) == false)
                return BadRequest(new ApiResponse((int)HttpStatusCode.NotFound, err));


            return Ok(resultCfg.ToSystemConfigResponse());
        }
        catch (Exception ex)
        {
            var statusCode = (int)HttpStatusCode.InternalServerError;
            _logger.LogError(ex, "An error occurred:");
            return StatusCode(statusCode, new ApiResponse(statusCode, $"An error has occurred: {ex.Message}"));
        }
    }
}