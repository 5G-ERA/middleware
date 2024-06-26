﻿using System.Net;
using Microsoft.AspNetCore.Mvc;

namespace Middleware.Orchestrator.Controllers;

[Route("api/[controller]")]
[ApiController]
public class HealthController : ControllerBase
{
    private readonly HttpClient _client;

    public HealthController(IHttpClientFactory factory)
    {
        _client = factory.CreateClient("healthCheckClient");
    }

    [HttpGet(Name = "OrchestratorHealthCheck")]
    [ProducesResponseType((int)HttpStatusCode.OK)]
    public IActionResult Get()
    {
        return Ok();
    }

    [HttpGet]
    [Route("spec", Name = "GetOrchestratorSpec")]
    [ProducesResponseType(typeof(string), (int)HttpStatusCode.OK)]
    public async Task<IActionResult> GetSpec()
    {
        var path = string.Empty;
#if DEBUG
        var addresses = Environment.GetEnvironmentVariable("ASPNETCORE_URLS")?.Split(";");
        if (addresses == null && Uri.IsWellFormedUriString(addresses![0], UriKind.RelativeOrAbsolute))
            return NotFound();

        _client.BaseAddress = new(addresses[0]);

        var bytes = await _client.GetByteArrayAsync("/swagger/v1/swagger.json");
        path = Path.Combine(Directory.GetCurrentDirectory(), "OrchestratorSpec.json");
        await System.IO.File.WriteAllBytesAsync(path, bytes);
#endif
        return Ok(path);
    }
}