using Microsoft.AspNetCore.Mvc;

namespace Middleware.CentralApi.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class HealthController : ControllerBase
{
    [HttpGet]
    public IActionResult Get()
    {
        return Ok();
    }
}