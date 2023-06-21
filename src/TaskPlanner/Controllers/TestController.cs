using Microsoft.AspNetCore.Mvc;

namespace Middleware.TaskPlanner.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class TestController : ControllerBase
{
    [HttpGet]
    [Route("hello", Name = "Hello")]
    public IActionResult Hello()
    {
        return Ok("hello there");
    }
}