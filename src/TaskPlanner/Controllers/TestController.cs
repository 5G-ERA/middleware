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



    [HttpPost]
    [Route("send", Name = "send")]
    public IActionResult Send()
    {
        return Ok("websocket test");
    }
}