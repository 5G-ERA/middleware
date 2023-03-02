using Microsoft.AspNetCore.Mvc;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class LocationsController : Controller
{
    // GET
    public IActionResult Index()
    {
        return Ok();
    }
}