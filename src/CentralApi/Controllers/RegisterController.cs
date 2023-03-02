using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Contracts.Requests;
using Middleware.Common.Responses;

namespace Middleware.CentralApi.Controllers;

[ApiController]
[Route("api/v1/[controller]")]
public class RegisterController : Controller
{
    // GET
    [HttpPost]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.OK)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.NotFound)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    public IActionResult Register([FromBody] RegisterRequest request)
    {
        //TODO: 
        return Ok();
    }
}