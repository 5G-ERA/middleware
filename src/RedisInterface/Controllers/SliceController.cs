using System.Net;
using Microsoft.AspNetCore.Mvc;
using Middleware.Common.Responses;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class SliceController : ControllerBase
{
    [HttpPost(Name = "SliceRegister")]
    [ProducesResponseType(typeof(void), (int)HttpStatusCode.Created)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.BadRequest)]
    [ProducesResponseType(typeof(ApiResponse), (int)HttpStatusCode.InternalServerError)]
    public async Task<IActionResult> RegisterSlice([FromBody] RegisterSlicesRequest request)
    {
        try
        {
            return StatusCode((int)HttpStatusCode.Created);
        }
        catch (Exception ex)
        {
            Console.WriteLine(ex);
            throw;
        }
    }
}