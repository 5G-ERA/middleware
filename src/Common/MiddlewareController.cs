using System.Net;
using Microsoft.AspNetCore.Mvc;

namespace Middleware.Common;

public class MiddlewareController : Controller
{
    protected IActionResult ErrorMessageResponse(HttpStatusCode code, string property, string message)
    {
        var error = new ValidationProblemDetails()
        {
            Status = (int)code,
            Extensions =
            {
                ["traceId"] = HttpContext.TraceIdentifier
            }
        };
        error.Errors.Add(new(property, new[] { message }));
        var resp = code switch
        {
            HttpStatusCode.NotFound => NotFound(error),
            HttpStatusCode.BadRequest => BadRequest(error),
            HttpStatusCode.InternalServerError => StatusCode(500, error),
            _ => throw new ArgumentOutOfRangeException(nameof(code), code, null)
        };
        return resp;
    }
}