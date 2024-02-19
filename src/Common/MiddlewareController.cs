using System.Net;
using Microsoft.AspNetCore.Mvc;

namespace Middleware.Common;

public class MiddlewareController : Controller
{
    protected IActionResult ErrorMessageResponse(HttpStatusCode code)
    {
        return GetResponse(code, null);
    }
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
        return GetResponse(code, error);
    }

    protected IActionResult ErrorMessageResponse(HttpStatusCode code, IDictionary<string, string[]> errors)
    {
        var retVal = new ValidationProblemDetails()
        {
            Status = (int)code,
            Extensions =
            {
                ["traceId"] = HttpContext.TraceIdentifier
            }
        };
        if (errors.Any())
        {
            foreach (var error in errors)
            {
                retVal.Errors.Add(error);
            }
        }
        return GetResponse(code, retVal);
    }
    
    private IActionResult GetResponse(HttpStatusCode code, object? data)
    {
        var resp = code switch
        {
            HttpStatusCode.NotFound => NotFound(data),
            HttpStatusCode.BadRequest => BadRequest(data),
            HttpStatusCode.InternalServerError => StatusCode(500, data),
            _ => throw new ArgumentOutOfRangeException(nameof(code), code, null)
        };
        return resp;
    }
}