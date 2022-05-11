namespace Middleware.Common.Models;

/// <summary>
/// Represents the Response from the API
/// </summary>
public class ApiResponse
{
    /// <summary>
    /// Status code returned by the API
    /// </summary>
    public int StatusCode { get; private set; }
    /// <summary>
    /// Message with the description of the problem from the API
    /// </summary>
    public string Message { get; private set; }
    
    public ApiResponse(int statusCode, string message)
    {
        StatusCode = statusCode;
        Message = message;
    }
}