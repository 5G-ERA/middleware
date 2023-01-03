using System;
using System.Net;
using System.Text.Json.Serialization;

namespace Middleware.Common.Responses;

/// <summary>
/// Represents the Response from the API
/// </summary>
public class ApiResponse
{
    /// <summary>
    /// Status code returned by the API
    /// </summary>
    [JsonPropertyName("statusCode")]
    public int StatusCode { get; protected set; }
    /// <summary>
    /// Message with the description of the problem from the API
    /// </summary>
    [JsonPropertyName("message")]
    public string Message { get; protected set; }
    /// <summary>
    /// Timestamp of response being created in UTC
    /// </summary>
    [JsonPropertyName("timeStamp")]
    public DateTime TimeStamp { get; }
    /// <summary>
    /// Is error response
    /// </summary>
    [JsonPropertyName("isError")]
    public bool IsError => StatusCode >= (int)HttpStatusCode.BadRequest;

    public ApiResponse()
    {
        TimeStamp = DateTime.UtcNow;
    }

    public ApiResponse(int statusCode, string message) : this()
    {
        StatusCode = statusCode;
        Message = message;
    }
}

public class ApiResponse<T> : ApiResponse
{
    [JsonPropertyName("data")]
    public T Data { get; private set; }
    public ApiResponse(T data)
    {
        Data = data;
        StatusCode = (int)HttpStatusCode.OK;
    }

    public ApiResponse(T data, int statusCode, string message) : base(statusCode, message)
    {
        Data = data;
    }
}
