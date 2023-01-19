using System.Text.Json.Serialization;

namespace Middleware.Common.Responses;

public class PagedResponse<T> : ApiResponse<T>
{
    [JsonPropertyName("pageNumber")]
    public int PageNumber { get; set; }
    [JsonPropertyName("pageSize")]
    public int PageSize { get; set; }
    [JsonPropertyName("firstPage")]
    public Uri FirstPage { get; set; }
    [JsonPropertyName("lastPage")]
    public Uri LastPage { get; set; }
    [JsonPropertyName("totalPages")]
    public int TotalPages { get; set; }
    [JsonPropertyName("firstPage")]
    public int TotalRecords { get; set; }
    [JsonPropertyName("nextPage")]
    public Uri NextPage { get; set; }
    [JsonPropertyName("previousPage")]
    public Uri PreviousPage { get; set; }

    public PagedResponse(T data, int page, int pageSize) : base(data)
    {

    }

    public PagedResponse(T data, int statusCode, string message) : base(data, statusCode, message)
    {

    }
}
