using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Responses;

public class LocationStatusResponse
{
    [JsonPropertyName("lastUpdatedTime")]
    public DateTime? LastUpdatedTime { get; set; }

    [JsonPropertyName("name")]
    public string Name { get; set; }

    [JsonPropertyName("isOnline")]
    public bool IsOnline { get; set; }

    [JsonPropertyName("isBusy")]
    public bool IsBusy { get; set; }

    [JsonPropertyName("numberOfRunningContainers")]
    public int NumberOfRunningContainers { get; set; }

    [JsonPropertyName("status")]
    public string Status { get; set; }
    public LocationStatusResponse()
    {

    }

    public LocationStatusResponse(string name, DateTime lastUpdatedTime, string status, bool isOnline, bool isBusy, int noOfContainers)
    {
        Name = name;
        LastUpdatedTime = lastUpdatedTime;
        Status = status;
        IsOnline = isOnline;
        IsBusy = isBusy;
        NumberOfRunningContainers = noOfContainers;
    }
}
