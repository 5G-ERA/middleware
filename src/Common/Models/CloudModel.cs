using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class CloudModel
{
    [JsonPropertyName("CloudID")]
    public Guid CloudId { get; set; }

    [JsonPropertyName("CloudStatus")]
    public string CloudStatus { get; set; }

    [JsonPropertyName("CloudIp")]
    public Uri CloudIp { get; set; }
}