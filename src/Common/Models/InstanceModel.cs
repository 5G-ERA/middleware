using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class InstanceModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("ImageName")]
    public string ImageName { get; set; }

    [JsonPropertyName("ServiceInstanceId")]
    public Guid ServiceInstanceId { get; set; }

    [JsonPropertyName("ServiceType")]
    public string ServiceType { get; set; }

    [JsonPropertyName("IsReusable")]
    public bool IsReusable { get; set; }

    [JsonPropertyName("DesiredStatus")]
    public string DesiredStatus { get; set; }

    [JsonPropertyName("ServiceUrl")]
    public Uri ServiceUrl { get; set; }

    [JsonPropertyName("ServiceStatus")]
    public string ServiceStatus { get; set; }
}