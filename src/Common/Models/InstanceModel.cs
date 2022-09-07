using System.ComponentModel.DataAnnotations;
using System.Diagnostics.CodeAnalysis;
using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class InstanceModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("ServiceInstanceId")]
    public Guid ServiceInstanceId { get; set; }

    [JsonPropertyName("ServiceType")]
    public string ServiceType { get; set; }

    [JsonPropertyName("IsReusable")]
    public bool? IsReusable { get; set; }

    [JsonPropertyName("DesiredStatus")]
    public string DesiredStatus { get; set; }

    [JsonPropertyName("ServiceUrl")]
    public Uri ServiceUrl { get; set; }

    [JsonPropertyName("RosTopicsPub")]
    public List<RosTopicModel> RosTopicsPub { get; set; }

    [JsonPropertyName("RosTopicsSub")]
    public List<RosTopicModel> RosTopicsSub { get; set; }

    [JsonPropertyName("Tags")]
    public List<string> Tags { get; set; }

    [JsonPropertyName("ServiceStatus")]
    public string ServiceStatus { get; set; } //updated every 10 sec

    [JsonPropertyName("ContainerImage")]
    [JsonIgnore]
    public ContainerImageModel ContainerImage { get; set; }
}