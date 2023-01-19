﻿using System.Text.Json.Serialization;

namespace Middleware.Models.Domain;

public class ContainerImageModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }

    [JsonPropertyName("K8SDeployment")]
    public string K8SDeployment { get; set; }

    [JsonPropertyName("K8SService")]
    public string K8SService { get; set; }
}