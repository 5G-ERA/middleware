﻿using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using System.Text.Json.Serialization;

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

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new ContainerImageDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Timestamp = domain.Timestamp,
            Description = domain.Description,
            K8SDeployment = domain.K8SDeployment,
            K8SService = domain.K8SService
        };
    }
}