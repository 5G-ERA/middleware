﻿using System.Text.Json.Serialization;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

internal class RosServiceContainer
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
    [JsonPropertyName("type")]
    public string Type { get; set; } = default!;
    [JsonPropertyName("qos")]
    public QosContainer Qos { get; set; }

    public static RosServiceContainer FromRosServiceModel(RosServiceModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type,
            Qos = QosContainer.FromQosModel(x.Qos)
        };
    }
}