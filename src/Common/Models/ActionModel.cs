﻿using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class ActionModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }
    [JsonPropertyName("Order")]

    public string ActionFamily { get; set; }
    [JsonPropertyName("ActionFamily")]
    public int Order { get; set; }

    [JsonPropertyName("Placement")]
    public string Placement { get; set; }

    [JsonPropertyName("ActionPriority")]
    public string ActionPriority { get; set; }

    [JsonPropertyName("Services")]
    public List<InstanceModel> Services { get; set; }
}