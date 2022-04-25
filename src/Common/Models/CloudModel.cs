﻿using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class CloudModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("CloudName")]
    public override string Name { get; set; }

    [JsonPropertyName("CloudStatus")]
    public string CloudStatus { get; set; }

    [JsonPropertyName("CloudIp")]
    public Uri CloudIp { get; set; }

    
}