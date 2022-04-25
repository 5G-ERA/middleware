﻿using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class PolicyModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("PolicyName")]
    public override string Name { get; set; }

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool? IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }
}