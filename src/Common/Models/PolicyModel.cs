﻿using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class PolicyModel
{
    [JsonPropertyName("Policy_Id")]
    public Guid Id { get; set; }

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }

    [JsonPropertyName("PolicyName")]
    public string PolicyName { get; set; }
}