﻿using System.Text.Json.Serialization;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain.Ros;

public class RosActionModel
{
    public string Name { get; set; } = default!;
 
    public string Type { get; set; } = default!;

    public RosAction ToDto()
    {
        var domain = this;
        return new()
        {
            Name = domain.Name,
            Type = domain.Type
        };
    }
}