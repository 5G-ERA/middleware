﻿using System.Text.Json.Serialization;
using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public class ActionModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    [JsonPropertyName("Name")]
    public override string Name { get; set; } = default!;

    [JsonPropertyName("Tags")]
    public List<string>? Tags { get; set; }

    [JsonPropertyName("Order")]
    public int Order { get; set; }

    [JsonPropertyName("Placement")]
    public string? Placement { get; set; }

    [JsonPropertyName("PlacementType")]
    public string? PlacementType { get; set; } // Either edge or cloud. 

    [JsonPropertyName("ActionPriority")]
    public string? ActionPriority { get; set; } = default!;

    [JsonPropertyName("ActionStatus")]
    public string? ActionStatus { get; set; }

    [JsonPropertyName("Services")]
    public List<InstanceModel> Services { get; set; } = new();

    [JsonPropertyName("MinimumRam")]
    public long? MinimumRam { get; set; }

    [JsonPropertyName("MinimumNumCores")]
    public int? MinimumNumCores { get; set; }

    /// <summary>
    ///     Network Slice associated for the specific action
    /// </summary>
    public string? NetworkSlice { get; set; }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new ActionDto
        {
            Id = domain.Id.ToString(),
            ActionPriority = domain.ActionPriority,
            Name = domain.Name,
            HardwareRequirements = new()
            {
                MinimumRam = domain.MinimumRam,
                MinimumNumCores = domain.MinimumNumCores
            },
            Tags = domain.Tags
        };
    }
}