using System.Text.Json.Serialization;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class PolicyModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    [JsonPropertyName("Name")]
    public override string Name { get; set; } = default!;

    /// <summary>
    /// Type of the policy
    /// </summary>
    [JsonPropertyName("Type")]
    public PolicyType Type { get; set; } = PolicyType.None;

    /// <summary>
    /// Defines the scope of the policy, if it is system-wide or resource specific
    /// </summary>
    public PolicyScope Scope { get; set; } = PolicyScope.System;

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; } = default!;

    [JsonPropertyName("IsExclusiveWithinType")]
    public int IsExclusiveWithinType { get; set; }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new PolicyDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = domain.Type.ToString(),
            Timestamp = domain.Timestamp,
            IsActive = domain.IsActive,
            Description = domain.Description,
            IsExclusiveWithinType = domain.IsExclusiveWithinType
        };
    }
}