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
    ///     Type of the policy
    /// </summary>
    [JsonPropertyName("Type")]
    public PolicyType Type { get; set; } = PolicyType.None;

    /// <summary>
    ///     Defines the scope of the policy, if it is system-wide or resource specific
    /// </summary>
    public PolicyScope Scope { get; set; } = PolicyScope.System;

    /// <summary>
    ///     Defines the priority the policy has
    /// </summary>
    public Priority Priority { get; set; } = Priority.Normal;

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; } = default!;

    [JsonPropertyName("IsExclusiveWithinType")]
    public int IsExclusiveWithinType { get; set; }

    /// <summary>
    ///     Onboarding validation of the policy data object.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsValid()
    {
        var policyTypesEnum = Enum.GetNames(typeof(PolicyType)).ToList();

        if (string.IsNullOrEmpty(Name)) return false;
        if (string.IsNullOrEmpty(IsActive.ToString())) return false;
        if (string.IsNullOrEmpty(Description)) return false;
        if (string.IsNullOrEmpty(IsExclusiveWithinType.ToString())) return false;
        if (!policyTypesEnum.Contains(Type.ToString())) return false;

        return true;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new PolicyDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = domain.Type.ToString(),
            Timestamp = domain.Timestamp,
            IsActive = domain.IsActive,
            Description = domain.Description,
            IsExclusiveWithinType = domain.IsExclusiveWithinType,
            Priority = domain.Priority.ToString()
        };
    }
}