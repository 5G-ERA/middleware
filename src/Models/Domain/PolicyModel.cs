using System.Text.Json.Serialization;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class PolicyModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")] // Compulsory field
    public override string Name { get; set; }

    [JsonPropertyName("Type")] // Compulsory field
    public string Type { get; set; }

    [JsonPropertyName("Timestamp")]
    public DateTime Timestamp { get; set; }

    [JsonPropertyName("IsActive")]
    public bool? IsActive { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }

    [JsonPropertyName("IsExclusiveWithinType")]
    public int IsExclusiveWithinType { get; set; } 


    /// <summary>
    /// Onboarding validation of the policy data object.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsValid()
    {
        var policyTypesEnum = Enum.GetNames(typeof(PolicyTypesEnum)).ToList();

        if (string.IsNullOrEmpty(Name.ToString())) return false;
        if (string.IsNullOrEmpty(IsActive.ToString())) return false;
        if (string.IsNullOrEmpty(Description.ToString())) return false;
        if (string.IsNullOrEmpty(IsExclusiveWithinType.ToString())) return false;
        if (!policyTypesEnum.Contains(Type)) return false;


        return true;
    }
    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new PolicyDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = domain.Type,
            Timestamp = domain.Timestamp,
            IsActive = domain.IsActive,
            Description = domain.Description,
            IsExclusiveWithinType = domain.IsExclusiveWithinType
        };
    }
}