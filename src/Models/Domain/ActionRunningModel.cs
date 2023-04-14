using System.Runtime.InteropServices;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using System.Text.Json.Serialization;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class ActionRunningModel : BaseModel
{
    /// <summary>
    /// Unique identifier of a running action
    /// </summary>
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    /// <summary>
    /// Identifier of the action definition
    /// </summary>
    [JsonPropertyName("ActionId")]
    public Guid ActionId { get; set; } 

    /// <summary>
    /// Identifier of the ActionPlan the running action is associated with
    /// </summary>
    [JsonPropertyName("ActionPlanId")]
    public Guid ActionPlanId { get; set; }

    /// <summary>
    /// Name of the running action
    /// </summary>
    [JsonPropertyName("Name")]
    public override string Name { get; set; } = default!;

    [JsonPropertyName("Tags")]
    public List<string>? Tags { get; set; }

    [JsonPropertyName("Order")]
    public int Order { get; set; }

    [JsonPropertyName("Placement")]
    public string Placement { get; set; } = default!;

    [JsonPropertyName("PlacementType")]
    public LocationType PlacementType { get; set; } 

    [JsonPropertyName("ActionPriority")]
    public string? ActionPriority { get; set; }

    [JsonPropertyName("ActionStatus")]
    public string? ActionStatus { get; set; }

    [JsonPropertyName("Services")]
    //[JsonIgnore]
    public List<InstanceRunningModel>? Services { get; set; }

    [JsonPropertyName("MinimumRam")]
    public long? MinimumRam { get; set; }

    [JsonPropertyName("MinimumNumCores")]
    public int? MinimumNumCores { get; set; }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new ActionRunningDto()
        {
            Id = domain.Id.ToString(),
            ActionPriority = domain.ActionPriority,
            Name = domain.Name,
            ActionId = domain.ActionId.ToString(),
            ActionPlanId = domain.ActionPlanId.ToString(),
            HardwareRequirements = new HardwareRequirements()
            {
                MinimumRam = domain.MinimumRam,
                MinimumNumCores = domain.MinimumNumCores
            },
            Tags = domain.Tags,
            Placement = domain.Placement,
            PlacementType = domain.PlacementType.ToString()
        };
    }
}
