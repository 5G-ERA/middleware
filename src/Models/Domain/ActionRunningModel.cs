using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Models.Domain;

public class ActionRunningModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("ActionParentId")]
    public Guid ActionParentId { get; set; } // This is the Id of normal Actio from which the running action is based.

    [JsonPropertyName("ActionPlanId")]
    public Guid ActionPlanId { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }


    [JsonPropertyName("Tags")]
    public List<string>? Tags { get; set; }

    [JsonPropertyName("Order")]
    public int Order { get; set; }

    [JsonPropertyName("Placement")]
    public string? Placement { get; set; }

    [JsonPropertyName("PlacementType")]
    public string? PlacementType { get; set; } // Either edge or cloud. 

    [JsonPropertyName("ActionPriority")]
    public string? ActionPriority { get; set; }

    [JsonPropertyName("ActionStatus")]
    public string? ActionStatus { get; set; }

    [JsonPropertyName("Services")]
    //[JsonIgnore]
    public List<InstanceRunningModel>? Services { get; set; }

    [JsonPropertyName("MinimumRam")]
    public int MinimumRam { get; set; }

    [JsonPropertyName("MinimumNumCores")]
    public int MinimumNumCores { get; set; }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new ActionRunningDto()
        {
            Id = domain.Id.ToString(),
            ActionPriority = domain.ActionPriority,
            Name = domain.Name,
            ActionParentId = domain.ActionParentId.ToString(),
            ActionPlanId = domain.ActionPlanId.ToString(),
            HardwareRequirements = new HardwareRequirements()
            {
                MinimumRam = domain.MinimumRam,
                MinimumNumCores = domain.MinimumNumCores
            },
            Tags = domain.Tags
        };
    }
}
