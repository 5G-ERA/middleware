using System.Text.Json.Serialization;

namespace Middleware.Models.Domain;

public class ActionModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("Tags")]
    public List<string> Tags { get; set; }

    [JsonPropertyName("Order")]
    public int Order { get; set; }

    [JsonPropertyName("Placement")]
    public string Placement { get; set; }

    [JsonPropertyName("PlacementType")]
    public string PlacementType { get; set; } // Either edge or cloud. 

    [JsonPropertyName("ActionPriority")]
    public string ActionPriority { get; set; }

    [JsonPropertyName("ActionStatus")]
    public string ActionStatus { get; set; }

    [JsonPropertyName("Services")]
    //[JsonIgnore]
    public List<InstanceModel> Services { get; set; }

    [JsonPropertyName("MinimumRam")]
    public int MinimumRam { get; set; }

    [JsonPropertyName("MinimumNumCores")]
    public int MinimumNumCores { get; set; }
}