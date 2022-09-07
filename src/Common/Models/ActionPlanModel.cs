using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public sealed class ActionPlanModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("Status")] //Status of whole plan
    public string Status { get; set; }

    [JsonPropertyName("LastStatusChange")]
    public DateTime LastStatusChange { get; set; }

    [JsonPropertyName("ActionSequence")]
    public List<ActionModel> ActionSequence { get; set; }

    public ActionPlanModel()
    {
        
    }

    public ActionPlanModel(Guid id, string name, List<ActionModel> actionSequence)
    {
        Id = id;
        Name = name;
        ActionSequence = actionSequence;
    }
    
}