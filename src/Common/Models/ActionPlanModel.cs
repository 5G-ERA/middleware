using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public sealed class ActionPlanModel : BaseModel
{
    [JsonPropertyName("ActionPlanId")]
    public override Guid Id { get; set; }

    [JsonPropertyName("TaskName")]
    public override string Name { get; set; }

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