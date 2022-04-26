using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class ActionPlanModel : BaseModel
{
    [JsonPropertyName("ActionPlanId")]
    public override Guid Id { get; set; }

    [JsonPropertyName("TaskName")]
    public override string Name { get; set; }

    [JsonPropertyName("ActionSequence")]
    public List<ActionModel> ActionSequence { get; set; }
    
}