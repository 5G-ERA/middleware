using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public sealed class ActionPlanModel : BaseModel
{
    [JsonPropertyName("Id")] //atuomatically generated plan id by middleware
    public override Guid Id { get; set; }

    [JsonPropertyName("TaskId")] //TaskID
    public Guid TaskId { get; set; }

    [JsonPropertyName("Name")] // Name of task
    public override string Name { get; set; }

    [JsonPropertyName("Status")] //Status of whole plan
    public string Status
    {
        get { return Status; }
        set
        {
            Status = value;           
        }
    }

    [JsonPropertyName("IsReplan")] //Status of whole plan
    public bool IsReplan { get; set; }

    [JsonPropertyName("LastStatusChange")]
    public DateTime LastStatusChange { get; set; } // AL 2022-05-10: Not sure we need this one or how to use it.

    [JsonPropertyName("ActionSequence")]
    public List<ActionModel> ActionSequence { get; set; }

    [JsonPropertyName("RobotId")]
    public Guid RobotId { get; set; }

    public ActionPlanModel()
    {

    }

    public ActionPlanModel(Guid id, string name, List<ActionModel> actionSequence, Guid robotId)
    {
        Id = id;
        Name = name;
        ActionSequence = actionSequence;
        RobotId = robotId;
    }
    public void SetStatus(string status)
    {
        Status = status;
        LastStatusChange = DateTime.UtcNow;
    }
}