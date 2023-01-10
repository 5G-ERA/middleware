using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskPlannerInputModel
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("LockResourceReUse")] // The middleware may not reuse containers from this plan
        public bool LockResourceReUse { get; set; }

        [JsonPropertyName("ReplanActionPlannerLocked")] // The middleware may only change the resource allocation and not the action seq semantics.
        public bool ReplanActionPlannerLocked { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }

        [JsonPropertyName("TaskDescription")]
        public String TaskDescription { get; set; }

        [JsonPropertyName("ContextKnown")]
        public bool ContextKnown { get; set; }

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }
    }
}
