using System.Text.Json.Serialization;

namespace Middleware.Models.Domain
{
    public class TaskPlannerInputModel
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("LockResourceReUse")]
        public bool LockResourceReUse { get; set; }

        [JsonPropertyName("ReplanActionPlannerLocked")]
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