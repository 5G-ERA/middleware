using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskPlannerInputModel
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("LockResourceReUse")]

        public bool LockResourceReUse { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }
    }
}
