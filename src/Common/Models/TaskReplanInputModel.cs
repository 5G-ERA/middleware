using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskReplanInputModel
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("LockResourceReUse")]
        public bool LockResourceReUse { get; set; }

        [JsonPropertyName("ContextKnown")]
        public bool ContextKnown { get; set; }

        [JsonPropertyName("CompleteReplan")]
        public bool CompleteReplan { get; set; }

        [JsonPropertyName("TaskID")]
        public Guid TaskID { get; set; }

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }

    }
}
