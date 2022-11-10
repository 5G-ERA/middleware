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

        public List<RosTopicModel> InputTopics { get; set; } // =  new() { "RGB Camera", "Point Cloud", "voice" };
        /// <summary>
        /// The topics that  robot will subscribe the output to 
        /// </summary>
        public List<RosTopicModel> OutputTopics { get; set; }

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }

    }
}
