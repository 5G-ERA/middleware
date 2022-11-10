using System.Text.Json.Serialization;

namespace Middleware.Common.Models
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
        
        /// <summary>
        /// The topics that the robot will publish an input to
        /// </summary>
        public List<RosTopicModel> InputTopics { get; set; } // =  new() { "RGB Camera", "Point Cloud", "voice" };
        /// <summary>
        /// The topics that  robot will subscribe the output to 
        /// </summary>
        public List<RosTopicModel> OutputTopics { get; set; } //= new() { "string", "sadkjfgakjsdas" };

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }
    }
}
