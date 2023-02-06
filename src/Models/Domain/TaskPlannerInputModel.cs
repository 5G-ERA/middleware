using System.Text.Json.Serialization;

namespace Middleware.Models.Domain
{
    public class TaskPlannerInputModel
    {
        [JsonPropertyName("RobotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("LockResourceReUse")]
        public bool LockResourceReUse { get; set; } //TODO: check only the instances that are avialable in the local middleware
        /// <summary>
        /// only change the instance placement in the existing plan during a replan
        /// </summary>
        [JsonPropertyName("ReplanActionPlannerLocked")]
        public bool ReplanActionPlannerLocked { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }

        [JsonPropertyName("TaskDescription")]
        public String TaskDescription { get; set; }
        
        /// <summary>
        /// Use the hardcoded action plan if true, use the semantic planning if false
        /// </summary>        
        [JsonPropertyName("ContextKnown")]
        public bool ContextKnown { get; set; } = true; 

        [JsonPropertyName("Questions")]
        public List<DialogueModel> Questions { get; set; }

        public bool IsValid()
        {
            return RobotId != Guid.Empty && Id != Guid.Empty;
        }
    }
}