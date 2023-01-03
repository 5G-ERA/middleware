﻿using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskModel : BaseModel
    {
        [JsonPropertyName("Id")]
        public override Guid Id { get; set; }

        [JsonPropertyName("Name")]
        public override string Name { get; set; }

        [JsonPropertyName("ReplanActionPlannerLocked")] // True: The robot requests to not change anything from action sequence but placement.
        public bool ReplanActionPlannerLocked { get; set; } //True: Dont change actions in action sequence replan

        [JsonPropertyName("ResourceLock")]
        public bool ResourceLock { get; set; } //Avoid reusage of resources always.

        [JsonPropertyName("TaskPriority")]
        public int TaskPriority { get; set; }

        [JsonPropertyName("ActionPlanId")]
        public Guid ActionPlanId { get; set; } //Automatically generated by middleware

        [JsonPropertyName("FullReplan")]
        public bool FullReplan { get; set; }

        [JsonPropertyName("PartialRePlan")]
        public bool PartialRePlan { get; set; }

        [JsonPropertyName("DeterministicTask")]
        public bool DeterministicTask { get; set; } //No randomness in the task nature. 

        [JsonPropertyName("MarkovianProcess")]
        public bool MarkovianProcess { get; set; } //Check if  actions in action sequence are sequential or affect each other.

        [JsonPropertyName("ActionSequence")]
        //[JsonIgnore]
        public List<ActionModel> ActionSequence { get; set; }

        [JsonPropertyName("Tags")] //TODO: define allows tags
        //[JsonIgnore]
        public List<string> Tags { get; set; }


        public TaskModel()
        {

        }
        public TaskModel(Guid id, int priority)
        {
            Id = id;
            TaskPriority = priority;
        }
    }
}
