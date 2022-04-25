using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskModel : BaseModel
    {
        [JsonPropertyName("Id")]
        public override Guid Id { get; set; }

        [JsonPropertyName("TaskName")]
        public override string Name { get; set; }

        [JsonPropertyName("TaskPriority")]
        public int TaskPriority { get; set; }

        [JsonPropertyName("ActionPlanId")]
        public Guid ActionPlanId { get; set; }

        [JsonPropertyName("ActionSequence")]
        public List<ActionModel> ActionSequence { get; set; }

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