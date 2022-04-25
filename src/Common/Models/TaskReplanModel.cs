using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class TaskReplanModel : BaseModel
    {
        [JsonPropertyName("Id")]
        public override Guid Id { get; set; }

        [JsonPropertyName("TaskReplanName")]
        public override string Name { get; set; }

        [JsonPropertyName("ActionPlanId")]
        public Guid ActionPlanId { get; set; }

        [JsonPropertyName("ActionSequence")]
        public List<ActionModel> ActionSequence { get; set; }

        public TaskReplanModel(Guid taskId)
        {
            Id = taskId;
            
        }

    }
}
