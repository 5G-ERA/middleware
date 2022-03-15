using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class TaskModel : BaseModel
    {
        [JsonPropertyName("TaskId")]
        public override Guid Id { get; set; }

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