using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class TaskModel
    {
        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }

        [JsonPropertyName("TaskPriority")]
        public string TaskPriority { get; set; }

        [JsonPropertyName("ActionPlanId")]
        public Guid ActionPlanId { get; set; }

        [JsonPropertyName("ActionSequence")]
        public List<ActionModel> ActionSequence { get; set; }
    }
}