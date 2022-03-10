using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace Middleware.Common.Models
{
    public class ExecutedTasksModel
    {
        [JsonPropertyName("RobotID")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }
    }
}
