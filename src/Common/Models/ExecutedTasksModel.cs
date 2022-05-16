using System.Text.Json.Serialization;

namespace Middleware.Common.Models
{
    public class ExecutedTasksModel
    {
        [JsonPropertyName("Id")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid Id { get; set; }
    }
}
