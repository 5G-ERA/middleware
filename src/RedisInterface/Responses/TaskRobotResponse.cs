using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Responses
{
    public record TaskRobotResponse
    {
        [JsonPropertyName("robotName")]
        public string RobotName { get; set; }

        [JsonPropertyName("robotId")]
        public Guid RobotId { get; set; }

        [JsonPropertyName("taskName")]
        public string TaskName { get; set; }

        [JsonPropertyName("taskId")]
        public Guid TaskId { get; set; }

        [JsonPropertyName("taskStartTime")]
        public DateTime TaskStartTime { get; set; }

        [JsonPropertyName("taskCompletedTime")]
        public DateTime? TaskCompletedTime { get; set; }

        [JsonPropertyName("RobotStatus")]
        public string RobotStatus { get; set; }

        public TaskRobotResponse()
        {

        }
        public TaskRobotResponse(Guid robotId, string robotName, Guid taskId, string taskName, DateTime taskStartTime, DateTime? taskCompletedTime, string robotStatus)
        {
            RobotId = robotId;
            RobotName = robotName;
            TaskId = taskId;
            TaskName = taskName;
            TaskStartTime = taskStartTime;
            TaskCompletedTime = taskCompletedTime;
            RobotStatus = robotStatus;
        }
    }
}
