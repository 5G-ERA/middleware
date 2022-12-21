using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Responses
{
    public record actionSequenceResponse
    {
        [JsonPropertyName("TaskName")]
        public string TaskName { get; set; }

        [JsonPropertyName("TaskId")]
        public Guid TaskId { get; set; }

        [JsonPropertyName("Actions")]
        public List<string> Actions { get; set; }

        public actionSequenceResponse()
        {

        }

        public actionSequenceResponse(string taskName, Guid taskId, List<string> actions)
        {
            TaskName = taskName;
            TaskId = taskId;
            Actions = actions;
        }
    }

}
