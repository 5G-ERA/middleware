using System.Text.Json.Serialization;

namespace Middleware.RedisInterface.Contracts.Responses
{
    public record ActionSequenceResponse
    {
        [JsonPropertyName("taskName")]
        public string TaskName { get; set; }

        [JsonPropertyName("taskId")]
        public Guid TaskId { get; set; }

        [JsonPropertyName("actions")]
        public List<string> Actions { get; set; }

        public ActionSequenceResponse()
        {

        }

        public ActionSequenceResponse(string taskName, Guid taskId, List<string> actions)
        {
            TaskName = taskName;
            TaskId = taskId;
            Actions = actions;
        }
    }

}
