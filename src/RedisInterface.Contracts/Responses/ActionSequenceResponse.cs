namespace Middleware.RedisInterface.Contracts.Responses
{
    public record ActionSequenceResponse
    {
        public string TaskName { get; set; }
        public Guid TaskId { get; set; }
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