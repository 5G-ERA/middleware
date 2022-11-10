namespace Middleware.TaskPlanner.Exceptions
{
    [Serializable]
    public class NotARobotTopicException : Exception
    {
        public string Reason { get; private set; }
        public NotARobotTopicException(string reason) : base($"The robot does not have the topic.{reason}")
        {
            Reason = reason;
        }
    }
}
