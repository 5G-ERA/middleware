namespace Middleware.ResourcePlanner.Exceptions
{
    [Serializable]
    public class TopicTypeNotAvailableInRobot : Exception
    {
        public string Reason { get; private set; }
        public TopicTypeNotAvailableInRobot(string reason) : base($"The robot does not have the topic type requried.{reason}")
        {
            Reason = reason;
        }
    }
}
