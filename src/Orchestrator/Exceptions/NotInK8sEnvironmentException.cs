namespace Middleware.Orchestrator.Exceptions
{
    [Serializable]
    public class NotInK8SEnvironmentException : Exception
    {
        public const string DefaultMessage = "The environment is not a Kubernetes environment, cannot instantiate the Middleware";
        public NotInK8SEnvironmentException() : base(DefaultMessage)
        {

        }
    }
}
