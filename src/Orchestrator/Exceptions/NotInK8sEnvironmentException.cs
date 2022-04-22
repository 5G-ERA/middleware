namespace Middleware.Orchestrator.Exceptions
{
    [Serializable]
    public class NotInK8SEnvironmentException : Exception
    {
        public NotInK8SEnvironmentException() : base("The environment is not a Kubernetes environment, cannot instantiate the Middleware")
        {

        }
    }
}
