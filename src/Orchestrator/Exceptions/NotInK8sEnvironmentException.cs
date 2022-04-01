namespace Middleware.Orchestrator.Exceptions
{
    [Serializable]
    public class NotInK8sEnvironmentException : Exception
    {
        public NotInK8sEnvironmentException() : base("The environment is not a Kubernetes environment, cannot instantiate the Middleware")
        {

        }
    }
}
