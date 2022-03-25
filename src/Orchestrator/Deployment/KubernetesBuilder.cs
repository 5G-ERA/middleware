using k8s;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment
{
    public class KubernetesBuilder : IKubernetesBuilder
    {
        private const string KUBERNETES_SERVICE_HOST = "KUBERNETES_SERVICE_HOST";

        private readonly ILogger<KubernetesBuilder> _logger;

        public KubernetesBuilder(ILogger<KubernetesBuilder> logger)
        {
            _logger = logger;
        }

        /// <summary>
        /// Initializes the Kubernetes Client
        /// </summary>
        /// <returns></returns>
        public IKubernetes CreateKubernetesClient()
        {
            _logger.LogDebug("Started creation of K8s client");
            if (IsK8sEnv() == false)
            {
                _logger.LogDebug("Not in K8s environment");
                throw new NotInK8sEnvironmentException();
            }
            var config = KubernetesClientConfiguration.InClusterConfig();

            return new Kubernetes(config);
        }

        private bool IsK8sEnv()
        {
            return string.IsNullOrEmpty(Environment.GetEnvironmentVariable(KUBERNETES_SERVICE_HOST)) == false;                
        }
    }
}
