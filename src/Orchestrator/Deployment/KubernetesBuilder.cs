using k8s;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment
{
    public class KubernetesBuilder : IKubernetesBuilder
    {
        private const string KubernetesServiceHost = "KUBERNETES_SERVICE_HOST";
        private const string KubernetesServicePort = "KUBERNETES_SERVICE_PORT";

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
            var config = KubernetesClientConfiguration.BuildDefaultConfig();
            //var config2 = KubernetesClientConfiguration.
            return new Kubernetes(config);
        }

        private bool IsK8sEnv()
        {
            var host = Environment.GetEnvironmentVariable(KubernetesServiceHost);
            var port = Environment.GetEnvironmentVariable(KubernetesServicePort);
            return string.IsNullOrEmpty(host) == false && string.IsNullOrEmpty(port) == false;
        }
    }
}
