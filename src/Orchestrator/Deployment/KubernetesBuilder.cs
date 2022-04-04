using k8s;
using Middleware.Common;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment
{
    public class KubernetesBuilder : IKubernetesBuilder
    {
        private const string KubernetesServiceHost = "KUBERNETES_SERVICE_HOST";
        private const string KubernetesServicePort = "KUBERNETES_SERVICE_PORT";

        private readonly ILogger<KubernetesBuilder> _logger;
        private readonly IEnvironment _env;

        public KubernetesBuilder(ILogger<KubernetesBuilder> logger, IEnvironment env)
        {
            _logger = logger;
            _env = env;
        }

        /// <summary>
        /// Initializes the Kubernetes Client
        /// </summary>
        /// <returns></returns>
        public IKubernetes CreateKubernetesClient()
        {
            _logger.LogDebug("Started creation of K8s client");
            if (IsK8SEnv() == false)
            {
                _logger.LogDebug("Not in K8s environment");
                throw new NotInK8sEnvironmentException();
            }
            var config = KubernetesClientConfiguration.BuildDefaultConfig();
            //var config2 = KubernetesClientConfiguration.
            return new Kubernetes(config);
        }

        private bool IsK8SEnv()
        {
            var host = _env.GetEnvVariable(KubernetesServiceHost);
            var port = _env.GetEnvVariable(KubernetesServicePort);
            return string.IsNullOrEmpty(host) == false && string.IsNullOrEmpty(port) == false;
        }
    }
}
