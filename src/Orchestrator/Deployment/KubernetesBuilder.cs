using k8s;
using Middleware.Common;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment
{
    public class KubernetesBuilder : IKubernetesBuilder
    {
        internal static string ServiceAccountPath =
            Path.Combine(new string[]
            {
                $"{Path.DirectorySeparatorChar}var", "run", "secrets", "kubernetes.io", "serviceaccount",
            });
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
                throw new NotInK8SEnvironmentException();
            }

            var kubeConfigPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".kube", "config");
            var config = AppConfig.AppConfiguration == "Development" 
                ? KubernetesClientConfiguration.BuildConfigFromConfigFile(kubeConfigPath) 
                : KubernetesClientConfiguration.InClusterConfig();
            
            return new Kubernetes(config);
        }

        private bool IsK8SEnv()
        {
            var host = _env.GetEnvVariable(KubernetesServiceHost);
            var port = _env.GetEnvVariable(KubernetesServicePort);
            _logger.LogDebug("K8s {env} has value {value}", KubernetesServiceHost, host);
            _logger.LogDebug("K8s {env} has value {value}", KubernetesServicePort, port);
            var exists = Directory.Exists(ServiceAccountPath);
            _logger.LogDebug("K8s ServiceAccoutPath exists {exists}", exists);
            if (exists)
            {
                var files = Directory.GetFiles(ServiceAccountPath);
                _logger.LogDebug("Files present in the ServiceAccountPath: {files}", string.Join(", ", files));
            }
            
            return string.IsNullOrEmpty(host) == false && string.IsNullOrEmpty(port) == false;
        }
    }
}
