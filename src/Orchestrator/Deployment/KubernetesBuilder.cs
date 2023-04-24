using k8s;
using Middleware.Common;
using Middleware.Orchestrator.Exceptions;

namespace Middleware.Orchestrator.Deployment
{
    public class KubernetesBuilder : IKubernetesBuilder
    {
        public static string ServiceAccountPath =
            Path.Combine(new string[]
            {
                $"{Path.DirectorySeparatorChar}var", "run", "secrets", "kubernetes.io", "serviceaccount",
            });
        internal static string KubeConfigPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".kube", "config");
        internal const string ServiceAccountTokenKeyFileName = "token";
        internal const string ServiceAccountRootCAKeyFileName = "ca.crt";
        internal const string ServiceAccountNamespaceFileName = "namespace";

        private const string KubernetesServiceHost = "KUBERNETES_SERVICE_HOST";
        private const string KubernetesServicePort = "KUBERNETES_SERVICE_PORT";

        private readonly ILogger<KubernetesBuilder> _logger;
        private readonly IEnvironment _env;

        public KubernetesBuilder(ILogger<KubernetesBuilder> logger, IEnvironment env)
        {
            _logger = logger;
            _env = env;
        }

        /// <inheritdoc />
        public IKubernetes CreateKubernetesClient()
        {
            _logger.LogTrace("Started creation of K8s client");

            var isValidInCLusterConfig = IsValidInClusterConfig();
            if (isValidInCLusterConfig == false && _env.FileExists(KubeConfigPath) == false)
            {
                _logger.LogDebug("Not in K8s environment");
                throw new NotInK8SEnvironmentException();
            }

            var config = isValidInCLusterConfig
                ? KubernetesClientConfiguration.InClusterConfig()
                : KubernetesClientConfiguration.BuildConfigFromConfigFile(KubeConfigPath);

            return new Kubernetes(config);
        }

        /// <inheritdoc />
        public bool IsValidInClusterConfig()
        {
            var retVal = IsK8SEnv();

            var exists = _env.DirectoryExists(ServiceAccountPath);

            _logger.LogTrace("K8s ServiceAccountPath exists {exists}", exists);

            if (retVal == false)
            {
                return false;
            }

            var fileNames = _env.GetFileNamesInDir(ServiceAccountPath);

            var filesExist = fileNames.Contains(ServiceAccountTokenKeyFileName)
                             && fileNames.Contains(ServiceAccountRootCAKeyFileName)
                             && fileNames.Contains(ServiceAccountNamespaceFileName);

            _logger.LogTrace("K8s ServiceAccount files exist {filesExist}", filesExist);
            retVal &= exists && filesExist;

            return retVal;

        }

        private bool IsK8SEnv()
        {
            var host = _env.GetEnvVariable(KubernetesServiceHost);
            var port = _env.GetEnvVariable(KubernetesServicePort);
            _logger.LogTrace("K8s {env} has value {value}", KubernetesServiceHost, host);
            _logger.LogTrace("K8s {env} has value {value}", KubernetesServicePort, port);

            return string.IsNullOrEmpty(host) == false && string.IsNullOrEmpty(port) == false;
        }
    }
}
