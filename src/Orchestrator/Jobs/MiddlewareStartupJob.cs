using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
    {
        private const string K8SNamespaceName = "middleware";
        private readonly string _awsRegistryName;

        private readonly IKubernetesBuilder _kubeBuilder;

        public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder) : base(logger)
        {
            _kubeBuilder = kubeBuilder;
            _awsRegistryName = Environment.GetEnvironmentVariable("AWS_IMAGE_REGISTRY"); //TODO: replace with the ENV Variable
        }

        protected override async Task ExecuteJobAsync(IJobExecutionContext context)
        {
            Logger.LogInformation("Executed");
            try
            {
                var client = _kubeBuilder.CreateKubernetesClient();

                await InstantiateMiddleware(client);

            }
            catch (NotInK8sEnvironmentException ex)
            {
                Logger.LogWarning("Could not instantiate the Kubernetes Client", ex);
                throw;
            }
        }

        /// <summary>
        /// Instantiates the pods for the middleware
        /// </summary>
        /// <param name="kubeClient"></param>
        /// <returns></returns>
        private async Task InstantiateMiddleware(IKubernetes kubeClient)
        {
            var namespaces = await kubeClient.ListNamespaceAsync();

            var exists = namespaces.Items.Any(n => n.Name() == K8SNamespaceName);

            if (exists == false)
                return;

            var dV1Deployment = CreateDeployment("gateway");

            try
            {
                var result = await kubeClient.CreateNamespacedDeploymentAsync(dV1Deployment, K8SNamespaceName);

            }
            catch (k8s.Autorest.HttpOperationException httpOperationException)
            {
                var phase = httpOperationException.Response.ReasonPhrase;
                var content = httpOperationException.Response.Content;
            }

        }

        public V1Deployment CreateDeployment(string name)
        {
            var selector = new V1LabelSelector
            {
                MatchLabels = new Dictionary<string, string> { { "app", name } }
            };
            var meta = new V1ObjectMeta()
            {
                Name = name,
                Labels = new Dictionary<string, string>()
                {
                    { "app", name }
                }
            };

            var container = new V1Container()
            {
                Name = name,
                Image = K8SImageHelper.BuildImageName(_awsRegistryName, name, "latest"),
                ImagePullPolicy = "Always",
                Env = new List<V1EnvVar>()
                {
                    new V1EnvVar("REDIS_INTERFACE_ADDRESS", $"http://redisinterface.api"),
                    new V1EnvVar("ORCHESTRATOR_ADDRESS", $"http://orchestrator.api"),
                    new V1EnvVar("TASK_PLANNER_ADDRESS", $"http://taskplanner.api"),
                    new V1EnvVar("RESOURCE_PLANNER_ADDRESS", $"http://resourceplanner.api")
                },
                Ports = new List<V1ContainerPort>(){new V1ContainerPort(80), new V1ContainerPort(433)}
            };
            var podSpec = new V1PodSpec(new List<V1Container>() { container });

            var template = new V1PodTemplateSpec(meta, podSpec);
            var spec = new V1DeploymentSpec(selector, template);

            var dep = new V1Deployment
            {
                Metadata = meta,
                Spec = spec,
                ApiVersion = "apps/v1",
                Kind = "Deployment"
            };

            return dep;
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="imageName"></param>
        /// <returns></returns>
        private static string NameToUrlString(string imageName)
        {
            return imageName.Replace('-', '.');
        }
    }
}
