using k8s;
using k8s.Models;
using Middleware.Common;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;
using Environment = System.Environment;

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
            _awsRegistryName = Environment.GetEnvironmentVariable("AWS_IMAGE_REGISTRY"); //TODO: replace with the parameters from the command line
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

            try
            {
                var deployments = await kubeClient.ListNamespacedDeploymentAsync(K8SNamespaceName);
                var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();
                var images = new List<string>
                    {"gateway", "redis-interface-api", "resource-planner-api", "task-planner-api"};

                foreach (string service in images)
                {
                    var v1Deployment = CreateDeployment(service);

                    if (deploymentNames.Contains(v1Deployment.Metadata.Name))
                        continue;

                    var result = await kubeClient.CreateNamespacedDeploymentAsync(v1Deployment, K8SNamespaceName);
                }
            }
            catch (k8s.Autorest.HttpOperationException httpOperationException)
            {
                var phase = httpOperationException.Response.ReasonPhrase;
                var content = httpOperationException.Response.Content;
                Logger.LogError(httpOperationException, "Unable to deploy the resource to k8s:{phase}, {content}", phase, content);
            }
        }

        public V1Deployment CreateDeployment(string name)
        {
            var dnsName = CreateDnsName(name);
            var selector = new V1LabelSelector
            {
                MatchLabels = new Dictionary<string, string> { { "app", name } }
            };
            var meta = new V1ObjectMeta()
            {
                Name = dnsName,
                Labels = new Dictionary<string, string>()
                {
                    { "app", name }
                }
            };
            var envList = new List<V1EnvVar>
            {
                new ("REDIS_INTERFACE_ADDRESS", $"http://redis-interface-api"),
                new ("ORCHESTRATOR_ADDRESS", $"http://orchestrator-api"),
                new ("TASK_PLANNER_ADDRESS", $"http://task-planner-api"),
                new ("RESOURCE_PLANNER_ADDRESS", $"http://resource-planner-api")
            };
            if (name.Contains("redis"))
            {
                envList.Add(new V1EnvVar("REDIS_HOSTNAME", Environment.GetEnvironmentVariable("REDIS_HOSTNAME")));
                envList.Add(new V1EnvVar("REDIS_PORT", Environment.GetEnvironmentVariable("REDIS_PORT")));
            }

            var container = new V1Container()
            {
                Name = name,
                Image = K8SImageHelper.BuildImageName(_awsRegistryName, name, "latest"),
                ImagePullPolicy = "Always",
                Env = envList,
                Ports = new List<V1ContainerPort>() { new (80), new (433) }
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

        private string CreateDnsName(string name)
        {
            var idx = name.LastIndexOf('-');
            if (idx <=0)
            {
                return name;
            }

            var arr = name.ToCharArray();
            arr[idx] = '.';
            return new string(arr);
        }
    }
}
