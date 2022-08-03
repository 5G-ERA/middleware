using k8s;
using k8s.Models;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Exceptions;
using Quartz;

namespace Middleware.Orchestrator.Jobs
{
    public class MiddlewareStartupJob : BaseJob<MiddlewareStartupJob>
    {
        private readonly IKubernetesBuilder _kubeBuilder;
        private readonly IDeploymentService _deploymentService;

        public MiddlewareStartupJob(ILogger<MiddlewareStartupJob> logger, IKubernetesBuilder kubeBuilder, IDeploymentService deploymentService) : base(logger)
        {
            _kubeBuilder = kubeBuilder;
            _deploymentService = deploymentService;
        }

        protected override async Task ExecuteJobAsync(IJobExecutionContext context)
        {
            Logger.LogInformation("Executed");
            try
            {
                var client = _kubeBuilder.CreateKubernetesClient();

                await InstantiateMiddleware(client);

            }
            catch (NotInK8SEnvironmentException ex)
            {

                Logger.LogWarning(ex, "Could not instantiate the Kubernetes Client");
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
            //var namespaces = await kubeClient.CoreV1.ListNamespaceAsync();

            //var exists = namespaces.Items.Any(n => n.Name() == AppConfig.K8SNamespaceName);

            //if (exists == false)
            //{
            //    Logger.LogError("Middleware has not been deployed in the correct namespace. Correct namespace is {namespace}", AppConfig.K8SNamespaceName);
            //    return;

            //}
            var success = true;
            bool shouldDryRun = AppConfig.IsDevEnvironment();
            try
            {
                var deployments = await kubeClient.AppsV1.ListNamespacedDeploymentAsync(AppConfig.K8SNamespaceName);
                var deploymentNames = deployments.Items.Select(d => d.Metadata.Name).ToArray();
                var services = await kubeClient.CoreV1.ListNamespacedServiceAsync(AppConfig.K8SNamespaceName);
                var serviceNames = services.Items.Select(d => d.Metadata.Name).ToArray();
                var images = new List<string>
                    {"gateway", "redis-interface-api", "resource-planner-api", "task-planner-api"};

                foreach (string service in images)
                {
                    Logger.LogDebug("Started deployment of {service}", service);
                    var v1Deployment = _deploymentService.CreateStartupDeployment(service);

                    if (deploymentNames.Contains(v1Deployment.Metadata.Name))
                    {
                        Logger.LogDebug("{service} is already deployed, moved to the next service", service);
                        continue;
                    }

                    var result = await kubeClient.AppsV1.CreateNamespacedDeploymentAsync(v1Deployment, AppConfig.K8SNamespaceName,
                        dryRun: shouldDryRun ? "All" : null);
                    

                    var kind = service != "gateway" ? K8SServiceKindEnum.ClusterIp : K8SServiceKindEnum.LoadBalancer;

                    var lbService = _deploymentService.CreateService(service, kind, result.Metadata);
                    var createdService = await kubeClient.CoreV1.CreateNamespacedServiceAsync(lbService, AppConfig.K8SNamespaceName);

                    if (service == "gateway")
                    {
                        AppConfig.MiddlewareAddress = createdService.GetExternalAddress(Logger);

                        if (string.IsNullOrEmpty(AppConfig.MiddlewareAddress))
                            Logger.LogError("Could not obtain the Gateway Address");
                    }
                }
            }
            catch (k8s.Autorest.HttpOperationException httpOperationException)
            {
                var phase = httpOperationException.Response.ReasonPhrase;
                var content = httpOperationException.Response.Content;
                Logger.LogError(httpOperationException, "Unable to deploy the resource to k8s:{phase}, {content}", phase, content);
                success = false;
            }
            if (success)
            {
                Logger.LogInformation("Successfully deployed the Middleware.");
            }
        }
    }
}
